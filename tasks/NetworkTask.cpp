#include "NetworkTask.h"

#include "JointLimits.h"
#include "Kinematics.h"
#include "Logger.h"
#include "MessageSchema.h"
#include "MotionTask.h"
#include "WebServerManager.h"
#include "WebSocketHandler.h"
#include "WiFiSetup.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

namespace Tasks {

// -------------------------------------------------------------------------
// Module-level state
// -------------------------------------------------------------------------
namespace {

Comms::WebServerManager gHttpServer;
Comms::WebSocketHandler gSocket;
std::atomic<bool> gInitialized{false};
bool gLastSocketConnected = false;
QueueHandle_t gProgressQueue = nullptr;
SemaphoreHandle_t gSocketMutex = nullptr;
TaskHandle_t gTaskHandle = nullptr;

Control::StateMachine* gStateMachine = nullptr;
SemaphoreHandle_t gStateMutex = nullptr;

NetworkTask::ControlRequestCb gControlRequestCb = nullptr;
NetworkTask::PayloadQueryCb gPayloadQueryCb = nullptr;
NetworkTask::LinkStatusCb gLinkStatusCb = nullptr;
NetworkTask::LinkActivityCb gLinkActivityCb = nullptr;

constexpr uint32_t kTaskStackWords = 8192;
constexpr UBaseType_t kTaskPriority = 1;

struct ProgressUpdate {
    float progress;
    float theta0;
    float theta1;
    float theta2;
    float theta3;
    uint32_t elapsedMs;
    uint32_t durationMs;
};

// Last commanded gripper state (tracks what the user last requested)
Control::GripperPose gPendingGripper = Control::GripperPose::Open;

// Current joint angles for trajectory start positions.
// Updated by main.cpp via MotionTask::currentAngles().
Motion::JointVector gLastKnownAngles = {
    JointLimits::kTheta0Home_rad,
    JointLimits::kTheta1Home_rad,
    JointLimits::kTheta2Home_rad,
    JointLimits::kTheta3Home_rad,
    JointLimits::kGripperHome_rad,
};

class StateGuard {
 public:
    explicit StateGuard(SemaphoreHandle_t mutex)
        : mutex_(mutex),
          locked_(mutex_ != nullptr &&
                  xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE) {}
    ~StateGuard() {
        if (locked_) {
            xSemaphoreGive(mutex_);
        }
    }
    bool locked() const { return locked_; }

 private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

class SocketGuard {
 public:
    SocketGuard()
        : locked_(gSocketMutex != nullptr &&
                  xSemaphoreTakeRecursive(gSocketMutex, portMAX_DELAY) == pdTRUE) {}
    ~SocketGuard() {
        if (locked_) {
            xSemaphoreGiveRecursive(gSocketMutex);
        }
    }
    bool locked() const { return locked_; }

 private:
    bool locked_;
};

}  // namespace

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

bool NetworkTask::begin() {
    if (gInitialized.load(std::memory_order_acquire)) {
        return true;
    }

    gProgressQueue = xQueueCreate(1, sizeof(ProgressUpdate));
    gSocketMutex = xSemaphoreCreateRecursiveMutex();
    if (gProgressQueue == nullptr || gSocketMutex == nullptr ||
        gStateMutex == nullptr) {
        LOG_ERROR("NetworkTask: failed to allocate task resources");
        if (gProgressQueue != nullptr) {
            vQueueDelete(gProgressQueue);
            gProgressQueue = nullptr;
        }
        if (gSocketMutex != nullptr) {
            vSemaphoreDelete(gSocketMutex);
            gSocketMutex = nullptr;
        }
        return false;
    }

    Comms::WiFiSetup::begin("RoboticArmAP", "roboticarm");
    gHttpServer.begin();
    gSocket.begin();
    gSocket.setMessageHandler(&NetworkTask::onSocketMessage);

    // Pre-populate the /api/limits endpoint with the arm's reach envelope
    // (|L1 − L2| to L1 + L2, not including L3 since that's the fingertip offset)
    gHttpServer.setLimitsPayload(
        "empty",
        JointLimits::kL1_cm - JointLimits::kL2_cm,   // min wrist-center reach
        JointLimits::kL1_cm + JointLimits::kL2_cm);   // max wrist-center reach

    gInitialized.store(true, std::memory_order_release);

    const BaseType_t created = xTaskCreatePinnedToCore(
        NetworkTask::taskEntry,
        "NetworkTask",
        kTaskStackWords,
        nullptr,
        kTaskPriority,
        &gTaskHandle,
        0);
    if (created != pdPASS) {
        gInitialized.store(false, std::memory_order_release);
        LOG_ERROR("NetworkTask: failed to start FreeRTOS task");
        vQueueDelete(gProgressQueue);
        vSemaphoreDelete(gSocketMutex);
        gProgressQueue = nullptr;
        gSocketMutex = nullptr;
        return false;
    }

    LOG_INFO("NetworkTask started on core 0");
    return true;
}

void NetworkTask::loop() {
    if (!gInitialized.load(std::memory_order_acquire)) {
        return;
    }

    SocketGuard socketGuard;
    if (!socketGuard.locked()) {
        return;
    }

    gHttpServer.handleClient();
    gSocket.handleClient();

    const bool socketConnected = gSocket.connected();
    if (socketConnected && !gLastSocketConnected) {
        if (gLinkStatusCb != nullptr) {
            gLinkStatusCb(true);
        }
    } else if (!socketConnected && gLastSocketConnected &&
               gLinkStatusCb != nullptr) {
        gLinkStatusCb(false);
    }
    gLastSocketConnected = socketConnected;

    ProgressUpdate update{};
    if (xQueueReceive(gProgressQueue, &update, 0) == pdTRUE) {
        gSocket.sendTrajectoryProgress(
            update.progress, update.theta0, update.theta1, update.theta2,
            update.theta3, update.elapsedMs, update.durationMs);
    }

    // Keep current angles up-to-date for trajectory start positions
    gLastKnownAngles = MotionTask::currentAngles();
}

void NetworkTask::taskEntry(void* /*param*/) {
    LOG_INFO("NetworkTask: task running on core %d", xPortGetCoreID());
    for (;;) {
        NetworkTask::loop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void NetworkTask::publishState(const char* state,
                               bool homed,
                               const char* payloadProfile,
                               const char* gripper) {
    if (!gInitialized.load(std::memory_order_acquire)) {
        return;
    }
    SocketGuard socketGuard;
    if (!socketGuard.locked()) {
        return;
    }
    gHttpServer.setStatusPayload(state, homed);
    gSocket.sendStateUpdate(state, homed, payloadProfile, gripper);
}

void NetworkTask::publishError(const char* code, const char* message) {
    if (!gInitialized.load(std::memory_order_acquire)) {
        return;
    }
    SocketGuard socketGuard;
    if (!socketGuard.locked()) {
        return;
    }
    gHttpServer.setLastError(message);
    gSocket.sendError(code, message);
}

void NetworkTask::publishTrajectoryProgress(float progress,
                                            float theta0,
                                            float theta1,
                                            float theta2,
                                            float theta3,
                                            uint32_t elapsedMs,
                                            uint32_t durationMs) {
    if (!gInitialized.load(std::memory_order_acquire)) {
        return;
    }
    const ProgressUpdate update = {
        progress, theta0, theta1, theta2, theta3, elapsedMs, durationMs};
    xQueueOverwrite(gProgressQueue, &update);
}

void NetworkTask::publishIkSolution(const Kinematics::IKResult& ik) {
    if (!gInitialized.load(std::memory_order_acquire)) {
        return;
    }
    if (ik.elbow_up.error == Kinematics::ErrorCode::INVALID_INPUT ||
        ik.elbow_down.error == Kinematics::ErrorCode::INVALID_INPUT) {
        publishError("INVALID_INPUT", "IK inputs must be finite numbers");
        return;
    }
    SocketGuard socketGuard;
    if (!socketGuard.locked()) {
        return;
    }

    // Error code to string for the JSON
    const char* upErr = nullptr;
    const char* downErr = nullptr;
    if (ik.elbow_up.error == Kinematics::ErrorCode::TARGET_UNREACHABLE) {
        upErr = "TARGET_UNREACHABLE";
    } else if (ik.elbow_up.error == Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED) {
        upErr = "JOINT_LIMIT_EXCEEDED";
    }
    if (ik.elbow_down.error == Kinematics::ErrorCode::TARGET_UNREACHABLE) {
        downErr = "TARGET_UNREACHABLE";
    } else if (ik.elbow_down.error == Kinematics::ErrorCode::JOINT_LIMIT_EXCEEDED) {
        downErr = "JOINT_LIMIT_EXCEEDED";
    }

    // Build and broadcast the full ik_solution message (spec §7.3).
    // The existing buildIkSolution helper includes input, both postures,
    // derivation steps, and validity flags.
    gSocket.broadcast(
        Comms::buildIkSolution(
            ik.input.x, ik.input.y, ik.input.z,
            JointLimits::radToDeg(ik.input.phi),
            // elbow_up
            JointLimits::radToDeg(ik.elbow_up.theta0),
            JointLimits::radToDeg(ik.elbow_up.theta1),
            JointLimits::radToDeg(ik.elbow_up.theta2),
            JointLimits::radToDeg(ik.elbow_up.theta3),
            ik.elbow_up.valid, upErr,
            // elbow_down
            JointLimits::radToDeg(ik.elbow_down.theta0),
            JointLimits::radToDeg(ik.elbow_down.theta1),
            JointLimits::radToDeg(ik.elbow_down.theta2),
            JointLimits::radToDeg(ik.elbow_down.theta3),
            ik.elbow_down.valid, downErr,
            ik.steps, ik.step_count));
}

bool NetworkTask::connected() {
    SocketGuard socketGuard;
    if (!socketGuard.locked()) {
        return false;
    }
    return gSocket.connected();
}

void NetworkTask::attachStateMachine(Control::StateMachine* sm) {
    gStateMachine = sm;
}

void NetworkTask::attachStateMutex(SemaphoreHandle_t mutex) {
    gStateMutex = mutex;
}

void NetworkTask::setLinkStatusCallback(LinkStatusCb cb) {
    gLinkStatusCb = cb;
}

void NetworkTask::setLinkActivityCallback(LinkActivityCb cb) {
    gLinkActivityCb = cb;
}

void NetworkTask::setControlRequestCallback(ControlRequestCb cb) {
    gControlRequestCb = cb;
}

void NetworkTask::setPayloadQueryCallback(PayloadQueryCb cb) {
    gPayloadQueryCb = cb;
}

// -------------------------------------------------------------------------
// Command flow: move_command → IK → trajectory → queue to MotionTask
// -------------------------------------------------------------------------

void NetworkTask::handleIkPreview(const String& message) {
    Comms::MoveCommand cmd;
    if (!Comms::parseIkPreview(message, cmd)) {
        publishError("BAD_COMMAND", "Invalid IK preview payload");
        return;
    }

    Kinematics::IKInput input;
    input.x = cmd.x;
    input.y = cmd.y;
    input.z = cmd.z;
    input.phi = JointLimits::degToRad(cmd.phi);
    publishIkSolution(Kinematics::solveIK(input));
}

void NetworkTask::handleMoveCommand(const String& message) {
    // Parse the JSON command
    Comms::MoveCommand cmd;
    if (!Comms::parseMoveCommand(message, cmd)) {
        publishError("BAD_COMMAND", "Invalid move command payload");
        return;
    }

    // Check state machine accepts a move
    if (gStateMachine == nullptr) {
        publishError("INTERNAL", "State machine not attached");
        return;
    }

    const bool recoveringCommandError =
        gStateMachine->state() == Control::SystemState::Error &&
        gStateMachine->homed() && !gStateMachine->estopLatched() &&
        gStateMachine->linkUp();
    if (!gStateMachine->acceptsMove() && !recoveringCommandError) {
        publishError("BAD_COMMAND",
                     "Move rejected: system not in IDLE+homed state");
        return;
    }

    // Already moving? Reject (spec says one command at a time)
    if (MotionTask::isMoving()) {
        publishError("BAD_COMMAND", "Move rejected: motion in progress");
        return;
    }

    // ---- Step 1: Run IK (cheap, analytical — spec §10 says inline) ----
    Kinematics::IKInput ikInput;
    ikInput.x = cmd.x;
    ikInput.y = cmd.y;
    ikInput.z = cmd.z;
    // phi comes in degrees from the API; convert to radians internally
    ikInput.phi = JointLimits::degToRad(cmd.phi);

    const Kinematics::IKResult ik = Kinematics::solveIK(ikInput);

    if (ik.elbow_up.error == Kinematics::ErrorCode::INVALID_INPUT) {
        publishError("INVALID_INPUT", "IK inputs must be finite numbers");
        return;
    }

    // Always broadcast the IK solution so the web app can visualize both
    // postures and the step-by-step derivation (spec §3.4, §7.3)
    publishIkSolution(ik);

    // ---- Step 2: Reachability check ----
    if (!ik.reachable) {
        publishError("TARGET_UNREACHABLE",
                     "Target position is outside the arm's reachable workspace");
        gStateMachine->fault(Control::FaultCode::TargetUnreachable);
        return;
    }

    // ---- Step 3: Select the user-requested posture ----
    const Kinematics::PostureSolution* chosen = nullptr;
    if (cmd.posture == "elbow_up") {
        chosen = &ik.elbow_up;
    } else {
        chosen = &ik.elbow_down;
    }

    if (!chosen->valid) {
        // Try the other posture
        const Kinematics::PostureSolution* other =
            (chosen == &ik.elbow_up) ? &ik.elbow_down : &ik.elbow_up;

        const char* errCode = "JOINT_LIMIT_EXCEEDED";
        if (chosen->error == Kinematics::ErrorCode::TARGET_UNREACHABLE) {
            errCode = "TARGET_UNREACHABLE";
        }

        if (other->valid) {
            // The requested posture is invalid but the other works
            publishError(errCode,
                         "Requested posture is invalid; the other posture is available");
        } else {
            // Both postures invalid
            publishError(errCode,
                         "Both postures exceed joint limits for this target");
        }
        gStateMachine->fault(Control::FaultCode::JointLimitExceeded);
        return;
    }

    // ---- Step 4: Build the trajectory plan ----
    const Control::GripperPose gripperPose =
        (cmd.gripper == "closed") ? Control::GripperPose::Closed
                                  : Control::GripperPose::Open;

    // Transition state machine: IDLE → MOVING
    if (!gStateMachine->requestMove(gripperPose)) {
        publishError("BAD_COMMAND", "State machine rejected the move");
        return;
    }

    // Build start and end joint vectors
    Motion::JointVector startAngles = gLastKnownAngles;

    Motion::JointVector endAngles;
    endAngles.theta0 = chosen->theta0;
    endAngles.theta1 = chosen->theta1;
    endAngles.theta2 = chosen->theta2;
    endAngles.theta3 = chosen->theta3;
    endAngles.gripper = (gripperPose == Control::GripperPose::Closed)
                            ? JointLimits::kGripperClosed_rad
                            : JointLimits::kGripperOpen_rad;

    const Motion::TrajectoryPlan plan =
        Motion::TrajectoryPlanner::plan(startAngles, endAngles,
                                        gStateMachine->payload());

    LOG_INFO("move: target(%.1f,%.1f,%.1f) phi=%.1f posture=%s duration=%.0f ms",
             cmd.x, cmd.y, cmd.z, cmd.phi, cmd.posture.c_str(),
             plan.duration * 1000.0f);

    // ---- Step 5: Enqueue to MotionTask ----
    MotionCommand motionCmd;
    motionCmd.type = MotionCommand::Type::Execute;
    motionCmd.plan = plan;

    if (!MotionTask::enqueue(motionCmd)) {
        publishError("INTERNAL", "Motion queue full — try again");
        // Roll back the state machine
        gStateMachine->notifyMoveComplete();
        return;
    }

    // Publish the MOVING state
    const char* profileStr =
        gStateMachine->payload() == Motion::PayloadProfile::Loaded ? "loaded" : "empty";
    publishState("MOVING", gStateMachine->homed(), profileStr, cmd.gripper.c_str());
}

// -------------------------------------------------------------------------
// Payload profile
// -------------------------------------------------------------------------

void NetworkTask::handlePayloadProfile(const String& message) {
    String profile;
    if (!Comms::parseSetPayloadProfile(message, profile)) {
        publishError("BAD_COMMAND", "Invalid payload profile");
        return;
    }

    if (gStateMachine == nullptr) {
        publishError("INTERNAL", "State machine not attached");
        return;
    }

    const Motion::PayloadProfile newProfile =
        (profile == "loaded") ? Motion::PayloadProfile::Loaded
                              : Motion::PayloadProfile::Empty;

    if (!gStateMachine->setPayload(newProfile)) {
        publishError("BAD_COMMAND",
                     "Payload profile can only be changed in IDLE state");
        return;
    }

    const char* gripperStr =
        gStateMachine->gripper() == Control::GripperPose::Closed ? "closed" : "open";
    publishState("IDLE", gStateMachine->homed(), profile.c_str(), gripperStr);

    // Update the /api/limits endpoint with the new profile
    gHttpServer.setLimitsPayload(
        profile.c_str(),
        JointLimits::kL1_cm - JointLimits::kL2_cm,
        JointLimits::kL1_cm + JointLimits::kL2_cm);

    LOG_INFO("payload profile set to %s", profile.c_str());
}

// -------------------------------------------------------------------------
// WebSocket message dispatcher
// -------------------------------------------------------------------------

void NetworkTask::onSocketMessage(const String& message) {
    if (gLinkActivityCb != nullptr) {
        gLinkActivityCb(millis());
    }

    StateGuard stateGuard(gStateMutex);
    if (!stateGuard.locked()) {
        publishError("INTERNAL", "State machine lock unavailable");
        return;
    }

    // ---- Heartbeat (lowest overhead, checked first) ----
    if (message.indexOf("ping") >= 0) {
        gSocket.sendPong();
        return;
    }

    // Preview only calculates IK; it never submits a motion command.
    if (message.indexOf("ik_preview") >= 0) {
        handleIkPreview(message);
        return;
    }

    // ---- Move command: full IK → trajectory → queue pipeline ----
    if (message.indexOf("move_command") >= 0) {
        handleMoveCommand(message);
        return;
    }

    // ---- Home request ----
    if (message.indexOf("home_request") >= 0) {
        if (gControlRequestCb != nullptr) {
            gControlRequestCb("home_request");
        } else {
            publishError("INTERNAL", "Control request handler not attached");
        }
        return;
    }

    // ---- Clear e-stop ----
    if (message.indexOf("clear_estop") >= 0) {
        if (gControlRequestCb != nullptr) {
            gControlRequestCb("clear_estop");
        } else {
            publishError("INTERNAL", "Control request handler not attached");
        }
        return;
    }

    // ---- Stop ----
    if (message.indexOf("stop_request") >= 0) {
        if (gControlRequestCb != nullptr) {
            gControlRequestCb("stop_request");
        } else {
            publishError("INTERNAL", "Control request handler not attached");
        }
        return;
    }

    // ---- Payload profile ----
    if (message.indexOf("set_payload_profile") >= 0) {
        handlePayloadProfile(message);
        return;
    }

    // Unknown message type
    LOG_WARN("NetworkTask: unknown message: %.60s", message.c_str());
}

}  // namespace Tasks
