#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <freertos/semphr.h>

#include "JointLimits.h"
#include "Logger.h"
#include "MotionConfig.h"
#include "MotionTask.h"
#include "NetworkTask.h"
#include "Pins.h"
#include "SafetyMonitor.h"
#include "ServoDriver.h"
#include "SimMode.h"
#include "StateMachine.h"
#include "StepperDriver.h"

#ifndef DRIVER_BENCH
#define DRIVER_BENCH 0
#endif

namespace {

Control::SafetyMonitor gSafety;
SemaphoreHandle_t gStateMutex = nullptr;
bool gStepperEnabled = false;
bool gMotionReady = true;
std::atomic<bool> gMoveCompletePending{false};
std::atomic<bool> gMoveStoppedPending{false};
std::atomic<bool> gClearEstopPending{false};
std::atomic<bool> gNetworkConnectedPending{false};
std::atomic<bool> gLinkActivityPending{false};
std::atomic<bool> gStatePublishPending{false};
std::atomic<uint32_t> gLastLinkActivityMs{0};

#if !DRIVER_BENCH
Control::StateMachine gState;
Control::SystemState gLoggedState = Control::SystemState::Boot;
bool gHavePublishedState = false;
Control::SystemState gPublishedState = Control::SystemState::Boot;
Motion::PayloadProfile gPublishedPayload = Motion::PayloadProfile::Empty;
Control::GripperPose gPublishedGripper = Control::GripperPose::Open;
#endif

void lockState() {
    if (gStateMutex != nullptr) {
        xSemaphoreTake(gStateMutex, portMAX_DELAY);
    }
}

void unlockState() {
    if (gStateMutex != nullptr) {
        xSemaphoreGive(gStateMutex);
    }
}

#if DRIVER_BENCH
enum class BenchPhase : uint8_t {
    Idle,
    CommandServos,
    HoldServos,
    CommandStepperFwd,
    WaitStepperFwd,
    CommandStepperBack,
    WaitStepperBack,
    Disable,
    Done,
};

BenchPhase sPhase = BenchPhase::Idle;
uint32_t sPhaseStartMs = 0;

constexpr float kBenchServoDeg = 90.0f;
constexpr float kBenchGripperDeg = 0.0f;
constexpr int32_t kBenchSteps = 800;  // 90° at 16× (3200 steps/rev)
constexpr uint32_t kServoHoldMs = 800;

void enter(BenchPhase next) {
    sPhase = next;
    sPhaseStartMs = millis();
}
#endif

// ISR-safe: stepper driver only. No Serial and no I2C.
void hardCut() {
    Motion::StepperDriver::emergencyStop();
}

void setStepperEnabled(bool enable) {
    if (enable == gStepperEnabled) {
        return;
    }
    Motion::StepperDriver::setEnabled(enable);
    gStepperEnabled = enable;
}

void forceSafeOutputs() {
    Motion::StepperDriver::emergencyStop();
    Motion::ServoDriver::freezeAll();
    gStepperEnabled = false;
}

#if !DRIVER_BENCH
void onMoveComplete(bool stopped) {
    gMoveStoppedPending.store(stopped, std::memory_order_relaxed);
    gMoveCompletePending.store(true, std::memory_order_release);
}

void onNetworkStatus(bool connected) {
    if (connected) {
        gNetworkConnectedPending.store(true, std::memory_order_release);
        gStatePublishPending.store(true, std::memory_order_release);
    }
}

void onNetworkActivity(uint32_t nowMs) {
    gLastLinkActivityMs.store(nowMs, std::memory_order_relaxed);
    gLinkActivityPending.store(true, std::memory_order_release);
}

bool isEstopLatched() {
    return gSafety.isLatched();
}

void handleControlRequest(const char* requestType) {
    if (std::strcmp(requestType, "clear_estop") == 0) {
        if (gState.state() != Control::SystemState::EstopTriggered) {
            Tasks::NetworkTask::publishError(
                "BAD_COMMAND", "Clear e-stop is only valid while e-stop is latched");
        } else {
            gClearEstopPending.store(true, std::memory_order_release);
        }
        return;
    }

    if (std::strcmp(requestType, "stop_request") == 0) {
        if (gState.state() != Control::SystemState::Moving) {
            return;
        }
        Tasks::MotionCommand command;
        command.type = Tasks::MotionCommand::Type::Stop;
        if (!Tasks::MotionTask::enqueue(command)) {
            Tasks::NetworkTask::publishError("INTERNAL", "Unable to queue stop request");
        }
        return;
    }

    Tasks::NetworkTask::publishError("BAD_COMMAND", "Unknown control request");
}

void logState() {
    lockState();
    const Control::SystemState state = gState.state();
    const Control::FaultCode fault = gState.fault();
    const Motion::PayloadProfile payload = gState.payload();
    unlockState();

    if (state == gLoggedState) {
        return;
    }
    LOG_INFO("state %s -> %s fault=%s payload=%s",
             Control::stateName(gLoggedState), Control::stateName(state),
             Control::faultName(fault),
             payload == Motion::PayloadProfile::Loaded ? "loaded" : "empty");
    gLoggedState = state;
}

void updateLed(uint32_t now) {
    bool on = false;
    lockState();
    const Control::SystemState state = gState.state();
    unlockState();
    switch (state) {
        case Control::SystemState::Idle:
        case Control::SystemState::Moving:
            on = true;
            break;
        case Control::SystemState::DisconnectLockout:
            on = ((now / 600) % 2) == 0;
            break;
        default:
            on = false;
            break;
    }
    digitalWrite(Pins::kStatusLed, on ? HIGH : LOW);
}

void publishCurrentState(bool force) {
    lockState();
    const Control::SystemState state = gState.state();
    const Motion::PayloadProfile payload = gState.payload();
    const Control::GripperPose gripper = gState.gripper();
    unlockState();

    if (!force && gHavePublishedState && state == gPublishedState &&
        payload == gPublishedPayload &&
        gripper == gPublishedGripper) {
        return;
    }

    Tasks::NetworkTask::publishState(
        Control::stateName(state),
        payload == Motion::PayloadProfile::Loaded ? "loaded" : "empty",
        gripper == Control::GripperPose::Closed ? "closed" : "open");
    gPublishedState = state;
    gPublishedPayload = payload;
    gPublishedGripper = gripper;
    gHavePublishedState = true;
}

void beginControl() {
    gState.reset();
    gState.begin();
    if (gSafety.isLatched()) {
        gState.notifyEstop();
        LOG_ERROR("e-stop line open at boot — motion latched off");
    }
    if (!gMotionReady) {
        LOG_ERROR("MotionTask unavailable; move commands cannot be executed");
    }
}
#endif

#if DRIVER_BENCH
void commandBenchServos() {
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelShoulder, kBenchServoDeg,
                                     Motion::ServoDriver::kShoulderPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelElbow, kBenchServoDeg,
                                     Motion::ServoDriver::kElbowPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelWrist, kBenchServoDeg,
                                     Motion::ServoDriver::kWristPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelGripper, kBenchGripperDeg,
                                     Motion::ServoDriver::kGripperPulse);
}
#endif

}  // namespace

void setup() {
    Serial.println("BUILD TEST 1: setup reached");          // first line in setup()
    Logger::begin(115200);
    pinMode(Pins::kStatusLed, OUTPUT);
    digitalWrite(Pins::kStatusLed, LOW);

    LOG_INFO("M5 control boot (ESP32-S3) sim=%d bench=%d", SimMode::isEnabled() ? 1 : 0,
             DRIVER_BENCH);
    LOG_INFO("pins STEP=%u DIR=%u EN=%u ESTOP=%u  estop HIGH=triggered",
             Pins::kNemaStep, Pins::kNemaDir, Pins::kNemaEn, Pins::kEstopSw);

    Motion::ServoDriver::begin();
    Motion::StepperDriver::begin();
    gSafety.setHardCut(&hardCut);
    gSafety.attachHardware();

    gStateMutex = xSemaphoreCreateMutex();
    if (gStateMutex == nullptr) {
        LOG_ERROR("failed to allocate state-machine mutex");
    }
    Tasks::NetworkTask::attachStateMutex(gStateMutex);

#if !DRIVER_BENCH
    Tasks::NetworkTask::attachStateMachine(&gState);
    Tasks::NetworkTask::setLinkStatusCallback(&onNetworkStatus);
    Tasks::NetworkTask::setLinkActivityCallback(&onNetworkActivity);
    Tasks::NetworkTask::setControlRequestCallback(&handleControlRequest);
    Tasks::MotionTask::setMoveCompleteCallback(&onMoveComplete);
    Tasks::MotionTask::setProgressCallback(
        &Tasks::NetworkTask::publishTrajectoryProgress);
    Tasks::MotionTask::setEmergencyStopCheckCallback(&isEstopLatched);
    gMotionReady = gStateMutex != nullptr && Tasks::MotionTask::begin();
    if (!gMotionReady) {
        LOG_ERROR("MotionTask failed to start; network move control disabled");
    }
#endif

#if !DRIVER_BENCH
    beginControl();
#endif

    if (!Tasks::NetworkTask::begin()) {
        LOG_ERROR("NetworkTask failed to start");
    }

#if DRIVER_BENCH
    enter(BenchPhase::CommandServos);
    LOG_INFO("DRIVER_BENCH: servos -> %.0f deg (gripper %.0f), stepper +/-%ld steps",
             kBenchServoDeg, kBenchGripperDeg, static_cast<long>(kBenchSteps));
#endif

    gSafety.kick(millis());
}

void loop() {
    const uint32_t now = millis();

#if DRIVER_BENCH
    if (gSafety.isLatched()) {
        forceSafeOutputs();
        if (sPhase != BenchPhase::Done) {
            LOG_ERROR("e-stop during driver bench");
            enter(BenchPhase::Done);
        }
        return;
    }
    switch (sPhase) {
        case BenchPhase::CommandServos:
            commandBenchServos();
            enter(BenchPhase::HoldServos);
            break;
        case BenchPhase::HoldServos:
            if (now - sPhaseStartMs >= kServoHoldMs) {
                enter(BenchPhase::CommandStepperFwd);
            }
            break;
        case BenchPhase::CommandStepperFwd:
            setStepperEnabled(true);
            Motion::StepperDriver::setTargetStep(kBenchSteps);
            enter(BenchPhase::WaitStepperFwd);
            LOG_INFO("[bench] stepper target +%ld", static_cast<long>(kBenchSteps));
            break;
        case BenchPhase::WaitStepperFwd:
            if (Motion::StepperDriver::atTarget()) {
                enter(BenchPhase::CommandStepperBack);
            }
            break;
        case BenchPhase::CommandStepperBack:
            Motion::StepperDriver::setTargetStep(0);
            enter(BenchPhase::WaitStepperBack);
            LOG_INFO("[bench] stepper target 0");
            break;
        case BenchPhase::WaitStepperBack:
            if (Motion::StepperDriver::atTarget()) {
                enter(BenchPhase::Disable);
            }
            break;
        case BenchPhase::Disable:
            setStepperEnabled(false);
            Motion::ServoDriver::freezeAll();
            digitalWrite(Pins::kStatusLed, HIGH);
            LOG_INFO("[bench] complete. current_step=%ld",
                     static_cast<long>(Motion::StepperDriver::currentStep()));
            enter(BenchPhase::Done);
            break;
        case BenchPhase::Idle:
        case BenchPhase::Done:
            break;
    }
    gSafety.kick(now);
#else
    if (gMoveCompletePending.exchange(false, std::memory_order_acquire)) {
        const bool stopped = gMoveStoppedPending.load(std::memory_order_relaxed);
        lockState();
        const bool completed = gState.notifyMoveComplete();
        const Control::SystemState state = gState.state();
        unlockState();
        if (!completed && state != Control::SystemState::EstopTriggered) {
            LOG_WARN("motion completion received outside MOVING state");
        }
        if (stopped) {
            LOG_INFO("motion stopped");
        }
    }

    if (gNetworkConnectedPending.exchange(false, std::memory_order_acquire)) {
        gSafety.startLinkSupervision(now);
        lockState();
        gState.notifyReconnect();
        unlockState();
    }
    if (gLinkActivityPending.exchange(false, std::memory_order_acquire)) {
        gSafety.noteLinkActivity(
            gLastLinkActivityMs.load(std::memory_order_relaxed));
    }
    if (gClearEstopPending.exchange(false, std::memory_order_acquire)) {
        const bool lineSafe =
            SimMode::isEnabled() || digitalRead(Pins::kEstopSw) == LOW;
        if (!lineSafe) {
            Tasks::NetworkTask::publishError(
                "BAD_COMMAND", "Cannot clear e-stop while the input is triggered");
        } else {
            gSafety.releaseLatch();
            const bool lineStillSafe =
                SimMode::isEnabled() || digitalRead(Pins::kEstopSw) == LOW;
            if (gSafety.isLatched() || !lineStillSafe) {
                if (!gSafety.isLatched()) {
                    gSafety.onEstopEdge();
                }
                lockState();
                gState.notifyEstop();
                unlockState();
                Tasks::NetworkTask::publishError(
                    "BAD_COMMAND", "E-stop became active while clearing");
            } else {
                lockState();
                const bool cleared = gState.requestClearEstop();
                unlockState();
                if (cleared) {
                    LOG_WARN("position may be uncertain after e-stop; verify visually");
                } else {
                    Tasks::NetworkTask::publishError(
                        "BAD_COMMAND", "Clear e-stop is only valid while e-stop is latched");
                }
            }
        }
    }

    if (gSafety.pollWatchdog(now)) {
        forceSafeOutputs();
        lockState();
        gState.fault(Control::FaultCode::Watchdog);
        unlockState();
        LOG_ERROR("control-loop watchdog");
    }

    if (gSafety.pollLink(now)) {
        lockState();
        const Control::SystemState before = gState.state();
        gState.notifyDisconnect();
        const Control::SystemState after = gState.state();
        unlockState();
        if (before == Control::SystemState::Moving) {
            LOG_WARN("link lost during move — finishing trajectory, then lockout");
        } else if (after == Control::SystemState::DisconnectLockout) {
            LOG_WARN("link lost — disconnect lockout");
        }
    }

    // Keep the state machine synchronized if the hardware interrupt trips.
    bool newlyEstopped = false;
    if (gSafety.isLatched()) {
        lockState();
        if (gState.state() != Control::SystemState::EstopTriggered) {
            gState.notifyEstop();
            newlyEstopped = true;
        }
        unlockState();
    }
    if (newlyEstopped) {
        forceSafeOutputs();
        LOG_ERROR("e-stop latched (HIGH = open circuit). clear_estop required");
    }

    publishCurrentState(
        gStatePublishPending.exchange(false, std::memory_order_acquire));

    logState();
    updateLed(now);
    gSafety.kick(now);
#endif
}
