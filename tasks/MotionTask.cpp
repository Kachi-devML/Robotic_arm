#include "MotionTask.h"

#include "JointLimits.h"
#include "Logger.h"
#include "MotionConfig.h"
#include "ServoDriver.h"
#include "SimMode.h"
#include "StepperDriver.h"

namespace Tasks {

// -------------------------------------------------------------------------
// Module-level state (accessed only by the MotionTask FreeRTOS task and the
// atomic/volatile fields that NetworkTask reads for status queries).
// -------------------------------------------------------------------------
namespace {

constexpr UBaseType_t kQueueDepth = 4;
constexpr uint32_t kTaskStackWords = 4096;
constexpr UBaseType_t kTaskPriority = 2;  // Higher than NetworkTask (1)

// Telemetry push rate: ~20 Hz (every 50 ms). Avoids flooding the WebSocket
// while still giving the UI a smooth progress bar / live angle display.
constexpr uint32_t kProgressIntervalMs = 50;

// Servo update period (ms): the hardware PWM runs at 50 Hz and a faster update
// rate does not produce a different pulse width until the next 20 ms frame.
constexpr uint32_t kServoUpdateMs = 1000 / MotionConfig::kServoUpdateHz;

QueueHandle_t gQueue = nullptr;
TaskHandle_t gTaskHandle = nullptr;

volatile bool gMoving = false;
volatile float gProgress = 0.0f;
volatile uint32_t gDurationMs = 0;
volatile uint32_t gElapsedMs = 0;

// Protected by a mutex because JointVector is multi-word.
SemaphoreHandle_t gAnglesMutex = nullptr;
Motion::JointVector gCurrentAngles = {};

MotionTask::MoveCompleteCb gMoveCompleteCb = nullptr;
MotionTask::ProgressCb gProgressCb = nullptr;
MotionTask::EmergencyStopCheckCb gEmergencyStopCheckCb = nullptr;

}  // namespace

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------

bool MotionTask::begin() {
    if (gQueue != nullptr) {
        return gTaskHandle != nullptr;
    }

    gQueue = xQueueCreate(kQueueDepth, sizeof(MotionCommand));
    gAnglesMutex = xSemaphoreCreateMutex();
    if (gQueue == nullptr || gAnglesMutex == nullptr) {
        LOG_ERROR("MotionTask: failed to allocate queue or angle mutex");
        if (gQueue != nullptr) {
            vQueueDelete(gQueue);
            gQueue = nullptr;
        }
        if (gAnglesMutex != nullptr) {
            vSemaphoreDelete(gAnglesMutex);
            gAnglesMutex = nullptr;
        }
        return false;
    }

    // Initialize current angles to the power-on neutral pose.
    gCurrentAngles.theta0 = JointLimits::kTheta0Home_rad;
    gCurrentAngles.theta1 = JointLimits::kTheta1Home_rad;
    gCurrentAngles.theta2 = JointLimits::kTheta2Home_rad;
    gCurrentAngles.theta3 = JointLimits::kTheta3Home_rad;
    gCurrentAngles.gripper = JointLimits::kGripperHome_rad;

    const BaseType_t created = xTaskCreatePinnedToCore(
        MotionTask::taskEntry,
        "MotionTask",
        kTaskStackWords,
        nullptr,
        kTaskPriority,
        &gTaskHandle,
        1  // Motion stays isolated from the network task on core 0.
    );

    if (created != pdPASS) {
        LOG_ERROR("MotionTask: failed to start FreeRTOS task");
        vQueueDelete(gQueue);
        vSemaphoreDelete(gAnglesMutex);
        gQueue = nullptr;
        gAnglesMutex = nullptr;
        return false;
    }

    LOG_INFO("MotionTask started on core 1, queue depth=%u", kQueueDepth);
    return true;
}

bool MotionTask::enqueue(const MotionCommand& cmd) {
    if (gQueue == nullptr) {
        return false;
    }
    // Non-blocking: if the queue is full the caller gets a rejection
    return xQueueSend(gQueue, &cmd, 0) == pdTRUE;
}

bool MotionTask::isMoving() {
    return gMoving;
}

Motion::JointVector MotionTask::currentAngles() {
    Motion::JointVector copy;
    if (gAnglesMutex != nullptr &&
        xSemaphoreTake(gAnglesMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        copy = gCurrentAngles;
        xSemaphoreGive(gAnglesMutex);
    } else {
        // Fallback: return the power-on neutral pose if the mutex is unavailable.
        copy.theta0 = JointLimits::kTheta0Home_rad;
        copy.theta1 = JointLimits::kTheta1Home_rad;
        copy.theta2 = JointLimits::kTheta2Home_rad;
        copy.theta3 = JointLimits::kTheta3Home_rad;
        copy.gripper = JointLimits::kGripperHome_rad;
    }
    return copy;
}

float MotionTask::currentProgress() {
    return gProgress;
}

uint32_t MotionTask::currentDurationMs() {
    return gDurationMs;
}

uint32_t MotionTask::currentElapsedMs() {
    return gElapsedMs;
}

void MotionTask::setMoveCompleteCallback(MoveCompleteCb cb) {
    gMoveCompleteCb = cb;
}

void MotionTask::setProgressCallback(ProgressCb cb) {
    gProgressCb = cb;
}

void MotionTask::setEmergencyStopCheckCallback(EmergencyStopCheckCb cb) {
    gEmergencyStopCheckCb = cb;
}

// -------------------------------------------------------------------------
// Apply joint angles to hardware drivers
// -------------------------------------------------------------------------

void MotionTask::applyJointAngles(const Motion::JointVector& angles) {
    // Stepper: convert θ0 radians to step count in output-shaft degrees.
    const int32_t targetStep = Motion::StepperDriver::radiansToSteps(angles.theta0);
    Motion::StepperDriver::setTargetStep(targetStep);

    // Servos: direct LEDC PWM updates at 50 Hz.
    const float shoulderDeg = JointLimits::radToDeg(angles.theta1);
    const float elbowDeg    = JointLimits::radToDeg(angles.theta2);
    const float wristDeg    = JointLimits::radToDeg(angles.theta3);
    const float gripperDeg  = JointLimits::radToDeg(angles.gripper);

    Motion::ServoDriver::writeAngleDeg(Motion::ServoId::SHOULDER, shoulderDeg);
    Motion::ServoDriver::writeAngleDeg(Motion::ServoId::ELBOW, elbowDeg);
    Motion::ServoDriver::writeAngleDeg(Motion::ServoId::WRIST, wristDeg);
    Motion::ServoDriver::writeAngleDeg(Motion::ServoId::GRIPPER, gripperDeg);
}

// -------------------------------------------------------------------------
// Trajectory execution loop
// -------------------------------------------------------------------------

void MotionTask::executeTrajectory(const Motion::TrajectoryPlan& plan) {
    if (gEmergencyStopCheckCb != nullptr && gEmergencyStopCheckCb()) {
        return;
    }

    if (plan.duration <= 0.0f) {
        // Zero-length move: already at the target.  Apply final angles and return.
        const Motion::JointVector finalAngles = plan.anglesAt(0.0f);
        applyJointAngles(finalAngles);

        if (gAnglesMutex != nullptr &&
            xSemaphoreTake(gAnglesMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gCurrentAngles = finalAngles;
            xSemaphoreGive(gAnglesMutex);
        }

        gProgress = 1.0f;
        gElapsedMs = 0;
        gDurationMs = 0;
        gMoving = false;
        if (gProgressCb != nullptr) {
            gProgressCb(
                1.0f,
                JointLimits::radToDeg(finalAngles.theta0),
                JointLimits::radToDeg(finalAngles.theta1),
                JointLimits::radToDeg(finalAngles.theta2),
                JointLimits::radToDeg(finalAngles.theta3),
                0, 0);
        }
        LOG_INFO("MotionTask: zero-duration move, already at target");
        if (gMoveCompleteCb != nullptr) {
            gMoveCompleteCb(false);
        }
        return;
    }

    const uint32_t durationMs = static_cast<uint32_t>(plan.duration * 1000.0f + 0.5f);
    gDurationMs = durationMs;
    gElapsedMs = 0;
    gProgress = 0.0f;
    gMoving = true;

    Motion::StepperDriver::enable();
    uint32_t settleStartMs = millis();
    while (!Motion::StepperDriver::isReady() &&
           (millis() - settleStartMs) < MotionConfig::STEPPER_ENABLE_SETTLE_MS + 10U) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint32_t startMs = millis();
    uint32_t lastServoMs = 0;
    uint32_t lastProgressMs = 0;
    TickType_t lastWake = xTaskGetTickCount();
    bool stopped = false;

    LOG_INFO("MotionTask: executing trajectory, duration=%lu ms, limiting_joint=%d",
             static_cast<unsigned long>(durationMs), plan.limiting_joint);

    while (true) {
        const uint32_t now = millis();
        const uint32_t elapsed = now - startMs;
        const float t = static_cast<float>(elapsed) / 1000.0f;

        if (gEmergencyStopCheckCb != nullptr && gEmergencyStopCheckCb()) {
            Motion::StepperDriver::emergencyStop();
            Motion::ServoDriver::freezeAll();
            stopped = true;
            break;
        }

        // Check for stop command (non-blocking peek at the queue)
        MotionCommand peek;
        if (xQueuePeek(gQueue, &peek, 0) == pdTRUE) {
            if (peek.type == MotionCommand::Type::Stop) {
                // Consume the stop command
                xQueueReceive(gQueue, &peek, 0);
                LOG_WARN("MotionTask: stop received at t=%.3f s", t);

                // Freeze at current position
                Motion::StepperDriver::setTargetStep(
                    Motion::StepperDriver::currentStep());
                Motion::ServoDriver::freezeAll();

                stopped = true;
                break;
            }
        }

        // Trajectory complete?
        if (t >= plan.duration) {
            break;
        }

        // Compute current joint angles from the trajectory
        const Motion::JointVector angles = plan.anglesAt(t);

        // Update stepper target every iteration (stepper ISR runs at 1 kHz
        // independently; we just update the commanded position)
        Motion::StepperDriver::setTargetStep(
            Motion::StepperDriver::radiansToSteps(angles.theta0));

        // Update servos at the 50 Hz LEDC frame rate.
        if (elapsed - lastServoMs >= kServoUpdateMs) {
            lastServoMs = elapsed;
            Motion::ServoDriver::writeAngleDeg(Motion::ServoId::SHOULDER,
                                              JointLimits::radToDeg(angles.theta1));
            Motion::ServoDriver::writeAngleDeg(Motion::ServoId::ELBOW,
                                              JointLimits::radToDeg(angles.theta2));
            Motion::ServoDriver::writeAngleDeg(Motion::ServoId::WRIST,
                                              JointLimits::radToDeg(angles.theta3));
            Motion::ServoDriver::writeAngleDeg(Motion::ServoId::GRIPPER,
                                              JointLimits::radToDeg(angles.gripper));
        }

        // Update shared state for status queries
        const float progress = plan.progress(t);
        gProgress = progress;
        gElapsedMs = elapsed;

        if (gAnglesMutex != nullptr &&
            xSemaphoreTake(gAnglesMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            gCurrentAngles = angles;
            xSemaphoreGive(gAnglesMutex);
        }

        // Push telemetry at the configured rate (~20 Hz)
        if (gProgressCb != nullptr && (elapsed - lastProgressMs >= kProgressIntervalMs)) {
            lastProgressMs = elapsed;
            gProgressCb(
                progress,
                JointLimits::radToDeg(angles.theta0),
                JointLimits::radToDeg(angles.theta1),
                JointLimits::radToDeg(angles.theta2),
                JointLimits::radToDeg(angles.theta3),
                elapsed, durationMs);
        }

        // Keep the command update loop at an absolute 1 ms cadence.
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }

    // ---- Move finished (or stopped) ----

    if (!stopped) {
        // Apply the exact final angles to ensure we land precisely
        const Motion::JointVector finalAngles = plan.anglesAt(plan.duration);
        applyJointAngles(finalAngles);

        if (gAnglesMutex != nullptr &&
            xSemaphoreTake(gAnglesMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gCurrentAngles = finalAngles;
            xSemaphoreGive(gAnglesMutex);
        }

        gProgress = 1.0f;
        gElapsedMs = durationMs;

        // Send final progress update
        if (gProgressCb != nullptr) {
            gProgressCb(
                1.0f,
                JointLimits::radToDeg(finalAngles.theta0),
                JointLimits::radToDeg(finalAngles.theta1),
                JointLimits::radToDeg(finalAngles.theta2),
                JointLimits::radToDeg(finalAngles.theta3),
                durationMs, durationMs);
        }
    }

    // Wait for stepper to reach the final commanded position (timeout: 2s)
    const uint32_t stepWaitStart = millis();
    while (!Motion::StepperDriver::atTarget() &&
           (millis() - stepWaitStart) < 2000) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    gMoving = false;
    if (MotionConfig::STEPPER_IDLE_RELEASE_ENABLED) {
        Motion::StepperDriver::release();
    }
    LOG_INFO("MotionTask: trajectory %s", stopped ? "stopped" : "complete");

    // Notify the state machine
    if (gMoveCompleteCb != nullptr) {
        gMoveCompleteCb(stopped);
    }
}

// -------------------------------------------------------------------------
// FreeRTOS task entry point
// -------------------------------------------------------------------------

void MotionTask::taskEntry(void* /*param*/) {
    LOG_INFO("MotionTask: task running on core %d", xPortGetCoreID());

    MotionCommand cmd;

    for (;;) {
        // Block waiting for the next command (infinite timeout)
        if (xQueueReceive(gQueue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (cmd.type) {
            case MotionCommand::Type::Execute:
                executeTrajectory(cmd.plan);
                break;

            case MotionCommand::Type::Stop:
                // If not moving, a Stop is a no-op
                if (gMoving) {
                    // This path shouldn't normally be reached because stops
                    // are consumed inside executeTrajectory via queue peek.
                    // But handle it defensively.
                    Motion::StepperDriver::setTargetStep(
                        Motion::StepperDriver::currentStep());
                    Motion::ServoDriver::freezeAll();
                    gMoving = false;
                    LOG_WARN("MotionTask: stop command (idle path)");
                    if (gMoveCompleteCb != nullptr) {
                        gMoveCompleteCb(true);
                    }
                }
                break;

        }
    }
}

}  // namespace Tasks
