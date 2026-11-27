#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "TrajectoryPlanner.h"

// Milestone 7 — FreeRTOS motion control task (AGENT_BUILD_SPEC §10, §6.3).
//
// Receives TrajectoryPlan descriptors from NetworkTask via a FreeRTOS queue.
// Runs a fixed-rate control loop that drives the stepper and servos along the
// planned S-curve trajectory, pushing trajectory_progress updates back to the
// network layer for the web app.
//
// Control loop timing:
//   - Stepper position update:  1 kHz (MotionConfig::kStepperLoopHz via ISR)
//   - Servo PWM update:        50 Hz (MotionConfig::kServoUpdateHz)
//   - Progress telemetry:      ~20 Hz (configurable, avoids flooding WebSocket)

namespace Tasks {

/// Command envelope pushed from NetworkTask → MotionTask via FreeRTOS queue.
struct MotionCommand {
    enum class Type : uint8_t {
        Execute,   // Run a trajectory plan to completion
        Stop,      // Abort the active trajectory immediately
    };

    Type type = Type::Execute;
    Motion::TrajectoryPlan plan;   // Only meaningful when type == Execute
};

class MotionTask {
 public:
    /// Create the FreeRTOS queue and motion task. Call once from setup().
    static bool begin();

    /// Enqueue a motion command from the network task (ISR-unsafe, runs from
    /// the network FreeRTOS task context).  Returns false if the queue is full
    /// (caller should reject the command with an error).
    static bool enqueue(const MotionCommand& cmd);

    /// Check whether the motion task is currently executing a trajectory.
    static bool isMoving();

    /// Read the current commanded joint angles (thread-safe snapshot).
    static Motion::JointVector currentAngles();

    /// Read move progress [0, 1] and timing info. Returns 0 if idle.
    static float currentProgress();

    /// Duration of the active trajectory in milliseconds. 0 if idle.
    static uint32_t currentDurationMs();

    /// Elapsed time of the active trajectory in milliseconds. 0 if idle.
    static uint32_t currentElapsedMs();

    /// Set the callback invoked after a trajectory completes (or is stopped).
    /// Called from the MotionTask FreeRTOS context — keep it short.
    using MoveCompleteCb = void (*)(bool stopped);
    static void setMoveCompleteCallback(MoveCompleteCb cb);

    /// Set the callback invoked periodically during trajectory execution to
    /// push trajectory_progress telemetry.
    using ProgressCb = void (*)(float progress,
                                float theta0, float theta1,
                                float theta2, float theta3,
                                uint32_t elapsedMs, uint32_t durationMs);
    static void setProgressCallback(ProgressCb cb);

    /// Abort the active trajectory when the system's hard safety latch is set.
    using EmergencyStopCheckCb = bool (*)();
    static void setEmergencyStopCheckCallback(EmergencyStopCheckCb cb);

 private:
    static void taskEntry(void* param);
    static void executeTrajectory(const Motion::TrajectoryPlan& plan);
    static void applyJointAngles(const Motion::JointVector& angles);
};

}  // namespace Tasks
