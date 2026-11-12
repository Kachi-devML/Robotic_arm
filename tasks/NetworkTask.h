#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Kinematics.h"
#include "StateMachine.h"
#include "TrajectoryPlanner.h"

// Milestone 7 — FreeRTOS network task (AGENT_BUILD_SPEC §10).
//
// Receives WebSocket messages, runs the IK solver (cheap, analytical — spec
// says to run it inline in the network context), validates the result against
// the state machine, builds a TrajectoryPlan, and pushes it to MotionTask
// via a FreeRTOS queue.
//
// Outbound: broadcasts state_update, ik_solution, trajectory_progress,
// and error messages over WebSocket.
//
// Runs as a pinned FreeRTOS task on core 0.
// MotionTask runs on core 1.

namespace Tasks {

class NetworkTask {
 public:
    /// Initialize networking and start the network service task on core 0.
    static bool begin();

    /// Service the network stack. Used by the pinned network task.
    static void loop();

    // --- Outbound state push ---

    static void publishState(const char* state,
                             bool homed,
                             const char* payloadProfile,
                             const char* gripper);
    static void publishError(const char* code, const char* message);
    static void publishTrajectoryProgress(float progress,
                                          float theta0,
                                          float theta1,
                                          float theta2,
                                          float theta3,
                                          uint32_t elapsedMs,
                                          uint32_t durationMs);
    static void publishIkSolution(const Kinematics::IKResult& ik);

    static bool connected();

    // --- Integration hooks (called by main.cpp) ---

    /// Provide the StateMachine and current-angles accessor so the network
    /// task can coordinate move commands with the state machine.
    static void attachStateMachine(Control::StateMachine* sm);
    static void attachStateMutex(SemaphoreHandle_t mutex);

    using LinkStatusCb = void (*)(bool connected);
    using LinkActivityCb = void (*)(uint32_t nowMs);
    static void setLinkStatusCallback(LinkStatusCb cb);
    static void setLinkActivityCallback(LinkActivityCb cb);

    /// Provide a callback the network task invokes when the user requests
    /// a home or clear-estop (things the control loop in main.cpp handles).
    using ControlRequestCb = void (*)(const char* requestType);
    static void setControlRequestCallback(ControlRequestCb cb);

    /// Provide a callback to query the current payload profile name.
    using PayloadQueryCb = const char* (*)();
    static void setPayloadQueryCallback(PayloadQueryCb cb);

 private:
    static void taskEntry(void* param);
    static void onSocketMessage(const String& message);
    static void handleIkPreview(const String& message);
    static void handleMoveCommand(const String& message);
    static void handlePayloadProfile(const String& message);
};

}  // namespace Tasks
