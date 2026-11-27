#pragma once

#include <stdint.h>

#include "TrajectoryPlanner.h"

// Explicit arm controller state machine (AGENT_BUILD_SPEC §5).
//
// Pure logic: no GPIO, I2C, or FreeRTOS. Callers apply motor outputs.
//
//   BOOT → IDLE ⇄ MOVING
//                ↓
//              ERROR → (valid move or clear) → IDLE
//
//   Any state → ESTOP_TRIGGERED
//     ESTOP_TRIGGERED → (explicit clear_estop with a safe line) → IDLE
//
//   MOVING + disconnect → stay MOVING until the trajectory finishes → DISCONNECT_LOCKOUT
//   Any other supervised state + disconnect → DISCONNECT_LOCKOUT immediately
//   DISCONNECT_LOCKOUT → (fresh connection) → IDLE
//
// Payload profile resets to EMPTY on boot, e-stop clear, and disconnect recovery.

namespace Control {

enum class SystemState : uint8_t {
    Boot = 0,
    Idle,
    Moving,
    Error,
    EstopTriggered,
    DisconnectLockout,
};

enum class FaultCode : uint8_t {
    None = 0,
    BadCommand,
    TargetUnreachable,
    IkFailure,
    JointLimitExceeded,
    Watchdog,
};

enum class GripperPose : uint8_t {
    Open = 0,
    Closed,
};

const char* stateName(SystemState state);
const char* faultName(FaultCode code);

class StateMachine {
 public:
    SystemState state() const { return state_; }
    FaultCode fault() const { return fault_; }
    Motion::PayloadProfile payload() const { return payload_; }
    GripperPose gripper() const { return gripper_; }
    bool linkUp() const { return linkUp_; }
    bool estopLatched() const { return estopLatched_; }
    bool finishAfterDisconnect() const { return finishAfterDisconnect_; }

    /// Reset to BOOT, EMPTY payload, and an open gripper.
    void reset();

    /// BOOT → IDLE.
    bool begin();

    /// IDLE → MOVING. A valid move issued from ERROR clears the fault and starts.
    /// Gripper pose is stored for telemetry.
    bool requestMove(GripperPose gripper);

    /// MOVING → IDLE, or → DISCONNECT_LOCKOUT if the link dropped mid-move.
    bool notifyMoveComplete();

    /// Operator stop. Aborts MOVING into IDLE. Ignored while a disconnect is
    /// waiting for the current trajectory to finish (no operator link).
    bool requestStop();

    /// Recoverable failure. Ignored in ESTOP and DISCONNECT_LOCKOUT.
    bool fault(FaultCode code);

    /// ERROR → IDLE without starting motion. The next command can then move.
    bool clearFault();

    /// Accepted in IDLE only. Rejected mid-move and in every other state.
    bool setPayload(Motion::PayloadProfile profile);

    /// Highest priority. Latches until an explicit, safe clear_estop command.
    void notifyEstop();

    /// ESTOP_TRIGGERED → IDLE. Caller must verify the e-stop line is safe and
    /// release the hardware safety latch first.
    bool requestClearEstop();

    /// Link dropped. MOVING keeps running with finishAfterDisconnect set.
    /// Every other non-ESTOP state goes straight to DISCONNECT_LOCKOUT.
    void notifyDisconnect();

    /// Link restored. Leaves DISCONNECT_LOCKOUT and returns to IDLE.
    void notifyReconnect();

    bool acceptsMove() const;

 private:
    SystemState state_ = SystemState::Boot;
    FaultCode fault_ = FaultCode::None;
    Motion::PayloadProfile payload_ = Motion::PayloadProfile::Empty;
    GripperPose gripper_ = GripperPose::Open;
    bool linkUp_ = true;
    bool estopLatched_ = false;
    bool finishAfterDisconnect_ = false;
};

}  // namespace Control
