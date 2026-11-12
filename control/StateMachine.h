#pragma once

#include <stdint.h>

#include "TrajectoryPlanner.h"

// Explicit arm controller state machine (AGENT_BUILD_SPEC §5).
//
// Pure logic: no GPIO, I2C, or FreeRTOS. Callers apply motor outputs.
//
//   BOOT → SELF_CHECK → HOMING → IDLE ⇄ MOVING
//                                  ↓
//                                ERROR → (valid move or clear) → IDLE
//
//   Any state → ESTOP_TRIGGERED
//     ESTOP_TRIGGERED → (clear_estop AND line safe AND re-home) → HOMING
//
//   MOVING + disconnect → stay MOVING until the trajectory finishes → DISCONNECT_LOCKOUT
//   Any other supervised state + disconnect → DISCONNECT_LOCKOUT immediately
//   DISCONNECT_LOCKOUT → (reconnect AND re-home) → HOMING
//
// Payload profile resets to EMPTY when homing completes (boot and every re-home).

namespace Control {

enum class SystemState : uint8_t {
    Boot = 0,
    SelfCheck,
    Homing,
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
    HomingFailed,
    SelfCheckFailed,
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
    bool homed() const { return homed_; }
    bool linkUp() const { return linkUp_; }
    bool estopLatched() const { return estopLatched_; }
    bool clearEstopRequested() const { return clearEstopRequested_; }
    bool finishAfterDisconnect() const { return finishAfterDisconnect_; }

    /// BOOT, EMPTY payload, gripper open, not homed.
    void reset();

    /// BOOT → SELF_CHECK.
    bool beginSelfCheck();

    /// SELF_CHECK → HOMING on success, ERROR / SELF_CHECK_FAILED otherwise.
    bool reportSelfCheck(bool ok);

    /// HOMING → IDLE. Marks homed, opens the gripper, forces the EMPTY profile.
    bool notifyHomingComplete();

    /// HOMING → ERROR / HOMING_FAILED. Leaves the arm unhomed.
    bool notifyHomingFailed();

    /// IDLE → MOVING. A valid move issued from ERROR clears the fault and starts.
    /// Rejected unless the arm is homed. Gripper pose is stored for telemetry.
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

    /// Highest priority. Latches until clear_estop and a successful re-home.
    void notifyEstop();

    /// Records the operator clear. Does not leave ESTOP_TRIGGERED and does not
    /// start motion — a released switch alone is not a resume.
    bool requestClearEstop();

    /// Explicit re-home.
    /// IDLE / ERROR → HOMING.
    /// ESTOP_TRIGGERED → HOMING only after requestClearEstop and a safe line.
    /// DISCONNECT_LOCKOUT → HOMING only after the link is back.
    /// `lineSafe` is the e-stop input: true when the circuit is closed (LOW).
    bool requestHome(bool lineSafe);

    /// Link dropped. MOVING keeps running with finishAfterDisconnect set.
    /// Every other non-ESTOP state goes straight to DISCONNECT_LOCKOUT.
    void notifyDisconnect();

    /// Link restored. Does not leave DISCONNECT_LOCKOUT.
    void notifyReconnect();

    bool acceptsMove() const;

 private:
    SystemState state_ = SystemState::Boot;
    FaultCode fault_ = FaultCode::None;
    Motion::PayloadProfile payload_ = Motion::PayloadProfile::Empty;
    GripperPose gripper_ = GripperPose::Open;
    bool homed_ = false;
    bool linkUp_ = true;
    bool estopLatched_ = false;
    bool clearEstopRequested_ = false;
    bool finishAfterDisconnect_ = false;

    void enterHoming();
};

}  // namespace Control
