#include "StateMachine.h"

namespace Control {

const char* stateName(SystemState state) {
    switch (state) {
        case SystemState::Boot:
            return "BOOT";
        case SystemState::SelfCheck:
            return "SELF_CHECK";
        case SystemState::Homing:
            return "HOMING";
        case SystemState::Idle:
            return "IDLE";
        case SystemState::Moving:
            return "MOVING";
        case SystemState::Error:
            return "ERROR";
        case SystemState::EstopTriggered:
            return "ESTOP_TRIGGERED";
        case SystemState::DisconnectLockout:
            return "DISCONNECT_LOCKOUT";
    }
    return "UNKNOWN";
}

const char* faultName(FaultCode code) {
    switch (code) {
        case FaultCode::None:
            return "NONE";
        case FaultCode::BadCommand:
            return "BAD_COMMAND";
        case FaultCode::TargetUnreachable:
            return "TARGET_UNREACHABLE";
        case FaultCode::IkFailure:
            return "IK_FAILED";
        case FaultCode::JointLimitExceeded:
            return "JOINT_LIMIT_EXCEEDED";
        case FaultCode::HomingFailed:
            return "HOMING_FAILED";
        case FaultCode::SelfCheckFailed:
            return "SELF_CHECK_FAILED";
        case FaultCode::Watchdog:
            return "WATCHDOG";
    }
    return "UNKNOWN";
}

void StateMachine::reset() {
    state_ = SystemState::Boot;
    fault_ = FaultCode::None;
    payload_ = Motion::PayloadProfile::Empty;
    gripper_ = GripperPose::Open;
    homed_ = false;
    linkUp_ = true;
    estopLatched_ = false;
    clearEstopRequested_ = false;
    finishAfterDisconnect_ = false;
}

bool StateMachine::beginSelfCheck() {
    if (state_ != SystemState::Boot) {
        return false;
    }
    state_ = SystemState::SelfCheck;
    fault_ = FaultCode::None;
    return true;
}

bool StateMachine::reportSelfCheck(bool ok) {
    if (state_ != SystemState::SelfCheck) {
        return false;
    }
    if (!ok) {
        state_ = SystemState::Error;
        fault_ = FaultCode::SelfCheckFailed;
        homed_ = false;
        return false;
    }
    enterHoming();
    return true;
}

bool StateMachine::notifyHomingComplete() {
    if (state_ != SystemState::Homing) {
        return false;
    }
    state_ = SystemState::Idle;
    fault_ = FaultCode::None;
    homed_ = true;
    gripper_ = GripperPose::Open;
    payload_ = Motion::PayloadProfile::Empty;
    estopLatched_ = false;
    clearEstopRequested_ = false;
    finishAfterDisconnect_ = false;
    return true;
}

bool StateMachine::notifyHomingFailed() {
    if (state_ != SystemState::Homing) {
        return false;
    }
    state_ = SystemState::Error;
    fault_ = FaultCode::HomingFailed;
    homed_ = false;
    return true;
}

bool StateMachine::requestMove(GripperPose gripper) {
    if (state_ == SystemState::Error) {
        if (!homed_ || estopLatched_ || !linkUp_) {
            return false;
        }
        state_ = SystemState::Idle;
        fault_ = FaultCode::None;
    }
    if (!acceptsMove()) {
        return false;
    }
    gripper_ = gripper;
    state_ = SystemState::Moving;
    finishAfterDisconnect_ = false;
    return true;
}

bool StateMachine::notifyMoveComplete() {
    if (state_ != SystemState::Moving) {
        return false;
    }
    if (finishAfterDisconnect_) {
        // Keep linkUp_ as it stands: a client may have reconnected while the
        // trajectory was finishing. Lockout still requires an explicit re-home.
        state_ = SystemState::DisconnectLockout;
        homed_ = false;
        finishAfterDisconnect_ = false;
        return true;
    }
    state_ = SystemState::Idle;
    return true;
}

bool StateMachine::requestStop() {
    if (state_ != SystemState::Moving || finishAfterDisconnect_) {
        return false;
    }
    state_ = SystemState::Idle;
    return true;
}

bool StateMachine::fault(FaultCode code) {
    if (code == FaultCode::None) {
        return false;
    }
    if (state_ == SystemState::EstopTriggered ||
        state_ == SystemState::DisconnectLockout ||
        state_ == SystemState::Boot) {
        return false;
    }
    state_ = SystemState::Error;
    fault_ = code;
    finishAfterDisconnect_ = false;
    if (code == FaultCode::HomingFailed || code == FaultCode::SelfCheckFailed ||
        code == FaultCode::Watchdog) {
        homed_ = false;
    }
    return true;
}

bool StateMachine::clearFault() {
    if (state_ != SystemState::Error) {
        return false;
    }
    state_ = SystemState::Idle;
    fault_ = FaultCode::None;
    return true;
}

bool StateMachine::setPayload(Motion::PayloadProfile profile) {
    if (state_ != SystemState::Idle) {
        return false;
    }
    payload_ = profile;
    return true;
}

void StateMachine::notifyEstop() {
    // Pre-empts disconnect-finish, homing, and motion. The latch stays set
    // after the switch is released (spec §5.1).
    state_ = SystemState::EstopTriggered;
    estopLatched_ = true;
    clearEstopRequested_ = false;
    finishAfterDisconnect_ = false;
    homed_ = false;
}

bool StateMachine::requestClearEstop() {
    if (state_ != SystemState::EstopTriggered) {
        return false;
    }
    clearEstopRequested_ = true;
    return true;
}

void StateMachine::enterHoming() {
    state_ = SystemState::Homing;
    fault_ = FaultCode::None;
    homed_ = false;
    finishAfterDisconnect_ = false;
    clearEstopRequested_ = false;
}

bool StateMachine::requestHome(bool lineSafe) {
    switch (state_) {
        case SystemState::Idle:
        case SystemState::Error:
            break;
        case SystemState::EstopTriggered:
            // clear_estop AND a closed e-stop circuit AND this re-home command.
            if (!clearEstopRequested_ || !lineSafe) {
                return false;
            }
            estopLatched_ = false;
            break;
        case SystemState::DisconnectLockout:
            if (!linkUp_) {
                return false;
            }
            break;
        default:
            return false;
    }
    enterHoming();
    return true;
}

void StateMachine::notifyDisconnect() {
    linkUp_ = false;
    if (state_ == SystemState::EstopTriggered) {
        return;
    }
    if (state_ == SystemState::Moving) {
        // Option A: do not abort. The caller finishes the trajectory, then
        // notifyMoveComplete() enters DISCONNECT_LOCKOUT.
        finishAfterDisconnect_ = true;
        return;
    }
    if (state_ == SystemState::DisconnectLockout) {
        return;
    }
    state_ = SystemState::DisconnectLockout;
    homed_ = false;
    finishAfterDisconnect_ = false;
}

void StateMachine::notifyReconnect() {
    linkUp_ = true;
}

bool StateMachine::acceptsMove() const {
    return state_ == SystemState::Idle && homed_ && !estopLatched_ && linkUp_;
}

}  // namespace Control
