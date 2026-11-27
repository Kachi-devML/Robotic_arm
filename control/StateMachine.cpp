#include "StateMachine.h"

namespace Control {

const char* stateName(SystemState state) {
    switch (state) {
        case SystemState::Boot:
            return "BOOT";
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
    linkUp_ = true;
    estopLatched_ = false;
    finishAfterDisconnect_ = false;
}

bool StateMachine::begin() {
    if (state_ != SystemState::Boot) {
        return false;
    }
    state_ = SystemState::Idle;
    fault_ = FaultCode::None;
    finishAfterDisconnect_ = false;
    return true;
}

bool StateMachine::requestMove(GripperPose gripper) {
    if (state_ == SystemState::Error) {
        if (estopLatched_ || !linkUp_) {
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
        // A reconnect may arrive before the finishing trajectory completes.
        state_ = linkUp_ ? SystemState::Idle : SystemState::DisconnectLockout;
        if (linkUp_) {
            payload_ = Motion::PayloadProfile::Empty;
        }
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
    // Pre-empts disconnect-finish and motion. The latch stays set after the
    // switch is released (spec §5.1).
    state_ = SystemState::EstopTriggered;
    estopLatched_ = true;
    finishAfterDisconnect_ = false;
}

bool StateMachine::requestClearEstop() {
    if (state_ != SystemState::EstopTriggered) {
        return false;
    }
    state_ = SystemState::Idle;
    fault_ = FaultCode::None;
    payload_ = Motion::PayloadProfile::Empty;
    estopLatched_ = false;
    finishAfterDisconnect_ = false;
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
    finishAfterDisconnect_ = false;
}

void StateMachine::notifyReconnect() {
    linkUp_ = true;
    if (state_ == SystemState::DisconnectLockout) {
        state_ = SystemState::Idle;
        fault_ = FaultCode::None;
        payload_ = Motion::PayloadProfile::Empty;
        finishAfterDisconnect_ = false;
    }
}

bool StateMachine::acceptsMove() const {
    return state_ == SystemState::Idle && !estopLatched_ && linkUp_;
}

}  // namespace Control
