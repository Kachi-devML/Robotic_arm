// Unit tests for milestone 5: state machine, homing sequence, safety latch.
//
// Run:  pio test -e native -f test_control

#include <unity.h>

#include "HomingSequence.h"
#include "MotionConfig.h"
#include "SafetyMonitor.h"
#include "StateMachine.h"

void setUp(void) {}
void tearDown(void) {}

static void bootToIdle(Control::StateMachine& sm) {
    sm.reset();
    TEST_ASSERT_TRUE(sm.beginSelfCheck());
    TEST_ASSERT_TRUE(sm.reportSelfCheck(true));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Homing),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.notifyHomingComplete());
}

static int32_t runHoming(Control::HomingSequence& home, int32_t switchAt,
                         bool* sawSnap, bool* sawZero, const char** reason) {
    home.start(0);
    int32_t step = 0;
    bool snap = false;
    bool zero = false;
    Control::HomingSequence::Phase phase = Control::HomingSequence::Phase::Snap;
    for (uint32_t t = 0; t <= 20000 && phase != Control::HomingSequence::Phase::Done &&
                         phase != Control::HomingSequence::Phase::Failed;
         t += 5) {
        const bool triggered = step <= switchAt;
        auto st = home.tick(t, triggered, step);
        phase = st.phase;
        if (st.snapServos) {
            snap = true;
        }
        if (st.zeroNow) {
            zero = true;
        }
        if (st.enableStepper) {
            step = st.targetStep;
        }
        if (st.failReason != nullptr && reason != nullptr) {
            *reason = st.failReason;
        }
    }
    if (sawSnap != nullptr) {
        *sawSnap = snap;
    }
    if (sawZero != nullptr) {
        *sawZero = zero;
    }
    return step;
}

void test_boot_homes_into_idle_empty(void) {
    Control::StateMachine sm;
    sm.reset();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Boot),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_FALSE(sm.homed());
    bootToIdle(sm);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.homed());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::GripperPose::Open),
                      static_cast<int>(sm.gripper()));
    TEST_ASSERT_EQUAL_STRING("IDLE", Control::stateName(sm.state()));
}

void test_self_check_failure(void) {
    Control::StateMachine sm;
    sm.reset();
    sm.beginSelfCheck();
    TEST_ASSERT_FALSE(sm.reportSelfCheck(false));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Error),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::FaultCode::SelfCheckFailed),
                      static_cast<int>(sm.fault()));
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));
}

void test_move_rejected_until_homed(void) {
    Control::StateMachine sm;
    sm.reset();
    sm.beginSelfCheck();
    sm.reportSelfCheck(true);
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Closed));
    TEST_ASSERT_TRUE(sm.notifyHomingComplete());
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Closed));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::GripperPose::Closed),
                      static_cast<int>(sm.gripper()));
    TEST_ASSERT_TRUE(sm.notifyMoveComplete());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
}

void test_payload_rejected_while_moving_and_resets_after_rehome(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.setPayload(Motion::PayloadProfile::Loaded));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_FALSE(sm.setPayload(Motion::PayloadProfile::Empty));
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Loaded),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.requestStop());
    TEST_ASSERT_TRUE(sm.requestHome(true));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Homing),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_TRUE(sm.notifyHomingComplete());
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
}

void test_fault_then_valid_move_recovers(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.fault(Control::FaultCode::TargetUnreachable));
    TEST_ASSERT_EQUAL_STRING("TARGET_UNREACHABLE", Control::faultName(sm.fault()));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Error),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::FaultCode::None),
                      static_cast<int>(sm.fault()));
}

void test_watchdog_fault_requires_rehome(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.fault(Control::FaultCode::Watchdog));
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_TRUE(sm.requestHome(true));
    TEST_ASSERT_TRUE(sm.notifyHomingComplete());
    TEST_ASSERT_TRUE(sm.acceptsMove());
}

void test_clear_fault_returns_to_idle(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    sm.fault(Control::FaultCode::BadCommand);
    TEST_ASSERT_TRUE(sm.clearFault());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.homed());
}

void test_estop_latches_until_clear_and_rehome(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    sm.requestMove(Control::GripperPose::Open);
    sm.notifyEstop();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.estopLatched());
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_FALSE(sm.acceptsMove());

    // Released line, no clear command: stay latched.
    TEST_ASSERT_FALSE(sm.requestHome(true));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));

    TEST_ASSERT_TRUE(sm.requestClearEstop());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));

    // Clear recorded, but the circuit is still open.
    TEST_ASSERT_FALSE(sm.requestHome(false));

    TEST_ASSERT_TRUE(sm.requestHome(true));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Homing),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.estopLatched());
    sm.notifyHomingComplete();
    TEST_ASSERT_TRUE(sm.acceptsMove());
}

void test_estop_preempts_disconnect_finish(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    sm.requestMove(Control::GripperPose::Closed);
    sm.notifyDisconnect();
    TEST_ASSERT_TRUE(sm.finishAfterDisconnect());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    sm.notifyEstop();
    TEST_ASSERT_FALSE(sm.finishAfterDisconnect());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));
}

void test_disconnect_while_moving_finishes_then_locks(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    sm.requestMove(Control::GripperPose::Open);
    sm.notifyDisconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.requestStop());
    sm.notifyReconnect();
    TEST_ASSERT_TRUE(sm.notifyMoveComplete());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_TRUE(sm.linkUp());
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_TRUE(sm.requestHome(true));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Homing),
                      static_cast<int>(sm.state()));
}

void test_disconnect_while_idle_locks_immediately(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    sm.notifyDisconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.requestHome(true));
    sm.notifyReconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.requestHome(true));
}

void test_disconnect_aborts_homing_into_lockout(void) {
    Control::StateMachine sm;
    sm.reset();
    sm.beginSelfCheck();
    sm.reportSelfCheck(true);
    sm.notifyDisconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.notifyHomingComplete());
}

void test_homing_failure_sets_error(void) {
    Control::StateMachine sm;
    sm.reset();
    sm.beginSelfCheck();
    sm.reportSelfCheck(true);
    TEST_ASSERT_TRUE(sm.notifyHomingFailed());
    TEST_ASSERT_EQUAL_STRING("HOMING_FAILED", Control::faultName(sm.fault()));
    TEST_ASSERT_FALSE(sm.homed());
    TEST_ASSERT_TRUE(sm.requestHome(true));
}

void test_homing_switch_already_closed_then_creep(void) {
    Control::HomingSequence home;
    bool snap = false;
    bool zero = false;
    // Switch pressed at step <= 0, which is where the sequence starts.
    const int32_t step = runHoming(home, 0, &snap, &zero, nullptr);
    TEST_ASSERT_TRUE(snap);
    TEST_ASSERT_TRUE(zero);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Done),
                      static_cast<int>(home.tick(30000, true, step).phase));
    TEST_ASSERT_TRUE(step <= 0);
}

void test_homing_seek_then_two_stage(void) {
    Control::HomingSequence home;
    bool zero = false;
    const int32_t step = runHoming(home, -400, nullptr, &zero, nullptr);
    TEST_ASSERT_TRUE(zero);
    TEST_ASSERT_TRUE(step <= -400);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Done),
                      static_cast<int>(home.tick(30000, true, step).phase));
}

void test_homing_max_travel_without_switch(void) {
    Control::HomingSequence home;
    const char* reason = nullptr;
    runHoming(home, -1000000, nullptr, nullptr, &reason);
    auto st = home.tick(1, false, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Failed),
                      static_cast<int>(st.phase));
    TEST_ASSERT_EQUAL_STRING("max rotation", reason);
}

void test_homing_timeout_when_motor_does_not_move(void) {
    Control::HomingSequence home;
    home.start(0);
    auto first = home.tick(0, false, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Snap),
                      static_cast<int>(first.phase));
    // Stalled: the physical step never follows the command.
    auto stalled = home.tick(MotionConfig::kHomingTimeoutMs, false, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Failed),
                      static_cast<int>(stalled.phase));
    TEST_ASSERT_EQUAL_STRING("timeout", stalled.failReason);
    TEST_ASSERT_FALSE(stalled.enableStepper);
}

void test_homing_stuck_switch(void) {
    Control::HomingSequence home;
    home.start(0);
    int32_t step = 0;
    Control::HomingSequence::Phase phase = Control::HomingSequence::Phase::Snap;
    const char* reason = nullptr;
    for (uint32_t t = 0; t <= 10000 && phase != Control::HomingSequence::Phase::Failed;
         t += 5) {
        auto st = home.tick(t, true, step);
        phase = st.phase;
        reason = st.failReason;
        if (st.enableStepper) {
            step = st.targetStep;
        }
    }
    TEST_ASSERT_EQUAL(static_cast<int>(Control::HomingSequence::Phase::Failed),
                      static_cast<int>(phase));
    TEST_ASSERT_EQUAL_STRING("switch stuck", reason);
}

void test_homing_snap_is_one_shot(void) {
    Control::HomingSequence home;
    home.start(100);
    int snaps = 0;
    int32_t step = 50;
    for (uint32_t t = 100; t < 250; t += 10) {
        auto st = home.tick(t, false, step);
        if (st.snapServos) {
            ++snaps;
        }
        TEST_ASSERT_FALSE(st.enableStepper);
    }
    TEST_ASSERT_EQUAL(1, snaps);
}

void test_safety_estop_latch_and_hard_cut(void) {
    static int cuts = 0;
    cuts = 0;
    Control::SafetyMonitor safety;
    safety.setHardCut([]() { ++cuts; });
    TEST_ASSERT_FALSE(safety.isLatched());
    safety.onEstopEdge();
    TEST_ASSERT_TRUE(safety.isLatched());
    TEST_ASSERT_EQUAL(1, cuts);
    safety.onEstopEdge();
    TEST_ASSERT_EQUAL(2, cuts);
    safety.releaseLatch();
    TEST_ASSERT_FALSE(safety.isLatched());
    TEST_ASSERT_EQUAL(2, cuts);
}

void test_safety_link_timeout_is_two_missed_heartbeats(void) {
    Control::SafetyMonitor safety;
    TEST_ASSERT_FALSE(safety.pollLink(5000));
    safety.startLinkSupervision(0);
    TEST_ASSERT_FALSE(safety.pollLink(MotionConfig::kLinkTimeoutMs - 1));
    TEST_ASSERT_TRUE(safety.pollLink(MotionConfig::kLinkTimeoutMs));
    TEST_ASSERT_FALSE(safety.pollLink(MotionConfig::kLinkTimeoutMs + 10));
    safety.noteLinkActivity(MotionConfig::kLinkTimeoutMs + 10);
    TEST_ASSERT_TRUE(safety.pollLink((MotionConfig::kLinkTimeoutMs + 10) +
                                     MotionConfig::kLinkTimeoutMs));
}

void test_safety_watchdog_edges_once(void) {
    Control::SafetyMonitor safety;
    TEST_ASSERT_FALSE(safety.pollWatchdog(1000));
    safety.kick(0);
    TEST_ASSERT_FALSE(safety.pollWatchdog(MotionConfig::kWatchdogTimeoutMs - 1));
    TEST_ASSERT_TRUE(safety.pollWatchdog(MotionConfig::kWatchdogTimeoutMs));
    TEST_ASSERT_FALSE(safety.pollWatchdog(MotionConfig::kWatchdogTimeoutMs + 5));
    safety.kick(MotionConfig::kWatchdogTimeoutMs + 5);
    TEST_ASSERT_FALSE(safety.pollWatchdog(MotionConfig::kWatchdogTimeoutMs + 5));
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_boot_homes_into_idle_empty);
    RUN_TEST(test_self_check_failure);
    RUN_TEST(test_move_rejected_until_homed);
    RUN_TEST(test_payload_rejected_while_moving_and_resets_after_rehome);
    RUN_TEST(test_fault_then_valid_move_recovers);
    RUN_TEST(test_watchdog_fault_requires_rehome);
    RUN_TEST(test_clear_fault_returns_to_idle);
    RUN_TEST(test_estop_latches_until_clear_and_rehome);
    RUN_TEST(test_estop_preempts_disconnect_finish);
    RUN_TEST(test_disconnect_while_moving_finishes_then_locks);
    RUN_TEST(test_disconnect_while_idle_locks_immediately);
    RUN_TEST(test_disconnect_aborts_homing_into_lockout);
    RUN_TEST(test_homing_failure_sets_error);
    RUN_TEST(test_homing_switch_already_closed_then_creep);
    RUN_TEST(test_homing_seek_then_two_stage);
    RUN_TEST(test_homing_max_travel_without_switch);
    RUN_TEST(test_homing_timeout_when_motor_does_not_move);
    RUN_TEST(test_homing_stuck_switch);
    RUN_TEST(test_homing_snap_is_one_shot);
    RUN_TEST(test_safety_estop_latch_and_hard_cut);
    RUN_TEST(test_safety_link_timeout_is_two_missed_heartbeats);
    RUN_TEST(test_safety_watchdog_edges_once);
    return UNITY_END();
}
