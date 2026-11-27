// Unit tests for the state machine and safety monitor.
//
// Run:  pio test -e native -f test_control

#include <unity.h>

#include "MotionConfig.h"
#include "SafetyMonitor.h"
#include "StateMachine.h"

void setUp(void) {}
void tearDown(void) {}

static void bootToIdle(Control::StateMachine& sm) {
    sm.reset();
    TEST_ASSERT_TRUE(sm.begin());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
}

void test_boot_starts_idle_and_accepts_move(void) {
    Control::StateMachine sm;
    sm.reset();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Boot),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.begin());
    TEST_ASSERT_EQUAL_STRING("IDLE", Control::stateName(sm.state()));
    TEST_ASSERT_TRUE(sm.acceptsMove());
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Closed));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
}

void test_payload_rejected_while_moving(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.setPayload(Motion::PayloadProfile::Loaded));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_FALSE(sm.setPayload(Motion::PayloadProfile::Empty));
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Loaded),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.requestStop());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
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

void test_watchdog_error_recovers_on_valid_move(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.fault(Control::FaultCode::Watchdog));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
}

void test_clear_fault_returns_to_idle(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.fault(Control::FaultCode::BadCommand));
    TEST_ASSERT_TRUE(sm.clearFault());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
}

void test_estop_requires_explicit_clear_and_resets_payload(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.setPayload(Motion::PayloadProfile::Loaded));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    sm.notifyEstop();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.estopLatched());
    TEST_ASSERT_FALSE(sm.acceptsMove());

    // Releasing the physical switch alone does not clear the state-machine latch.
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.requestClearEstop());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.estopLatched());
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.acceptsMove());
}

void test_estop_preempts_disconnect_finish(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Closed));
    sm.notifyDisconnect();
    TEST_ASSERT_TRUE(sm.finishAfterDisconnect());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    sm.notifyEstop();
    TEST_ASSERT_FALSE(sm.finishAfterDisconnect());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::EstopTriggered),
                      static_cast<int>(sm.state()));
}

void test_disconnect_while_moving_finishes_then_locks_until_reconnect(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.setPayload(Motion::PayloadProfile::Loaded));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    sm.notifyDisconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.requestStop());
    TEST_ASSERT_TRUE(sm.notifyMoveComplete());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));

    sm.notifyReconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.linkUp());
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.acceptsMove());
}

void test_reconnect_during_move_returns_idle_on_completion(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
    sm.notifyDisconnect();
    sm.notifyReconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Moving),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_TRUE(sm.notifyMoveComplete());
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
}

void test_disconnect_while_idle_locks_until_reconnect(void) {
    Control::StateMachine sm;
    bootToIdle(sm);
    TEST_ASSERT_TRUE(sm.setPayload(Motion::PayloadProfile::Loaded));
    sm.notifyDisconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::DisconnectLockout),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_FALSE(sm.requestMove(Control::GripperPose::Open));

    sm.notifyReconnect();
    TEST_ASSERT_EQUAL(static_cast<int>(Control::SystemState::Idle),
                      static_cast<int>(sm.state()));
    TEST_ASSERT_EQUAL(static_cast<int>(Motion::PayloadProfile::Empty),
                      static_cast<int>(sm.payload()));
    TEST_ASSERT_TRUE(sm.requestMove(Control::GripperPose::Open));
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
    RUN_TEST(test_boot_starts_idle_and_accepts_move);
    RUN_TEST(test_payload_rejected_while_moving);
    RUN_TEST(test_fault_then_valid_move_recovers);
    RUN_TEST(test_watchdog_error_recovers_on_valid_move);
    RUN_TEST(test_clear_fault_returns_to_idle);
    RUN_TEST(test_estop_requires_explicit_clear_and_resets_payload);
    RUN_TEST(test_estop_preempts_disconnect_finish);
    RUN_TEST(test_disconnect_while_moving_finishes_then_locks_until_reconnect);
    RUN_TEST(test_reconnect_during_move_returns_idle_on_completion);
    RUN_TEST(test_disconnect_while_idle_locks_until_reconnect);
    RUN_TEST(test_safety_estop_latch_and_hard_cut);
    RUN_TEST(test_safety_link_timeout_is_two_missed_heartbeats);
    RUN_TEST(test_safety_watchdog_edges_once);
    return UNITY_END();
}
