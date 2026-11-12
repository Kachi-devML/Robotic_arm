#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <freertos/semphr.h>

#include "HomingSequence.h"
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
std::atomic<bool> gHomeRequestPending{false};
std::atomic<bool> gHomeRequestClearsEstop{false};
std::atomic<bool> gNetworkConnectedPending{false};
std::atomic<bool> gLinkActivityPending{false};
std::atomic<bool> gStatePublishPending{false};
std::atomic<uint32_t> gLastLinkActivityMs{0};

#if !DRIVER_BENCH
Control::StateMachine gState;
Control::HomingSequence gHoming;
Control::SystemState gLoggedState = Control::SystemState::Boot;
bool gHavePublishedState = false;
Control::SystemState gPublishedState = Control::SystemState::Boot;
bool gPublishedHomed = false;
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
    if (!gMotionReady) {
        Tasks::NetworkTask::publishError("INTERNAL", "Motion task is unavailable");
        return;
    }

    if (std::strcmp(requestType, "home_request") == 0) {
        const bool lineSafe =
            SimMode::isEnabled() || digitalRead(Pins::kEstopSw) == LOW;
        const bool wasLatched = gSafety.isLatched();
        if (!lineSafe ||
            (wasLatched && gState.state() != Control::SystemState::EstopTriggered) ||
            !gState.requestHome(lineSafe)) {
            Tasks::NetworkTask::publishError(
                "BAD_COMMAND", "Home rejected in the current state or while e-stop is active");
            return;
        }
        gHomeRequestClearsEstop.store(wasLatched, std::memory_order_relaxed);
        gHomeRequestPending.store(true, std::memory_order_release);
        return;
    }

    if (std::strcmp(requestType, "clear_estop") == 0) {
        if (!gState.requestClearEstop()) {
            Tasks::NetworkTask::publishError(
                "BAD_COMMAND", "Clear e-stop is only valid while e-stop is latched");
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

bool homeSwitchTriggered(int32_t step) {
    if (SimMode::isEnabled()) {
        // Virtual switch at step 0 so sim boot can finish homing with no GPIO.
        return step <= 0;
    }
    // External pull-up: pressed (to GND) reads LOW.
    return digitalRead(Pins::kHomeSw) == LOW;
}

void snapServosHome() {
    const float shoulder = JointLimits::radToDeg(JointLimits::kTheta1Home_rad);
    const float elbow = JointLimits::radToDeg(JointLimits::kTheta2Home_rad);
    const float wrist = JointLimits::radToDeg(JointLimits::kTheta3Home_rad);
    const float gripper = JointLimits::radToDeg(JointLimits::kGripperHome_rad);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelShoulder, shoulder,
                                     Motion::ServoDriver::kShoulderPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelElbow, elbow,
                                     Motion::ServoDriver::kElbowPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelWrist, wrist,
                                     Motion::ServoDriver::kWristPulse);
    Motion::ServoDriver::setAngleDeg(Motion::ServoDriver::kChannelGripper, gripper,
                                     Motion::ServoDriver::kGripperPulse);
}

void applyHoming(const Control::HomingSequence::Status& st) {
    if (st.snapServos) {
        snapServosHome();
    }
    if (st.zeroNow) {
        Motion::StepperDriver::zeroPosition();
        setStepperEnabled(false);
        return;
    }
    if (st.phase == Control::HomingSequence::Phase::Failed ||
        st.phase == Control::HomingSequence::Phase::Aborted) {
        setStepperEnabled(false);
        return;
    }
    setStepperEnabled(st.enableStepper);
    if (st.enableStepper) {
        Motion::StepperDriver::setTargetStep(st.targetStep);
    }
}

void logState() {
    lockState();
    const Control::SystemState state = gState.state();
    const Control::FaultCode fault = gState.fault();
    const bool homed = gState.homed();
    const Motion::PayloadProfile payload = gState.payload();
    unlockState();

    if (state == gLoggedState) {
        return;
    }
    LOG_INFO("state %s -> %s fault=%s homed=%d payload=%s",
             Control::stateName(gLoggedState), Control::stateName(state),
             Control::faultName(fault), homed ? 1 : 0,
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
        case Control::SystemState::Homing:
            on = ((now / 200) % 2) == 0;
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
    const bool homed = gState.homed();
    const Motion::PayloadProfile payload = gState.payload();
    const Control::GripperPose gripper = gState.gripper();
    unlockState();

    if (!force && gHavePublishedState && state == gPublishedState &&
        homed == gPublishedHomed && payload == gPublishedPayload &&
        gripper == gPublishedGripper) {
        return;
    }

    Tasks::NetworkTask::publishState(
        Control::stateName(state), homed,
        payload == Motion::PayloadProfile::Loaded ? "loaded" : "empty",
        gripper == Control::GripperPose::Closed ? "closed" : "open");
    gPublishedState = state;
    gPublishedHomed = homed;
    gPublishedPayload = payload;
    gPublishedGripper = gripper;
    gHavePublishedState = true;
}

void beginControl(uint32_t now) {
    gState.reset();
    gState.beginSelfCheck();
    if (gSafety.isLatched()) {
        gState.notifyEstop();
        LOG_ERROR("e-stop line open at boot — motion latched off");
        return;
    }
    if (!gState.reportSelfCheck(gMotionReady)) {
        if (!gMotionReady) {
            LOG_ERROR("self-check failed: MotionTask unavailable");
        }
        return;
    }
    gHoming.start(now);
    LOG_INFO("homing: servos to 90 deg, gripper open, base seeking switch");
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
    Logger::begin(115200);
    pinMode(Pins::kStatusLed, OUTPUT);
    digitalWrite(Pins::kStatusLed, LOW);
    if (!SimMode::isEnabled()) {
        pinMode(Pins::kHomeSw, INPUT);
    }

    LOG_INFO("M5 control boot (ESP32-S3) sim=%d bench=%d", SimMode::isEnabled() ? 1 : 0,
             DRIVER_BENCH);
    LOG_INFO("pins STEP=%u DIR=%u EN=%u HOME=%u ESTOP=%u  estop HIGH=triggered",
             Pins::kNemaStep, Pins::kNemaDir, Pins::kNemaEn, Pins::kHomeSw, Pins::kEstopSw);

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

    const uint32_t now = millis();

#if !DRIVER_BENCH
    beginControl(now);
#endif

    if (!Tasks::NetworkTask::begin()) {
        LOG_ERROR("NetworkTask failed to start");
    }

#if DRIVER_BENCH
    enter(BenchPhase::CommandServos);
    LOG_INFO("DRIVER_BENCH: servos -> %.0f deg (gripper %.0f), stepper +/-%ld steps",
             kBenchServoDeg, kBenchGripperDeg, static_cast<long>(kBenchSteps));
#else
    lockState();
    const bool startHoming = gState.state() == Control::SystemState::Homing;
    unlockState();
    if (startHoming && !gSafety.isLatched()) {
        gHoming.start(millis());
    }
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
    if (gHomeRequestPending.exchange(false, std::memory_order_acquire)) {
        const bool clearsEstop =
            gHomeRequestClearsEstop.exchange(false, std::memory_order_relaxed);
        const bool lineSafe =
            SimMode::isEnabled() || digitalRead(Pins::kEstopSw) == LOW;
        if (!lineSafe || (gSafety.isLatched() && !clearsEstop)) {
            lockState();
            gState.notifyEstop();
            unlockState();
            Tasks::NetworkTask::publishError(
                "BAD_COMMAND", "E-stop became active before homing started");
        } else {
            if (gSafety.isLatched()) {
                gSafety.releaseLatch();
            }
            if (gSafety.isLatched() ||
                (!SimMode::isEnabled() && digitalRead(Pins::kEstopSw) == HIGH)) {
                lockState();
                gState.notifyEstop();
                unlockState();
                Tasks::NetworkTask::publishError(
                    "BAD_COMMAND", "E-stop became active before homing started");
            } else {
                gHoming.start(now);
                LOG_INFO("home request accepted");
            }
        }
    }

    if (gSafety.pollWatchdog(now)) {
        forceSafeOutputs();
        gHoming.abort();
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
        if (before == Control::SystemState::Homing) {
            gHoming.abort();
            forceSafeOutputs();
            LOG_WARN("link lost during homing — lockout");
        } else if (before == Control::SystemState::Moving) {
            LOG_WARN("link lost during move — finishing trajectory, then lockout");
        } else if (after == Control::SystemState::DisconnectLockout) {
            LOG_WARN("link lost — disconnect lockout");
        }
    }

    lockState();
    const bool isHoming = gState.state() == Control::SystemState::Homing;
    unlockState();
    if (isHoming && !gSafety.isLatched()) {
        const int32_t step = Motion::StepperDriver::currentStep();
        const auto st = gHoming.tick(now, homeSwitchTriggered(step), step);
        applyHoming(st);
        if (st.phase == Control::HomingSequence::Phase::Done) {
            lockState();
            gState.notifyHomingComplete();
            unlockState();
            LOG_INFO("homing complete, step zeroed");
        } else if (st.phase == Control::HomingSequence::Phase::Failed) {
            setStepperEnabled(false);
            lockState();
            gState.notifyHomingFailed();
            unlockState();
            LOG_ERROR("homing failed: %s", st.failReason != nullptr ? st.failReason : "?");
        }
    }

    // Rising edge may have landed during the homing tick. Cut again and latch
    // the state. Releasing the switch does not leave ESTOP_TRIGGERED.
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
        gHoming.abort();
        forceSafeOutputs();
        LOG_ERROR("e-stop latched (HIGH = open circuit). clear_estop then re-home");
    }

    publishCurrentState(
        gStatePublishPending.exchange(false, std::memory_order_acquire));

    logState();
    updateLed(now);
    gSafety.kick(now);
#endif
}
