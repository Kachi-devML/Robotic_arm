# 4-DoF Robotic Arm — Firmware & Web App Build Specification

**Revision 2.** Read this entire document before writing or changing any code. This is a complete architecture and behavior specification for a 4-DoF robotic arm controller. You (the coding agent) are building two things:

1. **ESP32-S3 firmware** (PlatformIO, Arduino core, FreeRTOS)
2. **Web application** (browser-based control + educational visualizer)

The top priority alongside correctness is **modularity**: every file should have one clear responsibility, be independently testable, and be easy for a human to read and debug. Do not write monolithic files. Do not use `delay()` anywhere in motion-related code. Do not hardcode magic numbers outside designated config files.

Build in the milestone order given at the end of this document — do not attempt to generate the entire system in one pass.

---

## 0. Revision 2 Summary (read first)

Milestones 1–9 were already built against Revision 1 of this spec. Revision 2 changes the hardware in three ways. **Adapt the existing code base; do not rewrite it.** Sections tagged **[REV 2]** contain the new requirements, and Section 12 gives the exact migration order and the list of files that must NOT change.

| # | Change | Where it lands |
|---|--------|----------------|
| A | **PCA9685 removed.** The four servo signal wires now go directly to ESP32 GPIOs and are driven by the ESP32 LEDC hardware PWM. Servos are powered by an external supply with a common ground. | §2.1, §2.2, §2.5, §6.3, §10; `Pins.h`, `ServoConfig.h` (new), `ServoDriver`, `platformio.ini`, `SimMode` |
| B | **Base stepper now sits behind a 5:1 planetary gearbox.** Every base angle in the system remains an *output-shaft* angle; gearing is converted to steps in exactly one place (`StepperDriver`). Microstepping is re-chosen. | §2.1, §2.6, §4, §6.1, §6.4; `MotionConfig.h` (stepper block), `StepperDriver`, `HomingSequence` constants |
| C | **Stepper coil power management.** The DRV8825 is enabled (EN low) for homing and motion and released (EN high) after a short idle delay. | §6.5; `StepperDriver`, `MotionTask`, `SafetyMonitor` (enable gating) |
| D | **Corrections to Revision 1 text:** the e-stop wiring sentence contradicted its own polarity rule (fixed in §2.3); the ESP32-S3 has no "input-only" GPIOs, so that remark was dropped from the pin tables. | §2.2, §2.3 |

**Design principle that keeps this change small:** everything above the hardware-driver layer (kinematics, S-curve, planner, message schema, web app) works in output-shaft **degrees** and is unaware of gearing, microstepping, PWM or I²C. Those details must stay hidden inside `StepperDriver` and `ServoDriver`.

---

## 1. System Overview

A 4-DoF robotic arm (base twist, shoulder, elbow, wrist) with a 1-DoF gripper, controlled by an ESP32-S3. The user targets a point in 3D space via a web app; the firmware solves inverse kinematics analytically, generates a jerk-limited motion profile, and drives a geared stepper (base) and three servos (shoulder/elbow/wrist) plus a gripper servo, synchronized in time.

The web app is also an **educational tool**: it must show the live IK derivation (numeric, step-by-step) and a 2D geometric diagram of both candidate solutions (elbow-up / elbow-down) before the user commits to a move.

### Joint naming convention

Use **θ0, θ1, θ2, θ3** for base, shoulder, elbow, wrist respectively (not θ1–θ4). This must be reflected consistently in variable names, JSON field names, and UI labels.

---

## 2. Hardware Reference

### 2.1 Bill of materials **[REV 2]**

| Joint | Actuator | Driver / interface |
|-|-|-|
| Base (θ0) | NEMA17 stepper + **5:1 planetary gearbox** | DRV8825 |
| Shoulder (θ1) | YSIDO 35KG servo | **Direct ESP32 GPIO (LEDC PWM)**, external servo supply |
| Elbow (θ2) | RCXAZ 14.5KG servo | **Direct ESP32 GPIO (LEDC PWM)**, external servo supply |
| Wrist (θ3) | MG90s servo | **Direct ESP32 GPIO (LEDC PWM)**, external servo supply |
| Gripper | SG90 servo | **Direct ESP32 GPIO (LEDC PWM)**, external servo supply |
| Homing | SS-5GL limit switch | GPIO (base only) |
| E-stop | Hardware N.C. switch | GPIO (see 2.3) |

The PCA9685 and the I²C bus are no longer used. Do not initialise `Wire` and do not link any PCA9685 library.

### 2.2 Pin map (ESP32-S3) **[REV 2]**

| GPIO | Net Name | Destination | Function |
|-|-|-|-|
| GPIO2 | STATUS_LED | LED + resistor | Status |
| GPIO10 | NEMA_STEP | DRV8825 STEP | Step pulse |
| GPIO11 | NEMA_DIR | DRV8825 DIR | Direction |
| GPIO12 | NEMA_EN | DRV8825 ENABLE | Driver enable, **active-low** (LOW = coils energised, HIGH = coils released). Verify against your DRV8825 board. Needs an external 10 kΩ pull-up to 3.3 V (see 2.6). |
| GPIO13 | NEMA_M0 | DRV8825 M0 | Microstep select |
| GPIO14 | NEMA_M1 | DRV8825 M1 | Microstep select |
| GPIO15 | NEMA_M2 | DRV8825 M2 | Microstep select |
| GPIO16 | HOME_SW | SS-5GL (via 10 kΩ) | Home input, external pull-up required |
| GPIO17 | ESTOP_SW | E-stop switch (via 10 kΩ) | E-stop input, external pull-up required (see 2.3) |
| **GPIO4** | **SERVO_SHOULDER_PWM** | YSIDO 35KG signal | LEDC PWM, 50 Hz |
| **GPIO5** | **SERVO_ELBOW_PWM** | RCXAZ 14.5KG signal | LEDC PWM, 50 Hz |
| **GPIO6** | **SERVO_WRIST_PWM** | MG90s signal | LEDC PWM, 50 Hz |
| **GPIO7** | **SERVO_GRIPPER_PWM** | SG90 signal | LEDC PWM, 50 Hz |
| GPIO8, GPIO9 | (free) | formerly I2C_SDA / I2C_SCL | Unused. Do not repurpose without updating this table. |

Servo pin choice: GPIO4–7 are plain GPIOs that are not strapping pins (0, 3, 45, 46), not USB (19, 20), not UART0 (43, 44), and not used by flash/PSRAM on common S3 modules. They are proposals: if the user's board differs, only `Pins.h` changes. The four servo outputs share a single LEDC timer (same frequency/resolution) and use four of the S3's eight LEDC channels.

### 2.3 E-stop wiring and polarity

GPIO17 is the hardware E-stop input, with a 10 kΩ external pull-up to 3.3 V.

**Wiring requirement — implement fail-safe:** wire the e-stop switch as **normally-closed (N.C.) between GPIO17 and GND**. With the switch closed (normal operation), the closed circuit pulls the line **LOW**. Pressing the e-stop *or* a broken/disconnected wire opens the circuit, and the pull-up then pulls the line **HIGH**. Firmware must therefore treat **LOW = safe/idle, and any transition to HIGH (open circuit) as triggered** — i.e., the safe state requires an actively-closed circuit. Document this polarity clearly in code comments since it is easy to get backwards.

E-stop must be serviced by a **hardware interrupt (attachInterrupt)**, not polled in a loop, and must be checked/actioned independent of FreeRTOS task scheduling latency wherever possible.

### 2.4 Mechanical link geometry

| Link | Center-to-center length | Used as |
|-|-|-|
| Shoulder → Elbow | 15 cm | **L1** |
| Elbow → Wrist | 12 cm | **L2** |
| Wrist → End-effector servo | 3.5 cm | (component of L3) |
| End-effector servo → Fingertip | 4.5 cm | (component of L3) |
| Base height offset (ground → shoulder axis) | 12 cm | **d1** |

**L3 = 8 cm** (wrist-to-fingertip, treated as one rigid segment — the wrist joint is the last *positioning* joint; the gripper servo only actuates the jaws and does not add a kinematic degree of freedom). All target coordinates (x, y, z) from the user represent the desired **fingertip/gripper position**, not the wrist joint position.

### 2.5 Servo power and signal wiring requirements **[REV 2]**

These are hardware requirements that firmware and documentation must assume. Put a short wiring summary in the project README.

1. **External servo supply, common ground.** Servos are powered from their own supply, never from the ESP32 board, USB, or its 5 V pin. The supply negative, the ESP32 GND and the DRV8825 logic ground must be joined (single star point at the supply negative). Without a common ground the PWM signals have no reference and servos jitter or do not move.
2. **Match voltage to each servo.** The big servos (YSIDO 35KG, RCXAZ 14.5KG) commonly run at higher voltage than the MG90s and SG90 (typically rated about 4.8–6 V). Confirm each servo's rated range; if the large servos use a higher rail, the small ones need a regulated lower rail. Size the supply for simultaneous stall-level peaks of the two large servos, not their idle draw.
3. **Bulk capacitance.** At least 470–1000 µF electrolytic across the servo supply close to the servos (more for the large ones). This prevents supply sag from resetting or glitching the ESP32 logic ground.
4. **Signal lines.** Add a 220–470 Ω series resistor on each signal line (protects the GPIO) and a 10 kΩ pull-down to GND on each signal line (keeps servos quiet while the ESP32 is in reset/boot, before PWM is attached).
5. **3.3 V logic.** Most servos accept a 3.3 V signal. If a servo is erratic or jittery on a known-good supply, add a 5 V-powered level shifter (e.g. 74AHCT125) on that signal rather than changing firmware.
6. **Power-up order.** Prefer powering the ESP32 first or together with the servo supply. The pull-downs above make the other order tolerable, but it should not be relied on.

### 2.6 Base stepper electrical setup (hardware, not firmware) **[REV 2]**

These are bench/hardware steps. Firmware cannot compensate for getting them wrong.

1. **Current limit is set by the VREF trimpot, not by load.** The DRV8825 regulates coil current to the VREF setting regardless of how heavy the load is. A heavy load causes lost steps, not extra driver current. Thermal shutdown happens when the *configured* current is too high for the cooling and duty cycle. The gearbox helps because the same motor current now produces roughly 4.5× the output torque (5 × gearbox efficiency of about 0.9), so the current limit can be **lowered**. It does so only if VREF is actually lowered.
2. **Starting value.** Peak chopping current is `I_peak = VREF / (5 × R_sense)`. With the common 0.100 Ω sense resistors this is `I_peak = 2 × VREF` (check the board's sense resistor markings; some clone boards differ). Start at I_peak ≈ 1.2–1.5 A (VREF ≈ 0.6–0.75 V) for the 2 A-rated motor, and raise it only if the base skips steps under the real payload. In microstepping, RMS coil current is about I_peak / 1.4.
3. **Cooling.** Fit a heatsink and, if the driver still runs hot at the chosen current, a small fan. Place a 100 µF (or larger) electrolytic across the motor supply at the DRV8825 board.
4. **Never connect or disconnect the motor while the driver is powered.**
5. **EN pull-up.** Add a 10 kΩ pull-up from NEMA_EN (GPIO12) to 3.3 V so the driver boots **disabled**. Many DRV8825 boards have a weak internal pull-down on ENABLE, which would otherwise leave the coils energised while the ESP32 is resetting or flashing. Verify with a multimeter that the pin sits HIGH with the ESP32 in reset.
6. **Gearbox ratio.** Many "5:1" NEMA17 planetary gearboxes are not exactly 5.000:1 (5.18:1 is common). The real ratio must be measured (Section 12, step 3d) and entered in one constant.

---

## 3. Kinematics

### 3.1 Coordinate frame

* Origin: center of the base, at ground level.
* Axes: standard right-handed frame — x/y horizontal (x forward), z vertical (up positive).
* Units: **input/output coordinates in centimeters**; **all internal angle math in radians**. Convert to degrees only at the point of driving the hardware and convert back to human-readable degrees only for JSON sent to the UI.

### 3.2 Inputs

* x, y, z (cm) — target fingertip position
* φ (phi, radians or degrees — pick one and be consistent; recommend degrees at the API boundary, radians internally) — user-specified wrist/approach angle, sent from the web app
* posture — `"elbow_up"` or `"elbow_down"`, user-selected after reviewing both candidate solutions
* gripper — `"open"` or `"closed"`

### 3.3 Analytical IK solver

Implement exactly as follows (this is fixed — do not deviate from this derivation):

```
Step 1 — Base twist
θ0 = atan2(y, x)

Step 2 — Reduce to the arm plane
r  = sqrt(x² + y²)
z' = z − d1

Step 3 — Wrist center (back off along the approach angle φ by L3)
rw = r  − L3·cos(φ)
zw = z' − L3·sin(φ)

Step 4 — 2-link solve for the wrist center
Dw² = rw² + zw²
cos(θ2) = (Dw² − L1² − L2²) / (2·L1·L2)
θ2 = atan2( ±sqrt(1 − cos(θ2)²), cos(θ2) )      // ± gives elbow-down / elbow-up
θ1 = atan2(zw, rw) − atan2( L2·sin(θ2), L1 + L2·cos(θ2) )
θ3 = φ − θ1 − θ2

Reachability constraint:
|L1 − L2| ≤ Dw ≤ (L1 + L2)
```

Note the renumbering from the original derivation (θ1→θ0, θ2→θ1, θ3→θ2, θ4→θ3) to match the joint-naming convention in Section 1.

### 3.4 Reachability validation (must run before IK, and again after)

1. **Pre-check**: Dw bounds above. If violated → reject with a specific error (`TARGET_UNREACHABLE`), and the response must still include the computed max/min reach so the UI can display it.
2. **Post-check**: after solving, verify θ0, θ1, θ2, θ3 all fall within the soft joint limits (Section 4). If either posture (elbow_up/elbow_down) violates limits, mark that specific posture as invalid in the response but still return the other if valid — the user picks between whichever postures are actually valid.
3. Both posture solutions (elbow_up AND elbow_down) must always be computed and returned together (with validity flags) so the web app visualizer can display both for the user to compare before committing.

### 3.5 Kinematics module must be pure/hardware-independent

The IK solver, reachability checks, and forward-kinematics (for drawing the diagram) must be implemented as **pure functions with no hardware dependencies** (no GPIO, no LEDC, no FreeRTOS calls) — inputs in, structured result out. This is required so they can be unit-tested on desktop/native environment and reused identically in both the firmware and (if useful) a JS port for client-side preview.

---

## 4. Joint Limits & Homing

**All joint angles in this table, in the JSON API and in the planner are measured at the joint's output.** For the base this means the **gearbox output shaft**, not the motor shaft. **[REV 2]**

| Joint | Full mechanical range | Soft-limited range (firmware-enforced) | Home position |
|-|-|-|-|
| θ0 (base) | 360° (no slip ring confirmed) | **±170° from home** (see assumption below) | 0° = limit-switch trigger point |
| θ1 (shoulder) | 180° | 0°–180° | 90° (vertical/upright) |
| θ2 (elbow) | 180° | 0°–180° | 90° (folded, compact/safe pose) |
| θ3 (wrist) | 180° | 0°–180° | 90° (level/neutral) |
| Gripper | 90° | 0°(open)–90°(closed) | Open |

**Assumption flagged to user**: base is soft-limited to ±170° because no slip ring is confirmed in the BOM — continuous rotation would eventually wind up the power/limit-switch wiring across the base joint. This constant lives in one place (`JointLimits.h`) and is trivial to widen to full 360° later if a slip ring is confirmed.

**Servo range assumption:** the table assumes 180° servos. If a large servo is actually a 270° model, set its angular range and pulse calibration in `ServoConfig.h`; nothing else changes.

### Homing sequence

1. On boot (or on explicit re-home command), **all three servos snap immediately** to their defined home angles (90°, 90°, 90°) and the gripper opens. **[REV 2]** `ServoDriver::begin()` must attach each LEDC channel with its **home pulse already programmed as the initial duty**, so no output ever pulses at an unintended width (e.g. 0° or full-scale) during start-up.
2. **Base then seeks the limit switch**: the driver is enabled (EN low) and allowed to settle (Section 6.5), then the base rotates at a slow, fixed seek speed until `HOME_SW` triggers; back off slightly and re-approach at a slower speed for repeatability (standard two-stage homing); zero the step count at that point. **[REV 2]** Seek speed, back-off distance and the homing timeout/maximum rotation are all expressed in **output-shaft degrees and °/s** in config and converted to steps by `StepperDriver`. Never express them as raw step counts.
3. Only after all four joints report "homed" does the system transition to `IDLE`.
4. If homing fails (switch never triggers within a timeout / max rotation), transition to `ERROR` with a specific `HOMING_FAILED` code.

---

## 5. System State Machine

Implement as an explicit enum + state machine (not scattered flags). States:

```
BOOT → SELF_CHECK → HOMING → IDLE ⇄ MOVING
                                 ↓
                              ERROR (recoverable — bad command, unreachable target, IK failure)
                                 ↓
                        (valid new command) → IDLE

Any state → ESTOP_TRIGGERED (highest priority, pre-empts everything)
  ESTOP_TRIGGERED → (explicit clear_estop command AND re-home) → HOMING

MOVING → (WiFi/WebSocket disconnect detected) → finish current trajectory → DISCONNECT_LOCKOUT
  DISCONNECT_LOCKOUT → (reconnect + explicit re-home command) → HOMING
```

### 5.1 E-stop behavior

* Triggering: immediate — stop generating step pulses (disable DRV8825 via `NEMA_EN`, i.e. drive it HIGH), freeze all servo PWM outputs at their current commanded position (do not continue ramping). **[REV 2]** With direct LEDC outputs, "freeze" means the servo update code stops writing new duty values; the LEDC hardware keeps emitting the last pulse width, so servos hold position exactly as before. E-stop does not cut servo power.
* This is a **hard cut, not a decelerated stop** — e-stop exists for emergencies, correctness of a smooth stop is not the goal.
* Latching: once triggered, stays in `ESTOP_TRIGGERED` regardless of switch state, until an explicit operator-issued `clear_estop` command is received over the web app **and** the system successfully re-homes. This prevents silently resuming into an unknown physical state.
* Note to user in code comments: since there's no relay cutting actual motor power, a released e-stop switch alone should never be sufficient to resume motion — the explicit clear + re-home requirement is what makes this safe with the current hardware.
* **[REV 2]** `NEMA_EN` is now also used by the routine idle-release policy (Section 6.5). The two must remain distinct code paths: the e-stop disable is **latched** and `StepperDriver::enable()` must refuse to energise the coils while `ESTOP_TRIGGERED`; the idle release is **soft and reversible**.

### 5.2 Disconnect-mid-move behavior (confirmed: Option A)

* Connection health tracked via WebSocket ping/pong heartbeat (recommend 1s interval, 2 missed = disconnected).
* If disconnect is detected while `MOVING`: **do not abort** — let the current trajectory finish normally, then transition to `DISCONNECT_LOCKOUT` instead of `IDLE`.
* If disconnect is detected while already `IDLE`: transition directly to `DISCONNECT_LOCKOUT`.
* Recovery requires a fresh WebSocket connection **and** an explicit re-home before any move command is accepted.

### 5.3 Payload profile

Two profiles, `EMPTY` and `LOADED`, selected by the user via the web app before issuing a move (not mid-move).

* `EMPTY`: full derived velocity/accel/jerk limits (Section 6).
* `LOADED`: all velocity/accel/jerk limits derated by a configurable factor, default **50%**, defined as a single constant.
* Defaults to `EMPTY` on boot and after every re-home.
* This requires no load-sensing hardware — it's an operator-declared state, which is appropriate given the current BOM.

---

## 6. Motion Profiling

### 6.1 Per-joint limits

Derive conservative default max velocity, acceleration, and jerk for each joint from datasheet stall torque/no-load speed figures (NEMA17; YSIDO 35KG; RCXAZ 14.5KG; MG90s; SG90). Mark every derived constant clearly in `MotionConfig.h` with a comment citing the datasheet figure it was derived from and a note that these are conservative starting points meant to be tuned on the physical bench. Apply a safety derating factor (recommend starting at 50–60% of theoretical max) — do not use raw datasheet max figures.

**[REV 2] Base (θ0) limits through the 5:1 gearbox.** Limits are stored in **output-shaft** units and must be **re-derived, not reused** from the direct-drive numbers:

* Output velocity ceiling = motor-side velocity ceiling ÷ `STEPPER_GEAR_RATIO`. The same motor step rate now moves the base 5× slower than before.
* Output torque = motor torque × `STEPPER_GEAR_RATIO` × gearbox efficiency (assume ≈ 0.9, and note the assumption in a comment).
* With that torque margin, the binding limit on acceleration and jerk is now most likely arm inertia, cable drag and mechanical stability rather than motor torque. Start at the 50–60% derating, test with the arm at full extension, and tune on the bench.
* The step-rate consequence of any chosen output velocity is checked at compile time (Section 6.4).
* Gearbox backlash is typically a fraction of a degree to about a degree at the output (check the datasheet). It does not affect the planner. Note in `StepperDriver` that homing and final approach should always arrive from the same rotation direction if repeatable positioning matters.

### 6.2 Time synchronization

1. For a given move, compute the **minimum time-optimal duration** for *each* joint independently under its own S-curve limits (respecting the active payload profile's derating).
2. Take the **maximum across all joints** as the single shared move duration.
3. Re-scale every joint's S-curve to that shared duration (time-scaling a jerk-limited profile preserves its shape while stretching duration — implement this as a normalized-time parameterization: a single progress variable s ∈ [0,1] driven by the shared S-curve shape, mapped to each joint's absolute angle range).
4. This guarantees all joints start and stop together regardless of how far each individually has to travel — the whole point of the synchronization step.

### 6.3 Execution constraints

* No `delay()` anywhere in motion code. Waits (e.g. driver settle time) are implemented as non-blocking timestamps.
* Trajectory evaluation is driven by a **hardware timer** running at a fixed control-loop rate (1 kHz).
* **Servo output [REV 2]:** servo angle updates run at **50 Hz**, matching the 50 Hz PWM frame (a servo only sees a new pulse width once per 20 ms frame, so updating faster is pointless). Pulses are generated by the ESP32 **LEDC hardware**, so pulse width is jitter-free regardless of WiFi, FreeRTOS scheduling or the control loop.
* **Servo driver [REV 2]:** `ServoDriver` wraps the LEDC peripheral directly: 50 Hz, 14-bit resolution (the S3 maximum at 50 Hz), giving about 1.2 µs of pulse-width resolution (finer than the PCA9685's 12-bit). Do **not** use a bit-banged or `delay()`-based servo library. Either the `ESP32Servo` library or direct LEDC calls is acceptable. Direct LEDC is preferred (`ledcAttach`/`ledcWrite` on Arduino-ESP32 core 3.x; `ledcSetup`/`ledcAttachPin`/`ledcWrite` on core 2.x). Pin the core version in `platformio.ini` and hide the API difference inside `ServoDriver`. Conversion chain: angle (deg) → pulse width (µs, per-servo calibration from `ServoConfig.h`) → duty = `round(pulse_us × 2^14 / 20000)`.
* **Stepper pulse generation:** implement a **custom lightweight timer-ISR step generator** rather than pulling in AccelStepper/FastAccelStepper — those libraries default to trapezoidal or approximate S-curve profiles and won't cleanly take an externally-computed jerk-limited position/velocity target per tick. The ISR's job is simply: given the current commanded step position from the trajectory module, emit STEP/DIR pulses to converge on it. **[REV 2]** The step ISR must run **faster than the 1 kHz trajectory tick** and be sized against the gearbox step rate (Section 6.4). The ISR must be `IRAM_ATTR`, must not call flash-resident or blocking code, and must not log.
* **[REV 2]** Servo driver library choice no longer depends on I²C. Remove the Adafruit PCA9685 and `Wire` dependencies from `platformio.ini` unless another I²C device is added later.

### 6.4 Stepper gearing and microstepping **[REV 2]**

All constants live in the stepper block of `MotionConfig.h`:

```cpp
constexpr float    STEPPER_FULL_STEPS_PER_REV   = 200.0f;  // 1.8° NEMA17
constexpr float    STEPPER_GEAR_RATIO           = 5.0f;    // MEASURE on the bench (§12 step 3d); many "5:1" units are 5.18:1
constexpr uint8_t  STEPPER_MICROSTEP_DIV        = 16;      // one of 1, 2, 4, 8, 16, 32
constexpr float    STEPPER_STEPS_PER_OUTPUT_DEG =
    (STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEP_DIV * STEPPER_GEAR_RATIO) / 360.0f;  // 44.444 at 1/16, 5:1
constexpr uint32_t STEPPER_ISR_HZ               = 20000;   // step-generator ISR rate
```

**Conversion rules (all inside `StepperDriver`):**

* Target steps = `lroundf(angle_deg × STEPPER_STEPS_PER_OUTPUT_DEG)`, always computed **absolutely from the home position** as an `int32_t`. Never accumulate per-tick rounded deltas, because rounding errors would drift.
* The `DIR` polarity is unchanged by a planetary gearbox (sun-in, carrier-out turns the same direction). Keep a `STEPPER_DIR_INVERT` config flag regardless.
* Microstep pins M0/M1/M2 are set by `StepperDriver::begin()` from `STEPPER_MICROSTEP_DIV` using the DRV8825 truth table below, **while EN is HIGH (driver disabled)**. Do not change the microstep mode while the coils are energised.

| M0 | M1 | M2 | Microstep |
|-|-|-|-|
| 0 | 0 | 0 | 1 (full) |
| 1 | 0 | 0 | 1/2 |
| 0 | 1 | 0 | 1/4 |
| 1 | 1 | 0 | 1/8 |
| 0 | 0 | 1 | 1/16 |
| 1 | 0 | 1 | 1/32 |
| 0 | 1 | 1 | 1/32 |
| 1 | 1 | 1 | 1/32 |

**Does gearing require a different microstep setting? Not required. Resolution is not the concern.** With the gearbox, resolution multiplies by 5 at any microstep setting. What the gearbox changes is the **step rate needed for a given output speed** (5× higher), which loads the step ISR, and which makes high microstep values unnecessary:

| Microstep | Steps per output rev | Output resolution | Step rate at 90°/s output |
|-|-|-|-|
| 1/4 | 4,000 | 0.090° | 1,000 /s |
| 1/8 | 8,000 | 0.045° | 2,000 /s |
| **1/16 (default)** | 16,000 | 0.0225° | 4,000 /s |
| 1/32 | 32,000 | 0.01125° | 8,000 /s |

At the 35 cm full reach, 0.0225° at the base is about 0.14 mm of tip movement. The servos' own resolution and deadband are coarser than that, so going above 1/16 adds step-rate load and no useful accuracy. DRV8825 is not an interpolating driver, and at high microstep values the incremental holding torque per microstep falls, though the gearbox gives plenty of margin. **Default 1/16; 1/8 is the fallback if the ISR budget or smoothness testing calls for it.** Change only `STEPPER_MICROSTEP_DIV`.

**Step-rate budget (compile-time check).** With the usual raise-STEP-then-lower-STEP-on-the-next-tick scheme, the maximum step rate is `STEPPER_ISR_HZ / 2`; keep the peak requirement under half of that. Enforce it in `MotionConfig.h`:

```cpp
static_assert(BASE_MAX_VEL_OUT_DPS * STEPPER_STEPS_PER_OUTPUT_DEG * 4.0f <= STEPPER_ISR_HZ,
              "Step ISR too slow for base max velocity at this microstep/gear ratio");
```

At the defaults (20 kHz ISR, 1/16, 5:1) this allows up to ≈112 °/s at the output. The DRV8825 itself needs only ≥1.9 µs STEP high and low times, so the ISR rate, not the driver, is the limit. **Warning:** if the Revision 1 stepper code emits at most one pulse per 1 kHz tick, the base would be silently capped at about 22 °/s at 1/16 with this gearbox, and the planner's durations would no longer match reality. Verify this explicitly (Section 12, step 3c).

### 6.5 Stepper coil power management **[REV 2]**

**Rule.** The DRV8825 is **enabled (EN low) whenever the base must hold or move precisely**, and **released (EN high) after the base has been idle for a short delay**, which covers pick/drop dwell while the gripper works. The DRV8825 has no hold-current reduction, so enable/disable is the only way to reduce standstill heat. Note that EN does not change the current level; the current while enabled is whatever VREF sets.

**Constants (`MotionConfig.h`):**

```cpp
constexpr bool     STEPPER_IDLE_RELEASE_ENABLED  = true;   // master switch; set false to keep coils always energised
constexpr uint32_t STEPPER_IDLE_RELEASE_DELAY_MS = 500;    // time after the last step before releasing coils
constexpr uint32_t STEPPER_ENABLE_SETTLE_MS      = 5;      // wait after EN low before the first STEP; confirm against the DRV8825 datasheet
```

**Behavior requirements:**

1. **Enable before motion.** Any command that needs the base to move (a trajectory, a homing step) first calls `StepperDriver::enable()`. This drives EN low and starts a **non-blocking** settle timer. `StepperDriver::isReady()` returns true only after `STEPPER_ENABLE_SETTLE_MS` has elapsed. No STEP pulses are emitted before then, and the trajectory time base (`elapsed_ms`) starts only once `isReady()`.
2. **Release only at true rest.** `StepperDriver::release()` may be called only when **all** hold: no active trajectory, commanded velocity is zero, `STEPPER_IDLE_RELEASE_DELAY_MS` has elapsed since the last emitted step, and the state is `IDLE` or `DISCONNECT_LOCKOUT`. Never release during `HOMING` or `MOVING`.
3. **Position bookkeeping survives release.** The step counter, DIR level and microstep mode are preserved. Any step request while released first triggers `enable()` plus settle.
4. **Safety gating.** `enable()` must consult `SafetyMonitor` and refuse in `ESTOP_TRIGGERED`. The E-stop path (Section 5.1) and the idle-release path both drive the same pin but are distinct code paths, and the log line must say which one acted.
5. **Boot behavior.** `setup()` drives EN HIGH before any other stepper initialisation (the external pull-up in 2.6 covers the period before firmware runs).
6. **No new hardware assumptions.** Releasing is a software policy; it must be fully suppressed by `STEPPER_IDLE_RELEASE_ENABLED = false` with no other behavior change. In dry-run mode (Section 9) the enable/release calls are logged, not performed.
7. **Optional telemetry (additive, backward-compatible).** `state_update` may carry `"base_coils": "energized" | "released"`. The web app must ignore unknown fields, so this can be added without breaking Revision 1 clients. Do not otherwise change the schema.

**Why this is acceptable, and its limits (for the code comments):**

* The base twist axis is **vertical**, so there is essentially no gravity torque on it at rest; this is what makes releasing the coils safe here. This reasoning would **not** hold for an axis that carries load against gravity.
* Releasing the coils leaves only motor detent torque plus gearbox friction holding the base. A 5:1 planetary gearbox is back-drivable, so a large bump or cable tension could rotate it. The motor is open-loop, so a shift of more than about one electrical cycle (4 full steps) could be re-captured at the wrong rotor position without any error being reported. Servo reaction forces from a gripper opening or closing are tiny, but it is why the drift test in Section 12 (step 4d) is required before relying on this feature.
* Re-energising snaps the rotor back to the commanded electrical position. Expect at most a small twitch (well under a degree at the output).
* The thermal saving is proportional to the fraction of time released. It is a complement to, not a replacement for, the correct VREF and cooling in Section 2.6. Short releases between moves are averaged out by the driver's thermal mass, so do not expect a large temperature drop from sub-second pauses.
* Optional future upgrade (not part of this revision): a driver with standstill current reduction (e.g. TMC2209) removes the need for this policy.

---

## 7. Communication Architecture

### 7.1 Transport split

* **WebSocket**: move commands, live IK step-by-step derivation + geometry for the visualizer, trajectory progress streaming, state/telemetry push, e-stop/clear commands, heartbeat.
* **HTTP**: `GET /api/limits` (joint ranges, current max reach envelope, current payload profile), `GET /api/status` (state, homed flag, last error) — occasional/poll-style data that doesn't need push semantics.
* Static web app assets served from ESP32 (LittleFS) via `GET /`.

### 7.2 WiFi

* **Station mode**, using **WiFiManager** for credential provisioning (captive portal on first boot / if saved credentials fail) rather than hardcoded SSID/password. This avoids re-flashing to change networks — important for a project you'll demo in different locations.

### 7.3 JSON message schema

All messages use a `type` envelope field. Design and implement exactly this shape:

**Client → Server (WebSocket):**

```json
{"type": "move_command", "x": 20.0, "y": 5.0, "z": 15.0, "phi": -90.0, "posture": "elbow_down", "gripper": "closed"}
{"type": "home_request"}
{"type": "stop_request"}
{"type": "clear_estop"}
{"type": "set_payload_profile", "profile": "loaded"}
{"type": "ping"}
```

**Server → Client (WebSocket):**

```json
{"type": "ik_solution", "input": {"x":20.0,"y":5.0,"z":15.0,"phi":-90.0},
 "solutions": {
   "elbow_up":   {"theta0":0.0,"theta1":0.0,"theta2":0.0,"theta3":0.0,"valid":true,  "error": null},
   "elbow_down": {"theta0":0.0,"theta1":0.0,"theta2":0.0,"theta3":0.0,"valid":false, "error":"JOINT_LIMIT_EXCEEDED"}
 },
 "steps": [
   {"label":"base_twist","expression":"θ0 = atan2(y,x)","value":0.245},
   {"label":"planar_reduction","expression":"r = sqrt(x²+y²)","value":20.6}
   /* ...one entry per derivation step, in order, for the educational panel */
 ]}

{"type":"trajectory_progress","progress":0.42,"joint_angles":{"theta0":0.0,"theta1":0.0,"theta2":0.0,"theta3":0.0},"elapsed_ms":210,"duration_ms":500}

{"type":"state_update","state":"IDLE","homed":true,"payload_profile":"empty","gripper":"open"}

{"type":"error","code":"TARGET_UNREACHABLE","message":"..."}

{"type":"pong"}
```

(The numeric values in the `ik_solution` and `trajectory_progress` examples are placeholders.)

**HTTP:**

```
GET /api/limits  → { "reach": {...}, "joint_limits": {...}, "payload_profile": "empty" }
GET /api/status  → { "state": "IDLE", "homed": true, "last_error": null }
```

Implement corresponding JSON serialization/parsing in one dedicated module (`MessageSchema`) — do not scatter ad-hoc JSON construction across multiple files. Joint angles in all messages are output-joint angles (for θ0, the gearbox output).

---

## 8. Web Application

* Plain HTML/CSS/JS (no heavy framework) — keeps it small enough to live in ESP32 LittleFS for a fully self-contained demo (no laptop-hosted backend required; browser connects straight to the ESP32's IP).
* **Canvas-based 2D visualizer**: draw the arm's side-view plane (post-base-rotation) showing L1/L2/L3 links, the target point, and both elbow-up and elbow-down candidate postures overlaid (highlight the invalid one if a posture fails limits) — driven directly by the `ik_solution` message.
* **Step-by-step math panel**: renders the `steps` array from `ik_solution` in order, each with its label, symbolic expression, and computed value — this is the "educational" requirement, keep it as a simple scrollable list, no need for anything fancier.
* Controls: numeric x/y/z/φ inputs, posture selector (enabled only for postures marked valid), gripper toggle, payload profile toggle, Home button, Stop button, and a prominent connection/state indicator.
* Max-reach values from `/api/limits` displayed persistently, not just on error.
* **[REV 2]** No functional change is required in the web app for Revision 2. If the optional `base_coils` field (6.5) is implemented, the UI may show it as a small status indicator, and must tolerate its absence.

---

## 9. Logging & Debug Support

* Leveled logging macros (`LOG_DEBUG/INFO/WARN/ERROR`) over Serial, compiled out entirely in release builds via a PlatformIO build flag.
* **Dry-run/simulation mode**: a compile-time or runtime flag that runs the full state machine, IK, and trajectory math on every command but skips real GPIO and LEDC writes, logging what *would* have been sent instead. This lets you validate logic on the bench without the arm connected — strongly recommended given how math-heavy this system is. **[REV 2]** In dry-run mode, `ServoDriver` logs the pulse width in µs and the LEDC duty it would have written, and `StepperDriver` logs the target step count (using the gear ratio and microstep constants) and any enable/release action.

---

## 10. Suggested Firmware File Structure **[REV 2]**

```
src/
  main.cpp
config/
  Pins.h                 // all GPIO assignments, one source of truth (REV 2: servo PWM pins added, I2C pins removed)
  JointLimits.h          // ranges, home angles, soft limits, link lengths (L1/L2/L3/d1); all in OUTPUT-shaft degrees
  MotionConfig.h         // velocity/accel/jerk per joint, payload derating factor,
                         // REV 2: stepper gear ratio / microstep / ISR rate / coil-release constants
  ServoConfig.h          // REV 2 (new): per-servo pulse min/max (µs), angular range, invert flag, trim; LEDC freq/resolution.
                         //   Replaces any PCA9685-specific constants (I2C address, tick counts) wherever they currently live.
kinematics/
  Kinematics.h/.cpp        // pure IK + FK + reachability, no hardware deps
motion/
  SCurveProfile.h/.cpp     // jerk-limited normalized-time profile generator
  TrajectoryPlanner.h/.cpp // per-joint duration calc + synchronization
  StepperDriver.h/.cpp     // timer ISR step/dir pulse generation; REV 2: owns deg<->step conversion, microstep pins, enable()/release()/isReady()
  ServoDriver.h/.cpp       // REV 2: ESP32 LEDC PWM servo driver (replaces PCA9685 wrapper); public interface unchanged
control/
  StateMachine.h/.cpp
  HomingSequence.h/.cpp
  SafetyMonitor.h/.cpp     // e-stop ISR, disconnect-lockout logic, watchdog; REV 2: gates StepperDriver::enable()
comms/
  WiFiSetup.h/.cpp         // WiFiManager integration
  WebServerManager.h/.cpp  // HTTP routes + static file serving
  WebSocketHandler.h/.cpp  // WS message dispatch
  MessageSchema.h/.cpp     // JSON (de)serialization, single source of truth
tasks/
  NetworkTask.h/.cpp       // FreeRTOS task: comms + command parsing + IK dispatch
  MotionTask.h/.cpp        // FreeRTOS task: fixed-rate control loop; REV 2: idle-release policy
utils/
  Logger.h/.cpp
  SimMode.h
data/                       // LittleFS contents: index.html, style.css, app.js, visualizer.js
```

**`ServoDriver` public interface contract (keep stable across Revision 2).** If the existing wrapper already exposes these operations under different names, keep the existing names; only the internals change:

```cpp
enum class ServoId { SHOULDER, ELBOW, WRIST, GRIPPER };
bool ServoDriver::begin();                         // attach LEDC channels with the HOME pulse already as the initial duty
void ServoDriver::writeAngleDeg(ServoId, float);   // clamp to soft limits → µs (ServoConfig) → LEDC duty
void ServoDriver::freeze();                        // e-stop: stop updating; LEDC keeps emitting the last duty
```

Command flow: `NetworkTask` receives a command → runs IK inline (cheap, analytical) → on valid + user-confirmed posture, builds a trajectory descriptor and pushes it to `MotionTask` via a FreeRTOS queue → `MotionTask` executes it against its fixed-rate timer loop, pushing `trajectory_progress` updates back out.

---

## 11. Build Order (do not skip ahead)

1. **Scaffolding**: PlatformIO project, `Pins.h`, `JointLimits.h`, `MotionConfig.h`, `Logger`. Nothing functional yet — just structure and constants.
2. **Kinematics module**, fully unit-testable in isolation (write a few known-input/expected-output cases). Confirm this works before touching hardware code at all.
3. **Motion profiling** (`SCurveProfile`, `TrajectoryPlanner`) — testable on desktop with printed/plotted output, no hardware needed.
4. **Hardware drivers**: `StepperDriver` (timer ISR), `ServoDriver` (**[REV 2]** LEDC PWM). Bench-test each independently (e.g., command a known angle/step count and verify by eye) before integrating.
5. **State machine + safety**: `StateMachine`, `HomingSequence`, `SafetyMonitor` (e-stop ISR + disconnect lockout).
6. **Networking**: `WiFiSetup`, `WebServerManager`, `WebSocketHandler`, `MessageSchema`.
7. **Task wiring**: `NetworkTask` + `MotionTask` + the queue between them.
8. **Web app**: static UI, canvas visualizer, step-by-step panel — build and test against the now-working firmware API.
9. **Integration pass**: full end-to-end run, then tune the Section 6.1 motion constants on the physical bench.

At each milestone, pause and confirm the module works in isolation before moving to the next — do not chain unverified modules together.

---

## 12. Revision 2 Migration Instructions **[REV 2]**

The code base already exists. Apply the changes in this order, confirming each step before the next. Keep commits small (one per step) so any step can be reverted.

### Step 0 — Baseline
Tag the current code. Run all native unit tests and record that they pass. These tests must still pass, unmodified, at the end.

### Step 1 — Servo swap (PCA9685 → LEDC)
a. `Pins.h`: add `SERVO_SHOULDER_PWM=4`, `SERVO_ELBOW_PWM=5`, `SERVO_WRIST_PWM=6`, `SERVO_GRIPPER_PWM=7`; remove `I2C_SDA`/`I2C_SCL`.
b. Create `ServoConfig.h` with LEDC frequency (50 Hz), resolution (14 bit), and per-servo `pulseMinUs`, `pulseMaxUs`, `angleRangeDeg`, `invert`, `trimUs`. Start with conservative pulses (about 600–2400 µs) and widen on the bench: driving a servo past its mechanical end stop causes buzzing and can burn the servo. Move any PCA9685-specific constants here or delete them.
c. Rewrite the **internals** of `ServoDriver` only; keep the public interface contract from Section 10. Initial duty at attach = home pulse.
d. `platformio.ini`: remove the PCA9685 library and `Wire` if unused; pin the Arduino-ESP32 core version.
e. Remove the I²C initialisation from `main.cpp`/setup code.
f. `SimMode`: update dry-run logging as in Section 9.
g. Add a **native** unit test for the pure conversion: angle → µs → duty. With 14-bit/50 Hz, 1500 µs → duty 1229 (1228.8 rounded); 500 µs → 410; 2500 µs → 2048.
h. Bench test (arm unpowered/disconnected from load): each servo to 0°, 90°, 180°; confirm no twitch at power-up and direction/range are correct. Test the large servos at the final supply voltage, with the supply bulk capacitor fitted.

### Step 2 — E-stop text/polarity check
Confirm the implemented e-stop polarity matches Section 2.3 (LOW = safe, HIGH = triggered). Fix the code or comments if they followed the old sentence.

### Step 3 — Gearbox and microstepping
a. `MotionConfig.h`: add the stepper block from Section 6.4. Re-derive base velocity/accel/jerk in **output** units (Section 6.1); do not carry over the old values.
b. `StepperDriver`: centralise `stepsFromDeg()` / `degFromSteps()` using `STEPPER_STEPS_PER_OUTPUT_DEG` (absolute, from home); set M0/M1/M2 from `STEPPER_MICROSTEP_DIV` in `begin()` while EN is HIGH. Add a native unit test: with defaults, 90° → 4000 steps, 360° → 16000 steps, and `degFromSteps(stepsFromDeg(x))` round-trips within one microstep.
c. **Step-rate audit.** Check how many pulses the existing step ISR can emit per second. If it is limited to one pulse per 1 kHz tick, replace it with a dedicated ISR at `STEPPER_ISR_HZ` that emits at most one step per tick. The 1 kHz trajectory update still sets the target. Add the `static_assert` from Section 6.4.
d. **Measure the real gear ratio.** Mark the gearbox output shaft, command a 360° base rotation (16,000 steps at the defaults), and measure the actual rotation `θ_actual`. Set `STEPPER_GEAR_RATIO_new = STEPPER_GEAR_RATIO_old × 360 / θ_actual`. A 5.18:1 unit entered as 5.0 would otherwise be off by about 3.6% (over 6° at the ±170° limit).
e. `HomingSequence`: express seek speed, back-off, and maximum search rotation in output degrees and °/s. Confirm homing still reaches the switch, the backoff behavior is unchanged, and the timeout produces `HOMING_FAILED` when the switch is unplugged.
f. Do the electrical setup in Section 2.6 (VREF, heatsink, EN pull-up, capacitor) before running the base under load.

### Step 4 — Coil power management (Section 6.5)
a. Add `enable()` / `release()` / `isReady()` to `StepperDriver`, using non-blocking timing. Add the three constants.
b. `SafetyMonitor` gates `enable()` (refuse in `ESTOP_TRIGGERED`).
c. `MotionTask`: enable before starting a trajectory or a homing move, release per the conditions in Section 6.5. `setup()` drives EN HIGH first.
d. **Drift test (required before leaving the feature on):** run 50 cycles of: move → idle past `STEPPER_IDLE_RELEASE_DELAY_MS` (coils released, gripper opens/closes) → move back. Then home and record the step offset at which the limit switch triggers versus the same test with `STEPPER_IDLE_RELEASE_ENABLED = false`. If the base visibly shifts or the homing offset differs by more than about 0.1° (output), set the flag to `false`, report the finding, and do not hide it.
e. Verify the E-stop still holds EN HIGH and that `clear_estop` + re-home is required to resume.

### Step 5 — Regression
All Step 0 tests pass without edits. Full end-to-end run from the web app.

### Files that must show **no functional diff**
`Kinematics.*`, `SCurveProfile.*`, `TrajectoryPlanner.*` (config values it reads may change), `MessageSchema.*` (except the optional `base_coils` field), everything under `data/`, `JointLimits.h` (except comments), `StateMachine.*`. A diff in any of these means the hardware change has leaked above the driver layer and should be reworked.
