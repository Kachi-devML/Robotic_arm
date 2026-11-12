# 4-DoF Robotic Arm

## Wiring summary

- Base stepper (NEMA17 + 5:1 planetary gearbox) drives the DRV8825 on GPIO10/11/12/13/14/15. The driver ENABLE pin is active-low: EN low = coils energised, EN high = released. Add the required 10 kΩ pull-up on the EN line as noted in the build spec.
- Home switch: SS-5GL limit switch on GPIO16 with external 10 kΩ pull-up to 3.3 V. Triggered when the switch closes to GND (LOW).
- E-stop: normally-closed switch on GPIO17 to GND with external 10 kΩ pull-up to 3.3 V. The safe state is LOW; a broken or pressed circuit rises to HIGH and latches the software e-stop.
- Servo power: four servos are powered from an external supply, not the ESP32 5 V rail. The servo supply negative, ESP32 GND, and DRV8825 logic ground must be tied together at a single star point.
- Servo signals: shoulder on GPIO4, elbow on GPIO5, wrist on GPIO6, gripper on GPIO7. Controlled through the ESP32 LEDC hardware PWM at 50 Hz with 14-bit resolution.

The PCA9685 / I²C bus is intentionally not used for this revision.
