#pragma once

#include <stdint.h>

// Single source of truth for GPIO assignments (ESP32-S3).
//
// Target module: ESP32-S3-WROOM-1 / DevKitC-1 class. If your board is N8R8
// (octal PSRAM), do not move these nets onto GPIO33–37.
//
// Revision 2: the PCA9685 and I²C bus are removed. The four servos are now
// direct LEDC outputs on GPIO4–GPIO7; the base stepper remains on the DRV8825 pins.

namespace Pins {

constexpr uint8_t kStatusLed = 2;

constexpr uint8_t kNemaStep = 10;
constexpr uint8_t kNemaDir = 11;
constexpr uint8_t kNemaEn = 12;  // DRV8825 ENABLE is active-low on the board used here.
constexpr uint8_t kNemaM0 = 13;
constexpr uint8_t kNemaM1 = 14;
constexpr uint8_t kNemaM2 = 15;

// SS-5GL via 10 kΩ external pull-up to 3V3. Firmware must not enable the internal pull-up.
// Released = HIGH. Pressed (switch to GND) = LOW = home trigger.
constexpr uint8_t kHomeSw = 16;

// Hardware e-stop: N.C. to GND, 10 kΩ external pull-up to 3V3.
// LOW  = safe / idle (closed circuit)
// HIGH = triggered (switch pressed OR wire broken / open circuit)
// Do not invert this. SafetyMonitor services the rising edge with attachInterrupt.
constexpr uint8_t kEstopSw = 17;

// Direct ESP32-LEDC servo outputs.  GPIO4–7 are plain GPIOs, not strapping pins,
// not used by USB/UART0, and are suitable for 50 Hz servo PWM.
constexpr uint8_t kServoShoulderPwm = 4;
constexpr uint8_t kServoElbowPwm = 5;
constexpr uint8_t kServoWristPwm = 6;
constexpr uint8_t kServoGripperPwm = 7;

}  // namespace Pins
