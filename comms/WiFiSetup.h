#pragma once

#include <Arduino.h>

namespace Comms {

class WiFiSetup {
 public:
    static void begin(const char* apName = "RoboticArmAP",
                      const char* apPassword = "roboticarm");
    static bool connect();
    static bool isConnected();
    static String localIP();
    static void resetSettings();
};

}  // namespace Comms
