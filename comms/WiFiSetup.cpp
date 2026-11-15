#include "WiFiSetup.h"

#include <WiFi.h>
#include <WiFiManager.h>

namespace Comms {

void WiFiSetup::begin(const char* apName, const char* apPassword) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFiManager manager;
    manager.setConfigPortalTimeout(180);
    manager.setDebugOutput(true);          // was false: prints WiFiManager's connection log
    manager.setAPCallback(nullptr);

    const bool connected = manager.autoConnect(apName, apPassword);

    Serial.printf("WiFi connected: %d, status: %d, SSID: %s, IP: %s\n",
                  connected, WiFi.status(), WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());
}

bool WiFiSetup::connect() {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiSetup::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String WiFiSetup::localIP() {
    return WiFi.localIP().toString();
}

void WiFiSetup::resetSettings() {
    WiFiManager manager;
    manager.resetSettings();
}

}  // namespace Comms
