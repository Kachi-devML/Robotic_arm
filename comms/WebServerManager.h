#pragma once

#include <Arduino.h>
#include <WebServer.h>

namespace Comms {

class WebServerManager {
 public:
    WebServerManager();
    ~WebServerManager();

    void begin();
    void handleClient();

    void setStatusPayload(const String& state);
    void setLastError(const String& lastError);
    void setLimitsPayload(const String& payloadProfile,
                          float minReach,
                          float maxReach);

 private:
    WebServer server_;
    String state_ = "BOOT";
    String lastError_ = "";
    String payloadProfile_ = "empty";
    float minReach_ = 0.0f;
    float maxReach_ = 0.0f;

    void handleRoot();
    void handleApiLimits();
    void handleApiStatus();
    void handleNotFound();
};

}  // namespace Comms
