#include "WebServerManager.h"

#include <LittleFS.h>

#include "JointLimits.h"
#include "MessageSchema.h"

namespace Comms {

WebServerManager::WebServerManager() : server_(80) {}

WebServerManager::~WebServerManager() = default;

void WebServerManager::begin() {
    if (!LittleFS.begin(true)) {
        return;
    }

    server_.on("/", HTTP_GET, [this]() { handleRoot(); });
    server_.on("/api/limits", HTTP_GET, [this]() { handleApiLimits(); });
    server_.on("/api/status", HTTP_GET, [this]() { handleApiStatus(); });
    server_.onNotFound([this]() { handleNotFound(); });
    server_.begin();
}

void WebServerManager::handleClient() {
    server_.handleClient();
}

void WebServerManager::setStatusPayload(const String& state, bool homed) {
    state_ = state;
    homed_ = homed;
}

void WebServerManager::setLastError(const String& lastError) {
    lastError_ = lastError;
}

void WebServerManager::setLimitsPayload(const String& payloadProfile,
                                       float minReach,
                                       float maxReach) {
    payloadProfile_ = payloadProfile;
    minReach_ = minReach;
    maxReach_ = maxReach;
}

void WebServerManager::handleRoot() {
    if (LittleFS.exists("/index.html")) {
        File file = LittleFS.open("/index.html", "r");
        if (file) {
            server_.streamFile(file, "text/html");
            file.close();
            return;
        }
    }

    server_.send(
        503, "text/plain",
        "Web UI assets are missing. Upload them with: pio run -t uploadfs -e esp32s3");
}

void WebServerManager::handleApiLimits() {
    const String payload = buildLimitsResponse(payloadProfile_.c_str(),
                                              minReach_,
                                              maxReach_,
                                              -170.0f,
                                              170.0f,
                                              JointLimits::kL1_cm,
                                              JointLimits::kL2_cm,
                                              JointLimits::kL3_cm,
                                              JointLimits::kD1_cm);
    server_.send(200, "application/json", payload);
}

void WebServerManager::handleApiStatus() {
    const String payload = buildStatusResponse(state_.c_str(), homed_, lastError_.isEmpty() ? nullptr : lastError_.c_str());
    server_.send(200, "application/json", payload);
}

void WebServerManager::handleNotFound() {
    if (LittleFS.exists(server_.uri())) {
        File file = LittleFS.open(server_.uri(), "r");
        if (file) {
            const String path = server_.uri();
            if (path.endsWith(".html")) {
                server_.streamFile(file, "text/html");
            } else if (path.endsWith(".css")) {
                server_.streamFile(file, "text/css");
            } else if (path.endsWith(".js")) {
                server_.streamFile(file, "application/javascript");
            } else {
                server_.streamFile(file, "application/octet-stream");
            }
            file.close();
            return;
        }
    }

    server_.send(404, "text/plain", "Not found");
}

}  // namespace Comms
