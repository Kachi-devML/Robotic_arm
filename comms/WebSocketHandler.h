#pragma once

#include <Arduino.h>
#include <WebSocketsServer.h>

namespace Comms {

using MessageHandler = void (*)(const String&);

class WebSocketHandler {
 public:
    WebSocketHandler();
    ~WebSocketHandler();

    void begin();
    void handleClient();
    void setMessageHandler(MessageHandler handler);
    void broadcast(const String& payload);
    bool connected() const;

    void sendStateUpdate(const char* state,
                         bool homed,
                         const char* payloadProfile,
                         const char* gripper);
    void sendError(const char* code, const char* message);
    void sendPong();
    void sendTrajectoryProgress(float progress,
                                float theta0,
                                float theta1,
                                float theta2,
                                float theta3,
                                uint32_t elapsedMs,
                                uint32_t durationMs);

 private:
    static void onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    static WebSocketHandler* instance_;

    mutable WebSocketsServer ws_;
    MessageHandler onMessage_ = nullptr;
};

}  // namespace Comms
