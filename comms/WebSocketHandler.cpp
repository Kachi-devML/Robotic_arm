#include "WebSocketHandler.h"

#include "MessageSchema.h"

namespace Comms {

WebSocketHandler* WebSocketHandler::instance_ = nullptr;

WebSocketHandler::WebSocketHandler() : ws_(81) {
    instance_ = this;
}

WebSocketHandler::~WebSocketHandler() = default;

void WebSocketHandler::begin() {
    ws_.begin();
    ws_.onEvent(WebSocketHandler::onEvent);
}

void WebSocketHandler::handleClient() {
    ws_.loop();
}

void WebSocketHandler::setMessageHandler(MessageHandler handler) {
    onMessage_ = handler;
}

void WebSocketHandler::broadcast(const String& payload) {
    String copy = payload;
    ws_.broadcastTXT(copy);
}

bool WebSocketHandler::connected() const {
    return ws_.connectedClients() > 0;
}

void WebSocketHandler::sendStateUpdate(const char* state,
                                       bool homed,
                                       const char* payloadProfile,
                                       const char* gripper) {
    broadcast(buildStateUpdate(state, homed, payloadProfile, gripper));
}

void WebSocketHandler::sendError(const char* code, const char* message) {
    broadcast(buildError(code, message));
}

void WebSocketHandler::sendPong() {
    broadcast(buildPong());
}

void WebSocketHandler::sendTrajectoryProgress(float progress,
                                            float theta0,
                                            float theta1,
                                            float theta2,
                                            float theta3,
                                            uint32_t elapsedMs,
                                            uint32_t durationMs) {
    broadcast(buildTrajectoryProgress(progress, theta0, theta1, theta2, theta3,
                                      elapsedMs, durationMs));
}

void WebSocketHandler::onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (instance_ == nullptr) {
        return;
    }

    switch (type) {
        case WStype_CONNECTED: {
            break;
        }
        case WStype_DISCONNECTED: {
            break;
        }
        case WStype_TEXT: {
            if (payload == nullptr || length == 0) {
                break;
            }
            String message(reinterpret_cast<const char*>(payload), length);
            if (instance_->onMessage_ != nullptr) {
                instance_->onMessage_(message);
            }
            break;
        }
        default:
            break;
    }
}

}  // namespace Comms
