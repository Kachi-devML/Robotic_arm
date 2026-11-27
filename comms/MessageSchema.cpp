#include "MessageSchema.h"

namespace Comms {

namespace {

bool parseCartesianCommand(const String& message,
                           const char* expectedType,
                           MoveCommand& out) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, message) ||
        !doc["type"].is<const char*>() ||
        String(doc["type"].as<const char*>()) != expectedType) {
        return false;
    }

    const char* fields[] = {"x", "y", "z", "phi"};
    for (const char* field : fields) {
        if (!doc[field].is<float>() && !doc[field].is<int>()) {
            return false;
        }
    }

    out = MoveCommand{};
    out.valid = true;
    out.x = doc["x"].as<float>();
    out.y = doc["y"].as<float>();
    out.z = doc["z"].as<float>();
    out.phi = doc["phi"].as<float>();

    if (doc["posture"].is<const char*>()) {
        out.posture = doc["posture"].as<const char*>();
    }
    if (doc["gripper"].is<const char*>()) {
        out.gripper = doc["gripper"].as<const char*>();
    }
    return true;
}

}  // namespace

bool parseMoveCommand(const String& message, MoveCommand& out) {
    return parseCartesianCommand(message, "move_command", out);
}

bool parseIkPreview(const String& message, MoveCommand& out) {
    return parseCartesianCommand(message, "ik_preview", out);
}

bool parseSetPayloadProfile(const String& message, String& profile) {
    StaticJsonDocument<128> doc;
    const DeserializationError err = deserializeJson(doc, message);
    if (err) {
        return false;
    }
    if (!doc.containsKey("type") ||
        String(doc["type"].as<const char*>()) != "set_payload_profile") {
        return false;
    }
    if (!doc.containsKey("profile")) {
        return false;
    }
    profile = doc["profile"].as<const char*>();
    return true;
}

String buildStateUpdate(const char* state,
                        const char* payloadProfile,
                        const char* gripper) {
    StaticJsonDocument<256> doc;
    doc["type"] = "state_update";
    doc["state"] = state;
    doc["payload_profile"] = payloadProfile;
    doc["gripper"] = gripper;

    String output;
    serializeJson(doc, output);
    return output;
}

String buildError(const char* code, const char* message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "error";
    doc["code"] = code;
    doc["message"] = message;

    String output;
    serializeJson(doc, output);
    return output;
}

String buildPong() {
    StaticJsonDocument<64> doc;
    doc["type"] = "pong";

    String output;
    serializeJson(doc, output);
    return output;
}

String buildLimitsResponse(const char* payloadProfile,
                          float minReach,
                          float maxReach,
                          float theta0MinDeg,
                          float theta0MaxDeg,
                          float link1,
                          float link2,
                          float link3,
                          float shoulderHeight) {
    StaticJsonDocument<256> doc;
    JsonObject reach = doc.createNestedObject("reach");
    reach["min"] = minReach;
    reach["max"] = maxReach;

    JsonObject jointLimits = doc.createNestedObject("joint_limits");
    jointLimits["theta0_min_deg"] = theta0MinDeg;
    jointLimits["theta0_max_deg"] = theta0MaxDeg;
    jointLimits["theta1_min_deg"] = 0.0f;
    jointLimits["theta1_max_deg"] = 180.0f;
    jointLimits["theta2_min_deg"] = 0.0f;
    jointLimits["theta2_max_deg"] = 180.0f;
    jointLimits["theta3_min_deg"] = 0.0f;
    jointLimits["theta3_max_deg"] = 180.0f;

    JsonObject links = doc.createNestedObject("links");
    links["l1_cm"] = link1;
    links["l2_cm"] = link2;
    links["l3_cm"] = link3;
    links["shoulder_height_cm"] = shoulderHeight;

    doc["payload_profile"] = payloadProfile;

    String output;
    serializeJson(doc, output);
    return output;
}

String buildStatusResponse(const char* state,
                          const char* lastError) {
    StaticJsonDocument<128> doc;
    doc["state"] = state;
    if (lastError != nullptr) {
        doc["last_error"] = lastError;
    } else {
        doc["last_error"] = nullptr;
    }

    String output;
    serializeJson(doc, output);
    return output;
}

String buildTrajectoryProgress(float progress,
                              float theta0,
                              float theta1,
                              float theta2,
                              float theta3,
                              uint32_t elapsedMs,
                              uint32_t durationMs) {
    StaticJsonDocument<256> doc;
    doc["type"] = "trajectory_progress";
    doc["progress"] = progress;
    JsonObject jointAngles = doc.createNestedObject("joint_angles");
    jointAngles["theta0"] = theta0;
    jointAngles["theta1"] = theta1;
    jointAngles["theta2"] = theta2;
    jointAngles["theta3"] = theta3;
    doc["elapsed_ms"] = elapsedMs;
    doc["duration_ms"] = durationMs;

    String output;
    serializeJson(doc, output);
    return output;
}

String buildIkSolution(float x,
                      float y,
                      float z,
                      float phi,
                      float theta0Up,
                      float theta1Up,
                      float theta2Up,
                      float theta3Up,
                      bool elbowUpValid,
                      const char* elbowUpError,
                      float theta0Down,
                      float theta1Down,
                      float theta2Down,
                      float theta3Down,
                      bool elbowDownValid,
                      const char* elbowDownError,
                      const Kinematics::DerivationStep* derivationSteps,
                      uint8_t stepCount) {
    JsonDocument doc;
    JsonObject input = doc.createNestedObject("input");
    input["x"] = x;
    input["y"] = y;
    input["z"] = z;
    input["phi"] = phi;

    JsonObject solutions = doc.createNestedObject("solutions");
    JsonObject elbowUp = solutions.createNestedObject("elbow_up");
    elbowUp["theta0"] = theta0Up;
    elbowUp["theta1"] = theta1Up;
    elbowUp["theta2"] = theta2Up;
    elbowUp["theta3"] = theta3Up;
    elbowUp["valid"] = elbowUpValid;
    if (elbowUpError != nullptr) {
        elbowUp["error"] = elbowUpError;
    } else {
        elbowUp["error"] = nullptr;
    }

    JsonObject elbowDown = solutions.createNestedObject("elbow_down");
    elbowDown["theta0"] = theta0Down;
    elbowDown["theta1"] = theta1Down;
    elbowDown["theta2"] = theta2Down;
    elbowDown["theta3"] = theta3Down;
    elbowDown["valid"] = elbowDownValid;
    if (elbowDownError != nullptr) {
        elbowDown["error"] = elbowDownError;
    } else {
        elbowDown["error"] = nullptr;
    }

    JsonArray steps = doc.createNestedArray("steps");
    for (uint8_t i = 0; derivationSteps != nullptr && i < stepCount; ++i) {
        JsonObject step = steps.createNestedObject();
        step["label"] = derivationSteps[i].label;
        step["expression"] = derivationSteps[i].expression;
        step["value"] = derivationSteps[i].value;
    }

    doc["type"] = "ik_solution";

    String output;
    serializeJson(doc, output);
    return output;
}

}  // namespace Comms
