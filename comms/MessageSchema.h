#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

#include "Kinematics.h"

namespace Comms {

struct MoveCommand {
    bool valid = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float phi = 0.0f;
    String posture = "elbow_down";
    String gripper = "open";
};

bool parseMoveCommand(const String& message, MoveCommand& out);
bool parseIkPreview(const String& message, MoveCommand& out);
bool parseSetPayloadProfile(const String& message, String& profile);

String buildStateUpdate(const char* state,
                        const char* payloadProfile,
                        const char* gripper);
String buildError(const char* code, const char* message);
String buildPong();
String buildLimitsResponse(const char* payloadProfile,
                          float minReach,
                          float maxReach,
                          float theta0MinDeg,
                          float theta0MaxDeg,
                          float link1,
                          float link2,
                          float link3,
                          float shoulderHeight);
String buildStatusResponse(const char* state,
                          const char* lastError);
String buildTrajectoryProgress(float progress,
                              float theta0,
                              float theta1,
                              float theta2,
                              float theta3,
                              uint32_t elapsedMs,
                              uint32_t durationMs);
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
                      const Kinematics::DerivationStep* steps,
                      uint8_t stepCount);

}  // namespace Comms
