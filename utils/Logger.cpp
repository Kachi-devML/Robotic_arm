#include "Logger.h"

#include <Arduino.h>
#include <stdio.h>

void Logger::begin(uint32_t baud) {
#if LOG_LEVEL >= 1
  Serial.begin(baud);
#else
  (void)baud;
#endif
}

const char* Logger::prefix(Level level) {
#if LOG_LEVEL < 1
  (void)level;
  return "";
#else
  switch (level) {
    case kError:
      return "E";
    case kWarn:
      return "W";
    case kInfo:
      return "I";
    case kDebug:
      return "D";
    default:
      return "?";
  }
#endif
}

void Logger::write(Level level, const char* fmt, ...) {
#if LOG_LEVEL >= 1
  if (static_cast<int>(level) > LOG_LEVEL) {
    return;
  }
  Serial.printf("[%s] ", prefix(level));
  va_list args;
  va_start(args, fmt);
  char buf[192];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
#else
  (void)level;
  (void)fmt;
#endif
}
