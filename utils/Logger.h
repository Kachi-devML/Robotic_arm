#pragma once

#include <stdarg.h>
#include <stdint.h>

// Leveled Serial logging. In release (LOG_LEVEL=0) the macros expand to
// nothing — no format evaluation, no Serial I/O.
//
// Levels: 0 off, 1 error, 2 warn, 3 info, 4 debug.
#ifndef LOG_LEVEL
#define LOG_LEVEL 3
#endif

class Logger {
 public:
  enum Level : uint8_t {
    kError = 1,
    kWarn = 2,
    kInfo = 3,
    kDebug = 4,
  };

  static void begin(uint32_t baud);
  static void write(Level level, const char* fmt, ...);

 private:
  static const char* prefix(Level level);
};

#if LOG_LEVEL >= 4
#define LOG_DEBUG(fmt, ...) ::Logger::write(::Logger::kDebug, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(...) ((void)0)
#endif

#if LOG_LEVEL >= 3
#define LOG_INFO(fmt, ...) ::Logger::write(::Logger::kInfo, fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(...) ((void)0)
#endif

#if LOG_LEVEL >= 2
#define LOG_WARN(fmt, ...) ::Logger::write(::Logger::kWarn, fmt, ##__VA_ARGS__)
#else
#define LOG_WARN(...) ((void)0)
#endif

#if LOG_LEVEL >= 1
#define LOG_ERROR(fmt, ...) ::Logger::write(::Logger::kError, fmt, ##__VA_ARGS__)
#else
#define LOG_ERROR(...) ((void)0)
#endif
