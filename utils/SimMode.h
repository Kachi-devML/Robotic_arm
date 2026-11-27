#pragma once

// Dry-run / simulation mode (AGENT_BUILD_SPEC §9).
//
// When SIM_MODE is defined (or simModeEnabled() returns true at runtime),
// the full state machine, IK, and trajectory math run on every command
// but real GPIO/I2C writes are skipped, replaced by Serial logging of
// what WOULD have been sent.
//
// This lets you validate logic on the bench without the arm connected.
//
// Build-time:  add  -DSIM_MODE  to build_flags in platformio.ini.
// Run-time:    call setSimMode(true) before entering the main loop.

namespace SimMode {

#ifdef SIM_MODE
inline constexpr bool kCompileTimeEnabled = true;
#else
inline constexpr bool kCompileTimeEnabled = false;
#endif

// Runtime flag — starts matching the compile-time setting.
// Can be toggled at startup (e.g. if a boot-time GPIO jumper is read,
// or a serial command is received).
inline bool gRuntimeEnabled = kCompileTimeEnabled;

/// True if simulation mode is active (either compile-time or runtime).
inline bool isEnabled() { return kCompileTimeEnabled || gRuntimeEnabled; }

/// Set the runtime simulation flag.  Has no effect if SIM_MODE is defined
/// at compile time (always enabled in that case).
inline void setEnabled(bool enable) { gRuntimeEnabled = enable; }

}  // namespace SimMode
