// TensDriver.h — owns the four PWM outputs.
//
// CHANGES FROM ORIGINAL:
//   * The internal estop_latched_ bool is replaced by a pointer to a
//     SystemState. apply() zeros outputs unless the state is Active.
//   * enterEstop()/clearEstop() are kept for compatibility with the
//     external host-control path in NextionBridge but they now just
//     forward to the controller via the registered callback. The actual
//     latch lives in NextionController's SystemState.
#pragma once

#include "HmiConfig.h"
#include <stdint.h>

struct TensAmplitudes {
  int quadAmp      = 0;
  int hamstringAmp = 0;
  int antTibAmp    = 0;
  int calfAmp      = 0;
};

class TensDriver {
public:
  void begin();

  // Bind the controller's state so apply() can read it. Must be called
  // before any apply() / state transition.
  void bindState(const hmi::SystemState* state) { state_ptr_ = state; }

  // Apply amplitudes (clamped to [0, 255]). Forced to 0 when state is
  // anything other than Active.
  void apply(const TensAmplitudes& amps);

  // ESTOP indicator LED — driven directly by the state-change callback
  // from NextionController. Independent from the gating above; the LED
  // just mirrors "are we currently in Error state".
  void setEstopIndicator(bool on);

  // Convenience: zero all outputs immediately. Called by the state-change
  // callback when entering Standby or Error.
  void zeroAllOutputs();

private:
  const hmi::SystemState* state_ptr_ = nullptr;
};
