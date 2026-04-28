// TensDriver.h — owns the four PWM outputs and the ESTOP latch.
#pragma once

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

  // Apply amplitudes (clamped to [0, 255]).  Forced to 0 while ESTOP latched.
  void apply(const TensAmplitudes& amps);

  void enterEstop(const char* source);
  void clearEstop(const char* source);
  bool estopLatched() const { return estop_latched_; }

private:
  void zeroAllOutputs();

  bool estop_latched_ = false;
};
