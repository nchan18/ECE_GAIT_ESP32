#include "TensDriver.h"
// #include "Config.h"  // TODO: missing file
#include <Arduino.h>

void TensDriver::begin() {
  // TODO: uncomment when Config.h is available
  // pinMode(cfg::CS_CALF,      OUTPUT);
  // pinMode(cfg::CS_ANTTIB,    OUTPUT);
  // pinMode(cfg::CS_HAMSTRING, OUTPUT);
  // pinMode(cfg::CS_QUAD,      OUTPUT);
  // pinMode(cfg::LED_ESTOP,    OUTPUT);
  // zeroAllOutputs();
  // digitalWrite(cfg::LED_ESTOP, LOW);
}

void TensDriver::zeroAllOutputs() {
  // TODO: uncomment when Config.h is available
  // analogWrite(cfg::CS_CALF,      0);
  // analogWrite(cfg::CS_ANTTIB,    0);
  // analogWrite(cfg::CS_HAMSTRING, 0);
  // analogWrite(cfg::CS_QUAD,      0);
}

void TensDriver::apply(const TensAmplitudes& a) {
  // Defensive: if no state has been bound (programmer error), force-zero.
  if (state_ptr_ == nullptr || *state_ptr_ != hmi::SystemState::Active) {
    zeroAllOutputs();
    return;
  }
  // TODO: uncomment when Config.h is available
  // analogWrite(cfg::CS_CALF,      constrain(a.calfAmp,      0, 255));
  // analogWrite(cfg::CS_ANTTIB,    constrain(a.antTibAmp,    0, 255));
  // analogWrite(cfg::CS_HAMSTRING, constrain(a.hamstringAmp, 0, 255));
  // analogWrite(cfg::CS_QUAD,      constrain(a.quadAmp,      0, 255));
}

void TensDriver::setEstopIndicator(bool on) {
  // TODO: uncomment when Config.h is available
  // digitalWrite(cfg::LED_ESTOP, on ? HIGH : LOW);
}
