#include "TensDriver.h"
#include "Config.h"
#include <Arduino.h>

void TensDriver::begin() {
  pinMode(cfg::CS_CALF,      OUTPUT);
  pinMode(cfg::CS_ANTTIB,    OUTPUT);
  pinMode(cfg::CS_HAMSTRING, OUTPUT);
  pinMode(cfg::CS_QUAD,      OUTPUT);
  pinMode(cfg::LED_ESTOP,    OUTPUT);
  zeroAllOutputs();
}

void TensDriver::zeroAllOutputs() {
  analogWrite(cfg::CS_CALF,      0);
  analogWrite(cfg::CS_ANTTIB,    0);
  analogWrite(cfg::CS_HAMSTRING, 0);
  analogWrite(cfg::CS_QUAD,      0);
}

void TensDriver::apply(const TensAmplitudes& a) {
  if (estop_latched_) {
    zeroAllOutputs();
    return;
  }
  analogWrite(cfg::CS_CALF,      constrain(a.calfAmp,      0, 255));
  analogWrite(cfg::CS_ANTTIB,    constrain(a.antTibAmp,    0, 255));
  analogWrite(cfg::CS_HAMSTRING, constrain(a.hamstringAmp, 0, 255));
  analogWrite(cfg::CS_QUAD,      constrain(a.quadAmp,      0, 255));
}

void TensDriver::enterEstop(const char* source) {
  estop_latched_ = true;
  zeroAllOutputs();
  digitalWrite(cfg::LED_ESTOP, HIGH);
  Serial.print("SYS,estop=1,src=");
  Serial.println(source);
}

void TensDriver::clearEstop(const char* source) {
  estop_latched_ = false;
  digitalWrite(cfg::LED_ESTOP, LOW);
  Serial.print("SYS,estop=0,src=");
  Serial.println(source);
}
