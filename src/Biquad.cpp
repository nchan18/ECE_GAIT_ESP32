#include "Biquad.h"
#include <Arduino.h>
#include <math.h>

void Biquad::reset() { z1_ = z2_ = 0.0f; }

float Biquad::process(float x) {
  const float y = b0_ * x + z1_;
  z1_ = b1_ * x - a1_ * y + z2_;
  z2_ = b2_ * x - a2_ * y;
  return y;
}

void Biquad::designBandPass(float f0, float bw, float fs) {
  const float w0    = 2.0f * PI * f0 / fs;
  const float sinw0 = sinf(w0);
  const float cosw0 = cosf(w0);
  const float alpha = sinw0 * sinhf(0.5f * logf(2.0f) * (bw / f0) * (w0 / sinw0));
  const float a0    = 1.0f + alpha;
  b0_ =  alpha / a0;
  b1_ =  0.0f;
  b2_ = -alpha / a0;
  a1_ = -2.0f * cosw0 / a0;
  a2_ = (1.0f - alpha) / a0;
  reset();
}

void Biquad::designLowPass(float fc, float fs, float Q) {
  const float w0    = 2.0f * PI * fc / fs;
  const float cosw0 = cosf(w0);
  const float alpha = sinf(w0) / (2.0f * Q);
  const float a0    = 1.0f + alpha;
  b0_ = (1.0f - cosw0) * 0.5f / a0;
  b1_ = (1.0f - cosw0)        / a0;
  b2_ = (1.0f - cosw0) * 0.5f / a0;
  a1_ = -2.0f * cosw0 / a0;
  a2_ = (1.0f - alpha) / a0;
  reset();
}
