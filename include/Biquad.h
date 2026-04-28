// Biquad.h — Direct-Form-II transposed biquad with Butterworth helpers.
#pragma once

class Biquad {
public:
  void reset();
  float process(float x);

  // RBJ-cookbook designs.  Call once at init; coefficients are normalised by a0.
  void designBandPass(float f0_hz, float bandwidth_hz, float fs_hz);
  void designLowPass (float fc_hz, float fs_hz, float Q);

private:
  float b0_ = 1.f, b1_ = 0.f, b2_ = 0.f;
  float a1_ = 0.f, a2_ = 0.f;
  float z1_ = 0.f, z2_ = 0.f;
};

// 4th-order Butterworth = cascade of two biquads with these Q values.
namespace butter4 {
constexpr float Q1 = 0.54119610f;
constexpr float Q2 = 1.30656296f;
}
