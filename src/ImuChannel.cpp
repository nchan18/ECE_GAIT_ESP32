#include "ImuChannel.h"
#include <Arduino.h>
#include <math.h>

void ImuChannel::begin(float fs) {
  // 4th-order LPF @ 6 Hz (cascade of two 2nd-order sections).
  lp_x1_.designLowPass(6.0f, fs, butter4::Q1);
  lp_x2_.designLowPass(6.0f, fs, butter4::Q2);
  lp_y1_.designLowPass(6.0f, fs, butter4::Q1);
  lp_y2_.designLowPass(6.0f, fs, butter4::Q2);
  lp_z1_.designLowPass(6.0f, fs, butter4::Q1);
  lp_z2_.designLowPass(6.0f, fs, butter4::Q2);

  gx_f_ = gy_f_ = gz_f_ = 0.f;
  magnitude_ = 0.f;
  ang_x_ = ang_y_ = ang_z_ = 0.f;
  prev_gx_ = prev_gy_ = prev_gz_ = 0.f;
  mag_run_mean_  = 0.f;
  mag_run_var_   = 1.f;
  last_event_ms_ = 0;
  event_ = false;
}

void ImuChannel::step(float gx, float gy, float gz, float dt) {
  gx_f_ = lp_x2_.process(lp_x1_.process(gx));
  gy_f_ = lp_y2_.process(lp_y1_.process(gy));
  gz_f_ = lp_z2_.process(lp_z1_.process(gz));

  magnitude_ = sqrtf(gx_f_ * gx_f_ + gy_f_ * gy_f_ + gz_f_ * gz_f_);

  // Trapezoidal integration to angle.
  ang_x_ += 0.5f * (gx_f_ + prev_gx_) * dt;
  ang_y_ += 0.5f * (gy_f_ + prev_gy_) * dt;
  ang_z_ += 0.5f * (gz_f_ + prev_gz_) * dt;
  prev_gx_ = gx_f_;
  prev_gy_ = gy_f_;
  prev_gz_ = gz_f_;

  // Slow drift removal (1-pole HPF, fc ≈ 0.05 Hz).
  const float drift_alpha = 1.0f - expf(-2.0f * PI * 0.05f * dt);
  ang_x_ -= drift_alpha * ang_x_;
  ang_y_ -= drift_alpha * ang_y_;
  ang_z_ -= drift_alpha * ang_z_;

  // Adaptive threshold via running mean/variance.
  constexpr float a = 1e-3f;
  const float diff = magnitude_ - mag_run_mean_;
  mag_run_mean_ += a * diff;
  mag_run_var_  += a * (diff * diff - mag_run_var_);
  const float std_dev = sqrtf(fmaxf(mag_run_var_, 1e-9f));
  const float thresh  = mag_run_mean_ + 0.5f * std_dev;

  event_ = false;
  const uint32_t now = millis();
  if (magnitude_ > thresh && (now - last_event_ms_) > 400u) {
    event_ = true;
    last_event_ms_ = now;
  }
}
