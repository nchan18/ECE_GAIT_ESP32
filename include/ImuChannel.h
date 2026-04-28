// ImuChannel.h — per-sensor IMU pipeline: 4th-order LPF, magnitude,
// drift-corrected orientation integration, and adaptive event detection.
#pragma once

#include "Biquad.h"
#include <stdint.h>

class ImuChannel {
public:
  void begin(float fs_hz);
  void step(float gx_dps, float gy_dps, float gz_dps, float dt_s);

  float magnitude() const { return magnitude_; }
  float angX()      const { return ang_x_; }
  float angY()      const { return ang_y_; }
  float angZ()      const { return ang_z_; }
  bool  event()     const { return event_; }

private:
  // Two cascaded biquads per axis = 4th-order Butterworth.
  Biquad lp_x1_, lp_x2_, lp_y1_, lp_y2_, lp_z1_, lp_z2_;

  float gx_f_ = 0.f, gy_f_ = 0.f, gz_f_ = 0.f;
  float magnitude_ = 0.f;

  float ang_x_ = 0.f, ang_y_ = 0.f, ang_z_ = 0.f;
  float prev_gx_ = 0.f, prev_gy_ = 0.f, prev_gz_ = 0.f;

  float    mag_run_mean_ = 0.f;
  float    mag_run_var_  = 1.f;
  uint32_t last_event_ms_ = 0;
  bool     event_ = false;
};
