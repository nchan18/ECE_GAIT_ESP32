// EmgChannel.h — per-channel EMG pipeline (BPF → rectify → envelope →
// normalise → burst-detected half-sine), plus an array helper that applies
// per-leg quad/hamstring mutual exclusion.
#pragma once

#include "Biquad.h"
#include "Config.h"

class EmgChannel {
public:
  void  begin(float fs_hz);
  void  step(float raw_sample);

  // Latest outputs.
  float envelope()   const { return envelope_; }
  float normEnv()    const { return norm_env_; }
  int   emgValue()   const { return emg_value_; }   // [0..9]
  float sinusoid()   const { return sinusoid_; }    // raw-scaled half-sine
  float rawPeak()    const { return raw_peak_; }
  void  setSinusoid(float v) { sinusoid_ = v; }     // for mutual exclusion

private:
  // 4th-order Butterworth band-pass = two stacked 2nd-order BPFs.
  Biquad bp1_, bp2_;

  // Moving-average envelope.
  static constexpr int ENV_WIN = 100;
  float env_buf_[ENV_WIN] = {0};
  int   env_idx_     = 0;
  float env_sum_     = 0.f;
  bool  env_filled_  = false;

  // Adaptive normalisation.
  float run_min_  = 1e30f;
  float run_max_  = -1e30f;
  float raw_peak_ = 1.0f;

  // Burst tracking.
  bool  in_burst_        = false;
  int   burst_len_       = 0;
  int   gap_len_         = 0;
  float burst_peak_norm_ = 0.f;
  int   burst_min_len_   = 0;
  int   burst_max_gap_   = 0;

  // Latest outputs.
  float envelope_  = 0.f;
  float norm_env_  = 0.f;
  int   emg_value_ = 0;
  float sinusoid_  = 0.f;
};

class EmgArray {
public:
  void begin(float fs_hz);
  void step(const float raw_samples[cfg::NUM_EMG]);

  EmgChannel&       operator[](int i)       { return ch_[i]; }
  const EmgChannel& operator[](int i) const { return ch_[i]; }

  // Map (leg, muscle) → 0..255 PWM amplitude using channel raw_peak as scale.
  int amplitudeForMuscle(char leg, uint8_t muscle) const;

private:
  void applyMutualExclusion();

  EmgChannel ch_[cfg::NUM_EMG];
};
