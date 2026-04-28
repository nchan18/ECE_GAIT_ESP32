#include "EmgChannel.h"
#include <Arduino.h>
#include <math.h>

void EmgChannel::begin(float fs) {
  // Wide BPF centred at 235 Hz (≈20–450 Hz band) cascaded for steeper roll-off.
  bp1_.designBandPass(235.0f, 430.0f, fs);
  bp2_.designBandPass(235.0f, 430.0f, fs);

  for (int i = 0; i < ENV_WIN; ++i) env_buf_[i] = 0.0f;
  env_idx_    = 0;
  env_sum_    = 0.0f;
  env_filled_ = false;

  run_min_  =  1e30f;
  run_max_  = -1e30f;
  raw_peak_ =  1.0f;

  in_burst_        = false;
  burst_len_       = 0;
  gap_len_         = 0;
  burst_peak_norm_ = 0.0f;
  burst_min_len_   = (int)(0.08f * fs); // ≥ 80 ms burst
  burst_max_gap_   = (int)(0.05f * fs); // ≤ 50 ms gap

  envelope_ = norm_env_ = sinusoid_ = 0.0f;
  emg_value_ = 0;
}

void EmgChannel::step(float raw) {
  // Track running raw peak with slow decay so the scaling adapts.
  const float arx = fabsf(raw);
  if (arx > raw_peak_) raw_peak_ = arx;
  raw_peak_ *= 0.99995f;
  if (raw_peak_ < 1e-3f) raw_peak_ = 1e-3f;

  // 1) 4th-order Butterworth BPF (causal cascade).
  float y = bp1_.process(raw);
  y = bp2_.process(y);

  // 2) Full-wave rectification.
  const float r = fabsf(y);

  // 3) Moving-average envelope.
  env_sum_ -= env_buf_[env_idx_];
  env_buf_[env_idx_] = r;
  env_sum_ += r;
  env_idx_ = (env_idx_ + 1) % ENV_WIN;
  if (!env_filled_ && env_idx_ == 0) env_filled_ = true;
  const float win =
      env_filled_ ? (float)ENV_WIN : (float)(env_idx_ == 0 ? ENV_WIN : env_idx_);
  envelope_ = env_sum_ / win;

  // 4) Running min/max with slow leak.
  if (envelope_ < run_min_) run_min_ = envelope_;
  if (envelope_ > run_max_) run_max_ = envelope_;
  run_min_ += (envelope_ - run_min_) * 1e-5f;
  run_max_ += (envelope_ - run_max_) * 1e-5f;
  const float span = fmaxf(run_max_ - run_min_, 1e-9f);
  norm_env_ = fminf(fmaxf((envelope_ - run_min_) / span, 0.0f), 1.0f);

  // 5) Integer level [0, 9].
  emg_value_ = (int)lroundf(norm_env_ * 9.0f);

  // 6) Burst detection → half-sine amplitude.
  constexpr float kThreshold = 0.30f;
  const bool active = norm_env_ >= kThreshold;
  if (active) {
    if (!in_burst_) {
      in_burst_        = true;
      burst_len_       = 0;
      burst_peak_norm_ = norm_env_;
    }
    burst_len_++;
    gap_len_ = 0;
    if (norm_env_ > burst_peak_norm_) burst_peak_norm_ = norm_env_;
  } else if (in_burst_) {
    gap_len_++;
    if (gap_len_ > burst_max_gap_) {
      in_burst_        = false;
      burst_len_       = 0;
      burst_peak_norm_ = 0.0f;
      gap_len_         = 0;
    } else {
      burst_len_++; // tolerate small dips inside a burst
    }
  }

  float sinus_unit = 0.0f;
  if (in_burst_ && burst_len_ >= burst_min_len_) {
    sinus_unit = burst_peak_norm_ * sinf(PI * fminf(norm_env_, 1.0f) * 0.5f);
  }
  sinusoid_ = sinus_unit * raw_peak_;
}

// ---------------------------------------------------------------------------
// EmgArray
// ---------------------------------------------------------------------------

void EmgArray::begin(float fs) {
  for (int i = 0; i < cfg::NUM_EMG; ++i) ch_[i].begin(fs);
}

void EmgArray::step(const float raw[cfg::NUM_EMG]) {
  for (int i = 0; i < cfg::NUM_EMG; ++i) ch_[i].step(raw[i]);
  applyMutualExclusion();
}

void EmgArray::applyMutualExclusion() {
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    if (cfg::SENSOR_MUSCLE_MAP[i].muscle != 0) continue; // quad
    for (int j = 0; j < cfg::NUM_EMG; ++j) {
      if (cfg::SENSOR_MUSCLE_MAP[j].leg    != cfg::SENSOR_MUSCLE_MAP[i].leg) continue;
      if (cfg::SENSOR_MUSCLE_MAP[j].muscle != 1) continue; // matching hamstring
      const float si = ch_[i].sinusoid();
      const float sj = ch_[j].sinusoid();
      if (si > sj)      ch_[j].setSinusoid(0.0f);
      else if (sj > si) ch_[i].setSinusoid(0.0f);
    }
  }
}

int EmgArray::amplitudeForMuscle(char leg, uint8_t muscle) const {
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    if (cfg::SENSOR_MUSCLE_MAP[i].leg    != leg)    continue;
    if (cfg::SENSOR_MUSCLE_MAP[i].muscle != muscle) continue;
    const float peak = fmaxf(ch_[i].rawPeak(), 1e-3f);
    const float u    = fminf(fmaxf(ch_[i].sinusoid() / peak, 0.0f), 1.0f);
    return (int)lroundf(u * 255.0f);
  }
  return 0;
}
