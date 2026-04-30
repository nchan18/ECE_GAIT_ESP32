// GaitMetrics.h — cross-leg RMSE + peak-timing-error metrics for page3
// ("AI Model Metrics").
//
// "Cross-leg RMSE" = how much the active leg's per-muscle envelope
// differs from the contralateral leg's, averaged over a sliding window.
// Useful as a gait-symmetry indicator that requires no offline model.
//
// Per-muscle RMSEs are computed independently on quadriceps, hamstring,
// and "biceps" (in this project's labeling, biceps = calf — there is no
// EMG sensor mapped to biceps femoris in the current SENSOR_MUSCLE_MAP,
// but the HMI page has a "rmseBiceps" field, so we use it for the calf
// channel which IS sensed bilaterally).
//
// peakTimingErr = milliseconds between left-leg quad burst peak and
// right-leg quad burst peak within the same gait cycle. Zero would mean
// perfectly synchronized; positive values mean one leg leads the other.
#pragma once

#include "HmiConfig.h"
#include "NextionController.h"
#include "EmgChannel.h"
#include <stdint.h>

class GaitMetrics {
public:
  explicit GaitMetrics(NextionController& controller);

  // Provide the EMG array reference — read access only.
  void begin(const EmgArray& emg);

  // Call every loop tick. Internally rate-limited to METRICS_PERIOD_MS.
  void tick();

  // Called by NextionController on navigation to page3 — pushes the most
  // recent computed values immediately rather than waiting for the next
  // periodic update.
  void onPageEntered(int page_index);

private:
  void sample();         // accumulate one sample into the rolling window
  void publish();        // format + push the four metric strings

  // Lookup the EmgChannel index for a (leg, muscle) pair, or -1 if missing.
  int channelFor(char leg, uint8_t muscle) const;

  // Rolling-window RMSE for the difference between two channels' sinusoid()
  // outputs. Window is METRICS_WINDOW samples deep.
  static constexpr int METRICS_WINDOW = 64;

  struct PairAccum {
    float diff_sq_sum = 0.f;
    float buf[METRICS_WINDOW] = {0};
    int   idx           = 0;
    int   count         = 0;     // saturates at METRICS_WINDOW

    void push(float diff);
    float rmse() const;
  };

  // Peak-timing tracker — holds the timestamps of the most recent burst
  // peaks per leg for the quad muscle.
  struct PeakTracker {
    uint32_t last_peak_ms = 0;
    float    peak_value   = 0.f;
    bool     in_burst     = false;

    // Returns true if a new peak was just confirmed (burst end detected).
    bool update(float sinusoid_value, uint32_t now_ms);
  };

  NextionController& controller_;
  const EmgArray*    emg_ = nullptr;

  uint32_t last_sample_ms_   = 0;
  uint32_t last_publish_ms_  = 0;

  PairAccum  rmse_quad_;
  PairAccum  rmse_hamstring_;
  PairAccum  rmse_calf_;

  PeakTracker peak_left_quad_;
  PeakTracker peak_right_quad_;

  // Latest published values cached for re-push on page entry.
  bool   have_published_       = false;
  float  cached_overall_       = 0.f;
  float  cached_rmse_quad_     = 0.f;
  float  cached_rmse_hamstring_ = 0.f;
  float  cached_rmse_calf_     = 0.f;
  float  cached_peak_err_ms_   = 0.f;
};
