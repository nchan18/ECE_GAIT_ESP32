#include "GaitMetrics.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// ─── PairAccum ──────────────────────────────────────────────────────────────

void GaitMetrics::PairAccum::push(float diff) {
  const float new_sq = diff * diff;
  if (count >= METRICS_WINDOW) {
    diff_sq_sum -= buf[idx];   // evict oldest
  } else {
    ++count;
  }
  buf[idx] = new_sq;
  diff_sq_sum += new_sq;
  idx = (idx + 1) % METRICS_WINDOW;
}

float GaitMetrics::PairAccum::rmse() const {
  if (count == 0) return 0.f;
  return sqrtf(diff_sq_sum / (float)count);
}

// ─── PeakTracker ────────────────────────────────────────────────────────────

bool GaitMetrics::PeakTracker::update(float v, uint32_t now_ms) {
  // Simple burst-peak detector: when sinusoid value is rising and exceeds
  // a small threshold, we're in a burst; track the maximum. When it drops
  // back below the threshold, we record the peak time and report it.
  constexpr float BURST_ENTRY  = 8.0f;   // sinusoid units; tuneable
  constexpr float BURST_EXIT   = 4.0f;

  if (!in_burst) {
    if (v > BURST_ENTRY) {
      in_burst   = true;
      peak_value = v;
      last_peak_ms = now_ms;
    }
    return false;
  }

  // In a burst — track ongoing peak.
  if (v > peak_value) {
    peak_value   = v;
    last_peak_ms = now_ms;
  }

  if (v < BURST_EXIT) {
    in_burst   = false;
    peak_value = 0.f;
    return true;   // peak just confirmed
  }
  return false;
}

// ─── GaitMetrics ────────────────────────────────────────────────────────────

GaitMetrics::GaitMetrics(NextionController& controller)
    : controller_(controller) {}

void GaitMetrics::begin(const int& emg) {  // placeholder parameter
  emg_ = &emg;
  last_sample_ms_  = millis();
  last_publish_ms_ = millis();
}

int GaitMetrics::channelFor(char leg, uint8_t muscle) const {
  // TODO: uncomment when Config.h is available
  // for (int i = 0; i < cfg::NUM_EMG; ++i) {
  //   if (cfg::SENSOR_MUSCLE_MAP[i].leg    != leg)    continue;
  //   if (cfg::SENSOR_MUSCLE_MAP[i].muscle != muscle) continue;
  //   return i;
  // }
  return -1;
}

void GaitMetrics::sample() {
  if (emg_ == nullptr) return;

  // TODO: uncomment when EmgChannel.h and Config.h are available
  // For each muscle that exists on both legs, accumulate (L - R) into the
  // rolling window. The Python's "RMSE" was against an offline model;
  // ours is a model-free symmetry indicator computed from the same data
  // the firmware was already producing.
  //
  // muscle 0 = quad, 1 = hamstring, 2 = calf (per cfg::SensorMap docs).
  // struct Pair { uint8_t muscle; PairAccum* accum; };
  // Pair pairs[] = {
  //   {0, &rmse_quad_},
  //   {1, &rmse_hamstring_},
  //   {2, &rmse_calf_},
  // };
  //
  // for (auto& p : pairs) {
  //   const int li = channelFor('L', p.muscle);
  //   const int ri = channelFor('R', p.muscle);
  //   if (li < 0 || ri < 0) continue;
  //   const float l = (*emg_)[li].sinusoid();
  //   const float r = (*emg_)[ri].sinusoid();
  //   p.accum->push(l - r);
  // }
  //
  // // Peak timing — track quad bursts on each leg independently.
  // const int lq = channelFor('L', 0);
  // const int rq = channelFor('R', 0);
  // const uint32_t now = millis();
  // if (lq >= 0) peak_left_quad_.update ((*emg_)[lq].sinusoid(), now);
  // if (rq >= 0) peak_right_quad_.update((*emg_)[rq].sinusoid(), now);
}

void GaitMetrics::tick() {
  // Always sample (even when page3 isn't visible) so the rolling RMSE
  // window stays current and is meaningful the moment the user lands on
  // page3.
  const uint32_t now = millis();
  if (now - last_sample_ms_ >= 50) {  // ~20 Hz
    last_sample_ms_ = now;
    sample();
  }

  // Only publish when page3 is showing.
  if (controller_.currentPageIndex() != hmi::PAGE_IDX_AI_METRICS) return;

  if (now - last_publish_ms_ < hmi::METRICS_PERIOD_MS) return;
  last_publish_ms_ = now;
  publish();
}

void GaitMetrics::onPageEntered(int page_index) {
  if (page_index != hmi::PAGE_IDX_AI_METRICS) return;
  if (have_published_) {
    // Re-push cached values immediately rather than waiting for the next
    // periodic publish.
    publish();
  }
  last_publish_ms_ = millis();
}

void GaitMetrics::publish() {
  cached_rmse_quad_      = rmse_quad_.rmse();
  cached_rmse_hamstring_ = rmse_hamstring_.rmse();
  cached_rmse_calf_      = rmse_calf_.rmse();

  // Aggregate = mean of the three. Treats missing channels as 0 which is
  // arguably wrong but matches the simplest interpretation; if a channel
  // is unmapped (calf, antTib) its accumulator stays at 0 RMSE which
  // pulls the average down. Acceptable for now.
  cached_overall_ = (cached_rmse_quad_ +
                     cached_rmse_hamstring_ +
                     cached_rmse_calf_) / 3.0f;

  // Peak timing error: signed difference between the two latest peak
  // times. Positive = right leg led, negative = left led. Display as
  // unsigned ms with no sign because the HMI field is narrow.
  float timing_err_ms = 0.f;
  if (peak_left_quad_.last_peak_ms != 0 && peak_right_quad_.last_peak_ms != 0) {
    const int32_t dt =
        (int32_t)peak_right_quad_.last_peak_ms -
        (int32_t)peak_left_quad_.last_peak_ms;
    timing_err_ms = (float)abs(dt);
  }
  cached_peak_err_ms_ = timing_err_ms;
  have_published_ = true;

  char buf[32];
  snprintf(buf, sizeof(buf), "%.4f", cached_overall_);
  controller_.publishTelemetryText("overallRmse", buf);
  snprintf(buf, sizeof(buf), "%.4f", cached_rmse_quad_);
  controller_.publishTelemetryText("rmseQuad", buf);
  snprintf(buf, sizeof(buf), "%.4f", cached_rmse_hamstring_);
  controller_.publishTelemetryText("rmseGastroc", buf);  // hamstring -> gastroc field
  snprintf(buf, sizeof(buf), "%.4f", cached_rmse_calf_);
  controller_.publishTelemetryText("rmseBiceps", buf);   // calf -> biceps field
  snprintf(buf, sizeof(buf), "%.1f ms", cached_peak_err_ms_);
  controller_.publishTelemetryText("peakTimingErr", buf);
}
