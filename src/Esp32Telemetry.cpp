#include "Esp32Telemetry.h"
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>

Esp32Telemetry::Esp32Telemetry(NextionController& controller)
    : controller_(controller) {}

void Esp32Telemetry::begin() {
  // ── Boot-cause detection ──────────────────────────────────────────────
  // If the last reset was triggered by either watchdog, surface that
  // immediately on errMsg + watchStatus so the user knows.
  const esp_reset_reason_t cause = esp_reset_reason();
  if (cause == ESP_RST_TASK_WDT || cause == ESP_RST_INT_WDT || cause == ESP_RST_WDT) {
    setErrorMessage("Recovered from WDT reboot");
    recovered_until_ms_ = millis() + hmi::WATCH_RECOVERED_DISPLAY_MS;
  } else if (cause == ESP_RST_BROWNOUT) {
    setErrorMessage("Recovered from brownout");
  } else if (cause == ESP_RST_PANIC) {
    setErrorMessage("Recovered from panic");
  }
  // Other reasons (POWERON, EXT, SW) leave errMsg in default "None" state,
  // which setErrorMessage("") below establishes via the dedup-friendly path.

  // Always call once with "" so the initial "None" gets pushed if/when the
  // user lands on page1. If a recovery message was already set above this
  // call is a no-op due to setErrorMessage()'s dedup check.
  if (error_msg_[0] == '\0') {
    setErrorMessage("");
  }

  // ── Initialize watchdog ───────────────────────────────────────────────
  // Subscribe the current task (loopTask on Arduino-ESP32) so a hung loop()
  // actually triggers the WDT.
  esp_task_wdt_init(hmi::TWD_TIMEOUT_S, /*panic=*/true);
  esp_task_wdt_add(nullptr);

  loop_start_ms_ = millis();
  last_live_push_ms_ = loop_start_ms_;
}

void Esp32Telemetry::petWatchdog() {
  const uint32_t now = millis();
  const uint32_t loop_dt = now - loop_start_ms_;
  loop_start_ms_ = now;

  // Track the worst loop time observed since the last updateWatchStatus()
  // call. updateWatchStatus() reads and resets this value once per second
  // and decides WARN/OK from the snapshot.
  if (loop_dt > worst_loop_ms_) worst_loop_ms_ = loop_dt;

  esp_task_wdt_reset();
}

void Esp32Telemetry::tick() {
  const uint32_t now = millis();

  if (pending_page_refresh_at_ms_ != 0 &&
      (int32_t)(now - pending_page_refresh_at_ms_) >= 0) {
    pending_page_refresh_at_ms_ = 0;
    pushBootOnce();
    pushAllLive();
    last_live_push_ms_ = now;
  }

  // Page entry one-shot is handled in onPageEntered(). The periodic tick
  // only fires when page1 is already showing.
  if (controller_.currentPageIndex() != hmi::PAGE_IDX_ESP_TELEM) return;

  if (now - last_live_push_ms_ < hmi::LIVE_TELEMETRY_PERIOD_MS) return;
  last_live_push_ms_ = now;

  pushDynamicTick();
}

void Esp32Telemetry::onPageEntered(int page_index) {
  if (page_index != hmi::PAGE_IDX_ESP_TELEM) return;
  // Defer the full refresh until the page has settled on the HMI.
  last_watch_status_ = nullptr;
  last_packet_loss_pct_ = -1;
  last_uptime_sec_ = -1;
  pending_page_refresh_at_ms_ = millis() + hmi::PAGE_SETTLE_MS;
}

void Esp32Telemetry::pushBootOnce() {
  // Arduino-ESP32 core version.
  char core_version[24];
  snprintf(core_version, sizeof(core_version), "%d.%d.%d",
           ESP_ARDUINO_VERSION_MAJOR,
           ESP_ARDUINO_VERSION_MINOR,
           ESP_ARDUINO_VERSION_PATCH);
  controller_.publishTelemetryTextForce("coreVersion", core_version);

  // errMsg is set via setErrorMessage() which dedups — so on a fresh page
  // entry we must explicitly re-push the current message, otherwise the
  // user would see whatever stale value was already on the Nextion field.
  if (error_msg_[0] != '\0') {
    controller_.publishTelemetryTextForce("errMsg", error_msg_);
  }
}

void Esp32Telemetry::pushAllLive() {
  pushDynamicTick();   // also pushes lastCommand and errMsg if non-empty
}

void Esp32Telemetry::pushDynamicTick() {
  const uint32_t now = millis();
  char buf[32];

  // ── cpuTemp ───────────────────────────────────────────────────────────
  // ESP32 internal temperature sensor returns degrees Celsius. The reading
  // is biased toward the die temperature, not ambient — typically reads
  // 20–40°C above ambient depending on load. Useful as a relative trend
  // indicator more than an absolute thermometer.
  const float cpu_c = temperatureRead();
  snprintf(buf, sizeof(buf), "%.1f C", cpu_c);
  controller_.publishTelemetryText("cpuTemp", buf);

  // ── freeMem ───────────────────────────────────────────────────────────
  const uint32_t free_heap = ESP.getFreeHeap();
  snprintf(buf, sizeof(buf), "%u B", (unsigned)free_heap);
  controller_.publishTelemetryText("freeMem", buf);

  // ── upTime ────────────────────────────────────────────────────────────
  const uint32_t up_sec = millis() / 1000;
  if ((int)up_sec != last_uptime_sec_) {
    formatUptime(up_sec, buf, sizeof(buf));
    controller_.publishTelemetryText("upTime", buf);
    last_uptime_sec_ = (int)up_sec;
  }

  // ── watchStatus ───────────────────────────────────────────────────────
  updateWatchStatus();

  // ── packetLoss ────────────────────────────────────────────────────────
  int pct = computePacketLossPct(now);
  if (pct != last_packet_loss_pct_) {
    snprintf(buf, sizeof(buf), "%d%%", pct);
    controller_.publishTelemetryText("packetLoss", buf);
    last_packet_loss_pct_ = pct;
  }

  // Auto-raise an error if parse loss is alarmingly high. When loss drops
  // back down, clear the error (other concurrent errors stay because we
  // only clear if the current message is the one we set).
  if (pct >= hmi::PARSE_LOSS_ERROR_PCT) {
    setErrorMessage("HMI: high parse loss");
  } else if (strcmp(error_msg_, "HMI: high parse loss") == 0) {
    setErrorMessage(nullptr);  // back to "None"
  }

  // ── lastCommand & errMsg ──────────────────────────────────────────────
  // lastCommand is pushed every tick (cheap; usually unchanged so Nextion
  // just redraws identical text). errMsg is push-on-change only — handled
  // inside setErrorMessage() so it never re-pushes the same string.
  if (last_command_[0] != '\0') {
    controller_.publishTelemetryText("lastCommand", last_command_);
  }
}

void Esp32Telemetry::updateWatchStatus() {
  // Snapshot the worst loop time observed since the last update, then reset
  // for the next window. This makes the comparison meaningful — the old
  // code reset worst_loop_ms_ at the *end* of every call, so the warned
  // flag would clear on the very next call regardless of recent loop
  // behavior.
  const uint32_t peak = worst_loop_ms_;
  worst_loop_ms_ = 0;

  const uint32_t now = millis();
  const uint32_t warn_threshold_ms =
      (uint32_t)(hmi::TWD_TIMEOUT_S * 1000.0f * hmi::WATCH_WARN_FRACTION);

  const char* status = "OK";

  if (recovered_until_ms_ != 0 && (int32_t)(now - recovered_until_ms_) < 0) {
    status = "RECOVERED";
  } else if (peak > warn_threshold_ms) {
    // Spike this window — raise/refresh WARN.
    status = "WARN";
    watchdog_warned_ = true;
  } else if (watchdog_warned_) {
    // Previous window was warned, this window was clean — keep showing
    // WARN for one cycle so the user sees it, then drop to OK next time.
    status = "WARN";
    watchdog_warned_ = false;
  }

  if (status != last_watch_status_) {
    controller_.publishTelemetryText("watchStatus", status);
    last_watch_status_ = status;
  }
}

int Esp32Telemetry::computePacketLossPct(uint32_t now_ms) const {
  // The for-loop is bounded by parse_history_count_, which only grows on
  // successful recordHmiFrame() calls. Every iterated slot is therefore
  // populated — no need to skip "uninitialized" entries (and no risk of
  // mis-skipping a real frame received at millis() == 0).
  int total = 0;
  int failed = 0;
  for (int i = 0; i < parse_history_count_; ++i) {
    const ParseSample& sample = parse_history_[i];
    if ((uint32_t)(now_ms - sample.at_ms) > hmi::PARSE_LOSS_WINDOW_MS) continue;
    ++total;
    if (!sample.parsed_ok) ++failed;
  }

  if (total <= 0) return last_packet_loss_pct_ < 0 ? 0 : last_packet_loss_pct_;
  return (failed * 100) / total;
}

void Esp32Telemetry::recordHmiFrame(bool parsed_ok) {
  const int slot = parse_history_index_;
  parse_history_[slot].at_ms = millis();
  parse_history_[slot].parsed_ok = parsed_ok ? 1 : 0;
  parse_history_index_ = (parse_history_index_ + 1) % hmi::PARSE_LOSS_SAMPLE_CAP;
  if (parse_history_count_ < hmi::PARSE_LOSS_SAMPLE_CAP) ++parse_history_count_;
}

void Esp32Telemetry::recordCommand(const char* description) {
  if (description == nullptr) return;
  strncpy(last_command_, description, sizeof(last_command_) - 1);
  last_command_[sizeof(last_command_) - 1] = '\0';

  // Push immediately if page1 is showing — gives the user instant feedback
  // rather than waiting for the next 1 Hz tick.
  if (controller_.currentPageIndex() == hmi::PAGE_IDX_ESP_TELEM) {
    controller_.publishTelemetryText("lastCommand", last_command_);
  }
}

void Esp32Telemetry::setErrorMessage(const char* msg) {
  // Pass nullptr or "" to clear the error to the "no error" state ("None").
  // Any non-empty msg is treated as an actual error condition.
  const char* effective = (msg == nullptr || msg[0] == '\0') ? "None" : msg;

  // Dedup: if the message hasn't changed, do not re-push to the display.
  // This makes setErrorMessage() cheap to call from periodic code paths.
  if (strcmp(error_msg_, effective) == 0) return;

  strncpy(error_msg_, effective, sizeof(error_msg_) - 1);
  error_msg_[sizeof(error_msg_) - 1] = '\0';
  if (controller_.currentPageIndex() == hmi::PAGE_IDX_ESP_TELEM) {
    controller_.publishTelemetryText("errMsg", error_msg_);
  }
}

void Esp32Telemetry::formatUptime(uint32_t total_sec, char* out, size_t out_size) {
  const uint32_t h = total_sec / 3600;
  const uint32_t m = (total_sec % 3600) / 60;
  const uint32_t s = total_sec % 60;
  snprintf(out, out_size, "%u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
}
