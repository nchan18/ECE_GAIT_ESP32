// Esp32Telemetry.h — produces page1 ("ESP32 Telemetry") field values.
//
// Splits into:
//   * Boot-once fields: coreVersion. Pushed at boot and on every
//     navigation to page1.
//   * Live fields (1 Hz): cpuTemp, freeMem, upTime, watchStatus.
//   * Event-driven: lastCommand, errMsg, packetLoss.
//
// Owns the hardware watchdog (esp_task_wdt), the rolling parse-failure
// window for packetLoss, the last-command string, and the error message
// dispatch.
#pragma once

#include "HmiConfig.h"
#include "NextionController.h"
#include <stdint.h>
#include <stddef.h>

class Esp32Telemetry {
public:
  explicit Esp32Telemetry(NextionController& controller);

  void begin();
  void tick();

  // ── Watchdog ─────────────────────────────────────────────────────────────
  // Call at the top of every loop() iteration so the WDT knows we are alive.
  // Also tracks the loop interval for the WARN/OK status determination.
  void petWatchdog();

  // ── Event-driven inputs ──────────────────────────────────────────────────
  // Called by NextionController's touch callback for every HMI frame.
  void recordHmiFrame(bool parsed_ok);

  // Called when a button event is recognized — drives lastCommand display.
  // The string is copied into an internal buffer.
  void recordCommand(const char* description);

  // Called by the rest of the firmware to report an error condition. Latest
  // non-empty error wins. Pass nullptr or "" to clear.
  void setErrorMessage(const char* msg);

  // Called by NextionController when the user navigates to page1 — triggers
  // a one-shot full refresh so the user sees current values immediately
  // instead of waiting for the next 1 Hz tick.
  void onPageEntered(int page_index);

private:
  void pushAllLive();
  void pushBootOnce();
  void pushDynamicTick();
  void updateWatchStatus();
  int  computePacketLossPct(uint32_t now_ms) const;

  // Format helpers
  static void formatUptime(uint32_t total_sec, char* out, size_t out_size);

  NextionController& controller_;

  // Cadence
  uint32_t last_live_push_ms_ = 0;
  uint32_t pending_page_refresh_at_ms_ = 0;

  // Watchdog state
  uint32_t loop_start_ms_         = 0;
  uint32_t worst_loop_ms_         = 0;
  bool     watchdog_warned_       = false;
  uint32_t recovered_until_ms_    = 0;   // 0 if not in recovery display window
  bool     reported_recovery_     = false;

  struct ParseSample {
    uint32_t at_ms    = 0;
    uint8_t  parsed_ok = 0;
  };

  // HMI parse-loss rolling window (stored as timestamped samples).
  ParseSample parse_history_[hmi::PARSE_LOSS_SAMPLE_CAP] = {};
  int         parse_history_count_   = 0;   // number of samples populated (saturates at CAP)
  int         parse_history_index_   = 0;   // next slot to write

  // lastCommand and errMsg buffers
  char     last_command_[48]      = {0};
  char     error_msg_[64]         = {0};

  // Cached watch status string pushed last (reduce redundant UART writes)
  const char* last_watch_status_   = "";
  int         last_packet_loss_pct_ = -1;
  int         last_uptime_sec_      = -1;
  // freeMem and cpuTemp change continuously, no de-dup
};
