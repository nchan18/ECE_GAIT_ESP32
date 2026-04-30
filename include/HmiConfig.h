// HmiConfig.h — Nextion HMI constants ported from nextion_config.py.
//
// Mirrors the structure of the original Python config but flattened into
// constexpr arrays/enums so it can sit in flash with no runtime allocation.
//
// All page-keyed metadata lives in PAGES[] (single source of truth). When you
// add or reorder pages in the Nextion editor, update PAGES[] here and the
// rest of the firmware picks it up automatically.
#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace hmi {

// ---------------------------------------------------------------------------
// SystemState — replaces the Python "estop_triggered + status_pic" pair with
// a single source of truth that gates UI display *and* TENS output.
//   Standby : sensors run, TENS forced to 0, UI green
//   Active  : sensors run, TENS follows SensorPipeline, UI yellow
//   Error   : ESTOP latched, TENS forced to 0, UI red blinking
// ---------------------------------------------------------------------------
enum class SystemState : uint8_t {
  Standby = 0,
  Active  = 1,
  Error   = 2,
};

// ---------------------------------------------------------------------------
// Status LED image IDs on the Nextion display.
// (Match nextion_config.py: STANDBY=0, ERROR=1, ACTIVE=2, BLANK=3.)
// ---------------------------------------------------------------------------
constexpr int STATUS_PIC_STANDBY = 0;
constexpr int STATUS_PIC_ERROR   = 1;
constexpr int STATUS_PIC_ACTIVE  = 2;
constexpr int STATUS_PIC_BLANK   = 3;

// Maps SystemState -> LED image (for the "lit" half of the blink cycle).
constexpr int statusPicForState(SystemState s) {
  return s == SystemState::Error  ? STATUS_PIC_ERROR
       : s == SystemState::Active ? STATUS_PIC_ACTIVE
       :                            STATUS_PIC_STANDBY;
}

// ---------------------------------------------------------------------------
// Status text overlay (statusTxt) label + Nextion RGB565 colors per state.
// ---------------------------------------------------------------------------
constexpr const char* statusLabelForState(SystemState s) {
  return s == SystemState::Error  ? "Error"
       : s == SystemState::Active ? "Active"
       :                            "Standby";
}

constexpr uint16_t statusColorForState(SystemState s) {
  return s == SystemState::Error  ? 63488   // red
       : s == SystemState::Active ? 65504   // yellow
       :                            2016;   // green
}

// ---------------------------------------------------------------------------
// Blink cadences (seconds per half-cycle) — slower in Standby, faster in
// Active, fastest in Error. Indexed by SystemState.
// ---------------------------------------------------------------------------
constexpr float BLINK_PERIOD_S[] = {
  0.5f,   // Standby
  0.3f,   // Active
  0.1f,   // Error
};
static_assert(sizeof(BLINK_PERIOD_S) / sizeof(BLINK_PERIOD_S[0]) == 3,
              "BLINK_PERIOD_S must cover all SystemState values");

constexpr uint32_t blinkPeriodMs(SystemState s) {
  return (uint32_t)(BLINK_PERIOD_S[(uint8_t)s] * 1000.0f);
}

// ---------------------------------------------------------------------------
// Page-settle delay: after a page change, the Nextion needs a brief moment
// before object writes land reliably. Same value as the Python's
// PAGE_SETTLE_TIME (0.08s).
// ---------------------------------------------------------------------------
constexpr uint32_t PAGE_SETTLE_MS = 80;

// ---------------------------------------------------------------------------
// Page table — single source of truth for everything keyed by page.
//
// `nextion_id` is the numeric page ID in the Nextion editor (used by touch
// events). `index` (implicit) is the position in this array, used everywhere
// in the firmware as the "logical page index". They should match and are
// kept in order: page0 (id=0), page1 (id=1), page2 (id=2), page3 (id=3).
//
// `en_component_id` is per-page because the EN button was added separately
// to each page and didn't get a consistent ID.
//
// `status_blank_pic` is the LED image used for the "off" half of the blink
// cycle. Use STATUS_PIC_BLANK (3) for standard pages.
// ---------------------------------------------------------------------------
struct PageInfo {
  const char* name;             // Nextion page name (used in "page <name>")
  uint8_t     nextion_id;       // Numeric page ID returned in touch events
  uint8_t     en_component_id;  // Component ID of the EN button on this page
  int         status_blank_pic; // LED image for blink "off"
};

constexpr PageInfo PAGES[] = {
  // name      nextion_id  en_id  blank_pic
  {  "page0",   0,           16,   STATUS_PIC_BLANK  },  // TENS Output Indicator
  {  "page1",   1,           33,   STATUS_PIC_BLANK  },  // ESP32 Telemetry
  {  "page2",   2,           25,   STATUS_PIC_BLANK  },  // Acknowledgments (different bg)
  {  "page3",   3,            8,   5                 },  // AI Model Metrics
};
constexpr int NUM_PAGES = sizeof(PAGES) / sizeof(PAGES[0]);

// Logical page indices for code that needs to special-case a specific page
// (e.g. metrics push only when page3 is active). Using these named constants
// avoids magic numbers — if PAGES[] is reordered, only these need updating.
constexpr int PAGE_IDX_TENS_OUTPUT  = 0;  // page0
constexpr int PAGE_IDX_ESP_TELEM    = 1;  // page1
constexpr int PAGE_IDX_ACK          = 2;  // page2
constexpr int PAGE_IDX_AI_METRICS   = 3;  // page3

constexpr bool WRAP_PAGES = true;

// ---------------------------------------------------------------------------
// Component IDs that are universal across all pages (added globally in the
// Nextion editor with consistent IDs).
// ---------------------------------------------------------------------------
constexpr uint8_t ESTOP_COMPONENT_ID     = 3;
constexpr uint8_t NEXT_PAGE_COMPONENT_ID = 5;
constexpr uint8_t PREV_PAGE_COMPONENT_ID = 6;

// Touch event bytes: byte 3 of a Nextion 0x65 touch packet is 0x01 for press,
// 0x00 for release. Naming these makes the controller code read better.
constexpr uint8_t TOUCH_EVENT_PRESS   = 0x01;
constexpr uint8_t TOUCH_EVENT_RELEASE = 0x00;

// ---------------------------------------------------------------------------
// EN button styling — two visual states.
//
// Idle ("Enable"): white text on dark bg, used in Standby and after ESTOP
//                  recovery. Means "press me to start".
//
// Reset ("RESET"): dark text on yellow bg (matches statusColor for Active),
//                  used in Active and Error. Means "press me to stop/clear".
//
// Each style sets 5 properties: txt, pco, pco2, bco, bco2. The matching
// 5-write apply sequence lives in NextionDisplay.
// ---------------------------------------------------------------------------
struct EnButtonStyle {
  const char* txt;
  uint16_t    pco;     // text color (idle press state)
  uint16_t    pco2;    // text color (pressed state)
  uint16_t    bco;     // background color (idle)
  uint16_t    bco2;    // background color (pressed)
};

constexpr EnButtonStyle EN_STYLE_IDLE = {
  "Enable",
  /*pco =*/ 65535,
  /*pco2=*/ 65535,
  /*bco =*/ 1032,
  /*bco2=*/ 1032,
};

constexpr EnButtonStyle EN_STYLE_RESET = {
  "RESET",
  /*pco =*/ 1,
  /*pco2=*/ 1,
  /*bco =*/ 65504,    // matches Active status color (yellow)
  /*bco2=*/ 1032,
};

// Maps SystemState -> EN button style. Idle in Standby, Reset in Active and
// Error (both "press me to clear" semantically).
constexpr const EnButtonStyle& enStyleForState(SystemState s) {
  return s == SystemState::Standby ? EN_STYLE_IDLE : EN_STYLE_RESET;
}

// ---------------------------------------------------------------------------
// Telemetry binding table.
//
// Slimmed from the Python version: now that the firmware is the only sender
// of telemetry, we control the keys, so no multi-name aliases or
// case-insensitive normalization. One canonical key per Nextion target.
//
// `mode`:
//   Txt -> writes <object>.txt="<value>"
//   Val -> writes <object>.val=<int(value)>
//
// `target_page_idx`: which page this object lives on (logical index into
// PAGES[]). Writes to objects on inactive pages are wasted UART bytes —
// callers (Esp32Telemetry, GaitMetrics) check this before pushing.
// ---------------------------------------------------------------------------
enum class BindingMode : uint8_t { Txt, Val };

struct TelemetryBinding {
  const char* key;             // canonical telemetry key
  const char* nextion_object;  // Nextion display object name
  BindingMode mode;
  int         target_page_idx; // PAGE_IDX_* constant
};

constexpr TelemetryBinding TELEMETRY_BINDINGS[] = {
  // ── Page 1: ESP32 Telemetry ──────────────────────────────────────────────
  {  "coreVersion",   "coreVersion",   BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "cpuTemp",       "cpuTemp",       BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "upTime",        "upTime",        BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "freeMem",       "freeMem",       BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "packetLoss",    "packetLoss",    BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "lastCommand",   "lastCommand",   BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "watchStatus",   "watchStatus",   BindingMode::Txt, PAGE_IDX_ESP_TELEM  },
  {  "errMsg",        "errMsg",        BindingMode::Txt, PAGE_IDX_ESP_TELEM  },

  // ── Page 3 (logical idx 3): AI Model Metrics ─────────────────────────────
  {  "overallRmse",   "overallRmse",   BindingMode::Txt, PAGE_IDX_AI_METRICS },
  {  "rmseGastroc",   "rmseGastroc",   BindingMode::Txt, PAGE_IDX_AI_METRICS },
  {  "rmseQuad",      "rmseQuad",      BindingMode::Txt, PAGE_IDX_AI_METRICS },
  {  "rmseBiceps",    "rmseBiceps",    BindingMode::Txt, PAGE_IDX_AI_METRICS },
  {  "peakTimingErr", "peakTimingErr", BindingMode::Txt, PAGE_IDX_AI_METRICS },
};
constexpr int NUM_TELEMETRY_BINDINGS =
    sizeof(TELEMETRY_BINDINGS) / sizeof(TELEMETRY_BINDINGS[0]);

// Linear-scan lookup. Table is small (~14 entries) and lookups happen at most
// at telemetry rate (~1 Hz), so a hash map would be overkill. Returns
// nullptr if no binding exists for `key`.
//
inline const TelemetryBinding* findBinding(const char* key) {
  for (int i = 0; i < NUM_TELEMETRY_BINDINGS; ++i) {
    const char* a = TELEMETRY_BINDINGS[i].key;
    const char* b = key;
    bool match = true;
    while (*a && *b) {
      if (*a != *b) { match = false; break; }
      ++a; ++b;
    }
    if (match && *a == '\0' && *b == '\0') {
      return &TELEMETRY_BINDINGS[i];
    }
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Watchdog parameters.
//
// TWD_TIMEOUT_S must comfortably exceed the worst-case loop() iteration. The
// SensorPipeline's I²C reads are the slowest reliable operation; everything
// else is microseconds. 5 seconds gives plenty of margin while still
// catching a real hang.
//
// WATCH_WARN_FRACTION: if a single loop() iteration exceeds this fraction of
// the timeout, watchStatus reports "WARN" — early indicator of trouble.
// ---------------------------------------------------------------------------
constexpr uint32_t TWD_TIMEOUT_S        = 5;
constexpr float    WATCH_WARN_FRACTION  = 0.5f;

// On WDT-recovery boot, watchStatus shows "RECOVERED" for this many
// milliseconds before reverting to "OK".
constexpr uint32_t WATCH_RECOVERED_DISPLAY_MS = 10000;

// ---------------------------------------------------------------------------
// Telemetry & metrics cadence (page1, page3).
// ---------------------------------------------------------------------------
constexpr uint32_t LIVE_TELEMETRY_PERIOD_MS = 1000;  // 1 Hz: cpuTemp, freeMem, upTime
constexpr uint32_t METRICS_PERIOD_MS        = 500;   // 2 Hz: cross-leg RMSE updates

// ---------------------------------------------------------------------------
// HMI frame parse-loss tracking (for the redefined `packetLoss` field).
//
// Rolling time window over recent HMI frames. This makes packetLoss a more
// standardized health view than a tiny frame-count window, since the result
// is always computed over the last N milliseconds.
// ---------------------------------------------------------------------------
constexpr uint32_t PARSE_LOSS_WINDOW_MS = 60000;  // 60-second health window
constexpr int      PARSE_LOSS_SAMPLE_CAP = 256;    // ring buffer capacity

// Threshold for raising errMsg = "HMI: high parse loss".
constexpr int PARSE_LOSS_ERROR_PCT = 25;

// ---------------------------------------------------------------------------
// Compile-time identity strings shown on the HMI.
// ---------------------------------------------------------------------------
constexpr const char* FIRMWARE_VERSION = "gait-1.0";

} // namespace hmi
