// main.cpp — top-level glue for the standalone (PC-independent) firmware.
//
// Architecture:
//
//   NextionDisplay  ── single owner of Serial2 (writes only)
//        ▲
//        │
//   NextionController ── system state machine (Standby/Active/Error)
//        ▲                blink scheduling, page nav, EN button logic
//        │                consumes touch events from the bridge
//        │                publishes telemetry on behalf of producers
//        │
//   ┌────┴──────────┬──────────────┬────────────────┐
//   │               │              │                │
// NextionBridge  Esp32Telemetry  GaitMetrics    SensorPipeline
//   (parses          (page1)       (page3)       (EMG + IMU)
//    HMI/host                                         │
//    UART frames)                                     ▼
//                                              TensAmplitudes ──► TensDriver
//                                                                  (gated by
//                                                                   SystemState)

#include <Arduino.h>
#include "Config.h"
#include "Esp32Telemetry.h"
#include "GaitMetrics.h"
#include "HmiConfig.h"
#include "NextionBridge.h"
#include "NextionController.h"
#include "NextionDisplay.h"
// #include "SensorPipeline.h"  // TODO: missing file
#include "TensDriver.h"

// ── Subsystem instances ────────────────────────────────────────────────────

static NextionDisplay    g_display;
static NextionController g_controller(g_display);
static TensDriver        g_tens;
// static SensorPipeline    g_pipeline;  // TODO: missing file
static Esp32Telemetry    g_telem(g_controller);
static GaitMetrics       g_metrics(g_controller);

// Manual amplitude override (dev-Python CSV path)
static TensAmplitudes g_manual_amps;
static uint32_t       g_manual_stamp_ms = 0;

static NextionBridge  g_bridge(g_controller, g_display, g_tens,
                                g_manual_amps, g_manual_stamp_ms);

// Track which page the user was on previously so we can fire onPageEntered
// callbacks exactly once per transition.
static int g_last_page_index = -1;

// ── Callbacks ──────────────────────────────────────────────────────────────

static void onTouchEvent(uint8_t /*page_id*/, uint8_t component_id,
                         uint8_t event, void* /*user*/) {
  // Only press events get a description — release events are noise.
  if (event != hmi::TOUCH_EVENT_PRESS) return;

  // Build a short description for the lastCommand field.
  char desc[48];
  if (component_id == hmi::ESTOP_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "ESTOP");
  } else if (component_id == hmi::NEXT_PAGE_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "Next page");
  } else if (component_id == hmi::PREV_PAGE_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "Prev page");
  } else {
    // Probably the EN button (per-page IDs); the state change callback
    // will follow with EN or RESET depending on the transition.
    snprintf(desc, sizeof(desc), "EN");
  }
  g_telem.recordCommand(desc);
}

static void onStateChange(hmi::SystemState old_state,
                          hmi::SystemState new_state, void* /*user*/) {
  // Mirror state to the ESTOP indicator LED.
  g_tens.setEstopIndicator(new_state == hmi::SystemState::Error);

  // When entering Standby or Error, force outputs to zero immediately
  // (don't wait for the next pipeline tick).
  if (new_state != hmi::SystemState::Active) {
    g_tens.zeroAllOutputs();
  }

  // Surface state transitions on the lastCommand field for visibility.
  char desc[48];
  if (new_state == hmi::SystemState::Active) {
    snprintf(desc, sizeof(desc), "EN");
    g_telem.recordCommand(desc);
  } else if (new_state == hmi::SystemState::Standby && old_state != hmi::SystemState::Standby) {
    // Transitioning to Standby from Active or Error = reset/disable
    snprintf(desc, sizeof(desc), "RESET");
    g_telem.recordCommand(desc);
  }
  // Don't record anything for transitions to Error — ESTOP/reset already did.

  // Clear the error message when we successfully recover from Error.
  if (old_state == hmi::SystemState::Error && new_state == hmi::SystemState::Standby) {
    g_telem.setErrorMessage("");
  }
}

static void onHmiParse(bool parsed_ok, void* /*user*/) {
  g_telem.recordHmiFrame(parsed_ok);
}

// ── Arduino entry points ───────────────────────────────────────────────────

void setup() {
  // Display first (begins Serial2). Display.begin must precede any code
  // that might want to write to the screen.
  // TODO: uncomment when Config.h is available
  g_display.begin(/*baud=*/115200, cfg::NEXTION_RX_PIN, cfg::NEXTION_TX_PIN);

  // Bridge starts host UART (Serial / UART0) for dev/debug.
  g_bridge.begin(/*host_baud=*/115200);
  g_bridge.setHmiParseObserver(&onHmiParse, nullptr);

  // TENS driver: bind to controller state, then begin (which zeros outputs).
  g_tens.bindState(g_controller.statePointer());
  g_tens.begin();

  // Sensor pipeline (I²C + ADC).
  // TODO: uncomment when SensorPipeline.h is available
  // g_pipeline.begin();
  // g_metrics.begin(g_pipeline.emg());

  // Telemetry: must come AFTER bridge (so parse observer is wired) and
  // includes watchdog init and reset-cause check.
  g_telem.begin();

  // Controller callbacks wired before begin() so the initial state-sync
  // doesn't fire callbacks against a half-initialized world.
  g_controller.setTouchCallback(&onTouchEvent, nullptr);
  g_controller.setStateChangeCallback(&onStateChange, nullptr);
  g_controller.begin();

  g_last_page_index = g_controller.currentPageIndex();
}

void loop() {
  // Pet the watchdog first — also tracks loop interval for WARN status.
  g_telem.petWatchdog();

  // Drain UARTs and dispatch frames.
  g_bridge.poll();

  // Run sensor sampling (ISR-paced internally; cheap to call every loop).
  // TODO: uncomment when SensorPipeline.h is available
  // g_pipeline.update();

  // ── TENS amplitude sourcing ───────────────────────────────────────────
  // Manual host CSV wins for MANUAL_OVERRIDE_MS after the latest frame;
  // otherwise the sensor pipeline drives it. TensDriver internally gates
  // by SystemState — output is zero unless state is Active.
  TensAmplitudes amps;
  const uint32_t now_ms = millis();
  // TODO: uncomment when Config.h is available
  // const bool manual_active =
  //     g_manual_stamp_ms != 0 &&
  //     (now_ms - g_manual_stamp_ms) < cfg::MANUAL_OVERRIDE_MS;
  const bool manual_active = false;  // placeholder

  if (manual_active) {
    amps = g_manual_amps;
    g_tens.apply(amps);
  // } else if (g_pipeline.hasFreshAmplitudes()) {
  //   amps = g_pipeline.consumeAmplitudes();
  //   g_tens.apply(amps);
  }

  // ── Page-entry callbacks ──────────────────────────────────────────────
  const int now_page = g_controller.currentPageIndex();
  if (now_page != g_last_page_index) {
    g_telem.onPageEntered(now_page);
    g_metrics.onPageEntered(now_page);
    g_last_page_index = now_page;
  }

  // ── HMI tick (status blink, deferred page-sync) ───────────────────────
  g_controller.tick();

  // ── Telemetry pushes (1 Hz, only when relevant page is active) ────────
  g_telem.tick();
  g_metrics.tick();

  // No delay — loop runs as fast as bytes & sensors allow.
}
