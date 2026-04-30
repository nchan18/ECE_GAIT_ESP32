// main.cpp — top-level glue for the standalone firmware.

#include <Arduino.h>

#include "Config.h"
#include "Esp32Telemetry.h"
#include "GaitMetrics.h"
#include "HmiConfig.h"
#include "NextionBridge.h"
#include "NextionController.h"
#include "NextionDisplay.h"
#include "SensorPipeline.h"
#include "TensDriver.h"

static NextionDisplay    g_display;
static NextionController g_controller(g_display);
static TensDriver        g_tens;
static SensorPipeline    g_pipeline;
static Esp32Telemetry    g_telem(g_controller);
static GaitMetrics       g_metrics(g_controller);

// Manual host CSV path (parsed by NextionBridge on Serial).
static TensAmplitudes g_manual_amps;
static uint32_t       g_manual_stamp_ms = 0;

static NextionBridge g_bridge(
    g_controller, g_display, g_tens, g_manual_amps, g_manual_stamp_ms);

static int g_last_page_index = -1;

static void onTouchEvent(uint8_t /*page_id*/, uint8_t component_id,
                         uint8_t event, void* /*user*/) {
  if (event != hmi::TOUCH_EVENT_PRESS) return;

  char desc[48];
  if (component_id == hmi::ESTOP_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "ESTOP");
  } else if (component_id == hmi::NEXT_PAGE_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "Next page");
  } else if (component_id == hmi::PREV_PAGE_COMPONENT_ID) {
    snprintf(desc, sizeof(desc), "Prev page");
  } else {
    snprintf(desc, sizeof(desc), "EN");
  }
  g_telem.recordCommand(desc);
}

static void onStateChange(hmi::SystemState old_state,
                          hmi::SystemState new_state, void* /*user*/) {
  g_tens.setEstopIndicator(new_state == hmi::SystemState::Error);

  if (new_state != hmi::SystemState::Active) {
    g_tens.zeroAllOutputs();
  }

  char desc[48];
  if (new_state == hmi::SystemState::Active) {
    snprintf(desc, sizeof(desc), "EN");
    g_telem.recordCommand(desc);
  } else if (new_state == hmi::SystemState::Standby &&
             old_state != hmi::SystemState::Standby) {
    snprintf(desc, sizeof(desc), "RESET");
    g_telem.recordCommand(desc);
  }

  if (old_state == hmi::SystemState::Error &&
      new_state == hmi::SystemState::Standby) {
    g_telem.setErrorMessage("");
  }
}

static void onHmiParse(bool parsed_ok, void* /*user*/) {
  g_telem.recordHmiFrame(parsed_ok);
}

void setup() {
  g_display.begin(cfg::HMI_BAUD, cfg::NEXTION_RX_PIN, cfg::NEXTION_TX_PIN);

  g_bridge.begin(cfg::HOST_BAUD);
  g_bridge.setHmiParseObserver(&onHmiParse, nullptr);

  g_tens.bindState(g_controller.statePointer());
  g_tens.begin();

  g_pipeline.begin();
  g_metrics.begin(g_pipeline.emg());

  g_telem.begin();

  g_controller.setTouchCallback(&onTouchEvent, nullptr);
  g_controller.setStateChangeCallback(&onStateChange, nullptr);
  g_controller.begin();

  g_last_page_index = g_controller.currentPageIndex();
}

void loop() {
  g_telem.petWatchdog();

  g_bridge.poll();
  g_pipeline.update();

  const uint32_t now_ms = millis();

  const bool manual_active =
      g_manual_stamp_ms != 0 &&
      (now_ms - g_manual_stamp_ms) < cfg::MANUAL_OVERRIDE_MS;

  if (manual_active) {
    g_tens.apply(g_manual_amps);
  } else if (g_pipeline.hasFreshAmplitudes()) {
    g_tens.apply(g_pipeline.consumeAmplitudes());
  }

  const int now_page = g_controller.currentPageIndex();
  if (now_page != g_last_page_index) {
    g_telem.onPageEntered(now_page);
    g_metrics.onPageEntered(now_page);
    g_last_page_index = now_page;
  }

  g_controller.tick();
  g_telem.tick();
  g_metrics.tick();
}
