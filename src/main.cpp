// main.cpp — top-level glue.
//
// Two amplitude sources feed the TENS driver:
//   * SensorPipeline (EMG/IMU on-board acquisition) — default source.
//   * NextionBridge (host CSV "Quad,Hamstring,AntTib,Calf") — manual override
//     that wins for cfg::MANUAL_OVERRIDE_MS after the latest CSV frame.
//
// ESTOP (from host or HMI) latches all outputs to 0 until RESET.

#include <Arduino.h>
#include "Config.h"
#include "NextionBridge.h"
#include "SensorPipeline.h"
#include "TensDriver.h"

static TensDriver     g_tens;
static TensAmplitudes g_manual_amps;
static uint32_t       g_manual_stamp_ms = 0;
static NextionBridge  g_bridge(g_tens, g_manual_amps, g_manual_stamp_ms);
static SensorPipeline g_pipeline;

static uint32_t g_last_telemetry_ms = 0;

static void emitTelemetry() {
  const auto& emg = g_pipeline.emg();
  Serial.print("EMGv:");
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    Serial.print(emg[i].emgValue());
    Serial.print(i == cfg::NUM_EMG - 1 ? ' ' : ',');
  }
  Serial.print("|SIN:");
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    Serial.print(emg[i].sinusoid(), 2);
    Serial.print(i == cfg::NUM_EMG - 1 ? ' ' : ',');
  }
  Serial.print("|MAG:");
  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    Serial.print(g_pipeline.imu(i).magnitude(), 2);
    Serial.print(i == cfg::NUM_IMU - 1 ? ' ' : ',');
  }
  Serial.print("|EVT:");
  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    Serial.print(g_pipeline.imu(i).event() ? 1 : 0);
    Serial.print(i == cfg::NUM_IMU - 1 ? '\n' : ',');
  }
}

void setup() {
  g_tens.begin();
  g_bridge.begin();      // Serial + Serial2
  g_pipeline.begin();    // pinModes, I²C, MPU wake-up
}

void loop() {
  g_bridge.poll();
  g_pipeline.update();

  TensAmplitudes amps;
  const uint32_t now_ms = millis();
  const bool manual_active =
      g_manual_stamp_ms != 0 &&
      (now_ms - g_manual_stamp_ms) < cfg::MANUAL_OVERRIDE_MS;

  if (manual_active) {
    amps = g_manual_amps;
    g_tens.apply(amps);
  } else if (g_pipeline.hasFreshAmplitudes()) {
    amps = g_pipeline.consumeAmplitudes();
    g_tens.apply(amps);
  }

  if (now_ms - g_last_telemetry_ms >= 100) {
    g_last_telemetry_ms = now_ms;
    emitTelemetry();
  }
}