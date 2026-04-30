// main.cpp — top-level glue (strict host-in-the-loop).
//
// Architecture:
//   1. SensorPipeline samples EMG @ EMG_FS and IMU @ IMU_FS, runs the
//      preprocessing chain, and exposes per-channel features.
//   2. Telemetry frames (EMG levels, sinusoids, IMU magnitudes, events) are
//      streamed up to the host over the inference link every
//      cfg::TELEMETRY_PERIOD_MS.
//   3. The host runs inference and sends TENS amplitudes back as
//        TENS,<quad>,<hamstring>,<antTib>,<calf>\n
//      on the same inference link.  Those amplitudes are the ONLY source
//      driving g_tens — there is no on-device sensor → TENS fallback.
//   4. If no host TENS command arrives within cfg::HOST_COMMAND_TIMEOUT_MS,
//      outputs are forced to 0 (failsafe).
//
// The Nextion bridge remains in place for the HMI (Serial2) and accepts
// ESTOP / RESET from either the host (USB) or the HMI.  Its legacy CSV
// amplitude path is retained as a secondary host channel.
//
// ESTOP (from host or HMI, on either link) latches all outputs to 0 until
// RESET.

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include "Config.h"
#include "HostLink.h"
#include "NextionBridge.h"
#include "SensorPipeline.h"
#include "TensDriver.h"

static TensDriver     g_tens;
static TensAmplitudes g_manual_amps;
static uint32_t       g_manual_stamp_ms = 0;
static NextionBridge  g_bridge(g_tens, g_manual_amps, g_manual_stamp_ms);
static SensorPipeline g_pipeline;
static HostLink&      g_inference_link = makeHostLink();

static uint32_t g_last_telemetry_ms   = 0;
static uint32_t g_last_host_cmd_ms    = 0;
static bool     g_have_host_cmd       = false;
static TensAmplitudes g_host_amps;

// Line buffer for parsing host commands on the inference link.
static char     g_host_line[96];
static size_t   g_host_line_len = 0;

// Compact processed-EMG frame: per-channel level [0..9] and sinusoid
// amplitude (post-mutual-exclusion).  Followed by IMU magnitude and event
// flags so the host can run higher-level inference.
static void emitTelemetry(Stream& out) {
  const auto& emg = g_pipeline.emg();

  out.print("EMG,");
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    out.print(emg[i].emgValue());
    out.print(i == cfg::NUM_EMG - 1 ? '|' : ',');
  }
  out.print("SIN,");
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    out.print(emg[i].sinusoid(), 2);
    out.print(i == cfg::NUM_EMG - 1 ? '|' : ',');
  }
  out.print("MAG,");
  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    out.print(g_pipeline.imu(i).magnitude(), 2);
    out.print(i == cfg::NUM_IMU - 1 ? '|' : ',');
  }
  out.print("EVT,");
  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    out.print(g_pipeline.imu(i).event() ? 1 : 0);
    out.print(i == cfg::NUM_IMU - 1 ? '\n' : ',');
  }
}

// Parse a single newline-terminated host command of the form:
//   TENS,<quad>,<hamstring>,<antTib>,<calf>
//   ESTOP
//   RESET
// Returns true if the line was a recognised TENS command (amps populated).
static bool parseHostLine(const char* line, TensAmplitudes& out) {
  // Strip CR / leading whitespace.
  while (*line == ' ' || *line == '\t' || *line == '\r') ++line;
  if (*line == '\0') return false;

  // Case-insensitive keyword check.
  auto starts_with = [](const char* s, const char* kw) {
    while (*kw) {
      if (toupper((unsigned char)*s) != *kw) return false;
      ++s; ++kw;
    }
    return true;
  };

  if (starts_with(line, "ESTOP")) { g_tens.enterEstop("host-link"); return false; }
  if (starts_with(line, "RESET")) { g_tens.clearEstop("host-link"); return false; }

  if (!starts_with(line, "TENS,")) return false;
  line += 5;

  int vals[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    char* endp = nullptr;
    long v = strtol(line, &endp, 10);
    if (endp == line) return false;
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
    vals[i] = (int)v;
    line = endp;
    if (i < 3) {
      if (*line != ',') return false;
      ++line;
    }
  }
  out.quadAmp      = vals[0];
  out.hamstringAmp = vals[1];
  out.antTibAmp    = vals[2];
  out.calfAmp      = vals[3];
  return true;
}

static void pollHostCommands() {
  // In UART mode the inference link shares `Serial` with NextionBridge, which
  // owns the read side and already routes host CSV amplitudes into
  // g_manual_amps.  Reading here would steal bytes from the bridge parser.
  if (g_inference_link.mode() == cfg::InferenceMode::Uart) return;

  Stream& in = g_inference_link.stream();
  while (in.available() > 0) {
    const char c = (char)in.read();
    if (c == '\n') {
      g_host_line[g_host_line_len] = '\0';
      TensAmplitudes amps;
      if (parseHostLine(g_host_line, amps)) {
        g_host_amps         = amps;
        g_last_host_cmd_ms  = millis();
        g_have_host_cmd     = true;
      }
      g_host_line_len = 0;
    } else if (g_host_line_len + 1 < sizeof(g_host_line)) {
      g_host_line[g_host_line_len++] = c;
    } else {
      // Overflow — drop the line.
      g_host_line_len = 0;
    }
  }
}

void setup() {
  g_tens.begin();
  g_bridge.begin();          // Serial (host) + Serial2 (Nextion)
  g_inference_link.begin();  // UART -> reuses Serial; BT -> BluetoothSerial
  g_pipeline.begin();        // pinModes, I²C, MPU wake-up
}

void loop() {
  g_bridge.poll();          // Nextion HMI + ESTOP/RESET passthrough
  g_pipeline.update();      // sensor sampling + preprocessing (telemetry only)
  pollHostCommands();       // host -> TENS amplitudes on the inference link

  const uint32_t now_ms = millis();

  // Strict host-in-the-loop arbitration.  In priority order:
  //   1. Nextion-bridge CSV (legacy host channel via USB-serial framed for HMI)
  //   2. Inference-link TENS,... command from host
  //   3. Failsafe zero (host watchdog timeout — no on-device sensor fallback).
  const bool nextion_active =
      g_manual_stamp_ms != 0 &&
      (now_ms - g_manual_stamp_ms) < cfg::MANUAL_OVERRIDE_MS;
  const bool host_cmd_active =
      g_have_host_cmd &&
      (now_ms - g_last_host_cmd_ms) < cfg::HOST_COMMAND_TIMEOUT_MS;

  if (nextion_active) {
    g_tens.apply(g_manual_amps);
  } else if (host_cmd_active) {
    g_tens.apply(g_host_amps);
  } else {
    TensAmplitudes zero;
    g_tens.apply(zero);
  }

  // Drain stale sensor amplitudes so they never accumulate; they are not
  // used to drive TENS in strict host-in-the-loop mode.
  if (g_pipeline.hasFreshAmplitudes()) (void)g_pipeline.consumeAmplitudes();

  if (g_inference_link.ready() &&
      now_ms - g_last_telemetry_ms >= cfg::TELEMETRY_PERIOD_MS) {
    g_last_telemetry_ms = now_ms;
    emitTelemetry(g_inference_link.stream());
  }
}