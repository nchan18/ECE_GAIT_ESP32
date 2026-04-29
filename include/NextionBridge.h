// NextionBridge.h — Nextion-style frame parser + host/HMI routing.
//
// Frames are payload bytes terminated by 0xFF 0xFF 0xFF.
//   USB Serial  <-> host PC (dev/debug only; not required at runtime)
//   Serial2     <-> Nextion HMI
//
// CHANGES FROM ORIGINAL:
//   * Does not write to Serial2 directly any more — single-owner pattern
//     means all Serial2 writes go through NextionDisplay.
//   * HMI touch events are routed to NextionController instead of being
//     forwarded to the host Serial.
//   * ESTOP/RESET commands from either side are dispatched to the
//     controller's external-event entry points so the state machine sees
//     them identically to a touch press.
//   * The TENS amplitude CSV path is preserved for the dev-Python case
//     and still drives manual_amps_ as before.
#pragma once

#include "NextionController.h"
#include "NextionDisplay.h"
#include "TensDriver.h"
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

class NextionBridge {
public:
  NextionBridge(NextionController& controller,
                NextionDisplay&    display,
                TensDriver&        tens,
                TensAmplitudes&    manual_amps,
                uint32_t&          manual_amps_stamp_ms);

  void begin(uint32_t host_baud);
  void poll();

  // Optional sink for HMI parse-success/failure stats. NextionController
  // doesn't track parse loss, but Esp32Telemetry does — register it here.
  using ParseObserver = void (*)(bool parsed_ok, void* user);
  void setHmiParseObserver(ParseObserver cb, void* user) {
    hmi_parse_cb_   = cb;
    hmi_parse_user_ = user;
  }

private:
  struct FrameParser {
    uint8_t buf[128];
    size_t  len     = 0;
    int     ffCount = 0;
  };

  void handleHostByte(uint8_t b);
  void handleHmiByte (uint8_t b);
  void feedParser    (FrameParser& p, uint8_t b,
                      void (NextionBridge::*onFrame)(const uint8_t*, size_t));
  void processHostFrame(const uint8_t* data, size_t len);
  void processHmiFrame (const uint8_t* data, size_t len);

  static bool isControlCommand    (const uint8_t* d, size_t n, const char* expected);
  static bool startsWithIgnoreCase(const uint8_t* d, size_t n, const char* prefix);
  bool        tryParseAmplitudes  (const uint8_t* d, size_t n);

  NextionController& controller_;
  NextionDisplay&    display_;
  TensDriver&        tens_;
  TensAmplitudes&    manual_amps_;
  uint32_t&          manual_stamp_ms_;
  FrameParser        host_parser_;
  FrameParser        hmi_parser_;

  ParseObserver hmi_parse_cb_   = nullptr;
  void*         hmi_parse_user_ = nullptr;
};
