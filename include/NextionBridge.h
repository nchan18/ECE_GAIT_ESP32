// NextionBridge.h — Nextion-style frame parser + host/HMI command dispatch.
//
// Frames are payload bytes terminated by 0xFF 0xFF 0xFF.
// USB Serial  <-> host PC
// Serial2     <-> Nextion HMI
#pragma once

#include "TensDriver.h"
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

class NextionBridge {
public:
  NextionBridge(TensDriver& tens, TensAmplitudes& manual_amps,
                uint32_t& manual_amps_stamp_ms);

  void begin();
  void poll();   // call every loop tick

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

  static bool isControlCommand            (const uint8_t* d, size_t n, const char* expected);
  static bool startsWithIgnoreCase        (const uint8_t* d, size_t n, const char* prefix);
  static bool isAllowedUiCommandWhileLatched(const uint8_t* d, size_t n);
  bool        tryParseAmplitudes          (const uint8_t* d, size_t n);

  TensDriver&     tens_;
  TensAmplitudes& manual_amps_;
  uint32_t&       manual_stamp_ms_;
  FrameParser     host_parser_;
  FrameParser     hmi_parser_;
};
