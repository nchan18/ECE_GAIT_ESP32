#include "NextionBridge.h"
#include "Config.h"
#include "TensDriver.h"
#include <ctype.h>
#include <string.h>

NextionBridge::NextionBridge(NextionController& controller,
                             NextionDisplay&    display,
                             TensDriver&        tens,
                             TensAmplitudes&    manual_amps,
                             uint32_t&          manual_amps_stamp_ms)
    : controller_(controller),
      display_(display),
      tens_(tens),
      manual_amps_(manual_amps),
      manual_stamp_ms_(manual_amps_stamp_ms) {}

void NextionBridge::begin(uint32_t host_baud) {
  Serial.begin(host_baud);
  // Note: Serial2 is initialized by NextionDisplay::begin() — single owner.
  // We just consume bytes from it here.
}

void NextionBridge::poll() {
  while (Serial.available() > 0) {
    handleHostByte((uint8_t)Serial.read());
  }
  while (Serial2.available() > 0) {
    handleHmiByte((uint8_t)Serial2.read());
  }
}

void NextionBridge::handleHostByte(uint8_t b) {
  feedParser(host_parser_, b, &NextionBridge::processHostFrame);
}

void NextionBridge::handleHmiByte(uint8_t b) {
  feedParser(hmi_parser_, b, &NextionBridge::processHmiFrame);
}

void NextionBridge::feedParser(FrameParser& p, uint8_t b,
                               void (NextionBridge::*onFrame)(const uint8_t*, size_t)) {
  if (b == 0xFF) {
    p.ffCount++;
    if (p.ffCount == 3) {
      (this->*onFrame)(p.buf, p.len);
      p.len     = 0;
      p.ffCount = 0;
    }
    return;
  }
  while (p.ffCount > 0) {
    if (p.len < sizeof(p.buf)) p.buf[p.len++] = 0xFF;
    p.ffCount--;
  }
  if (p.len < sizeof(p.buf)) p.buf[p.len++] = b;
}

bool NextionBridge::isControlCommand(const uint8_t* d, size_t len, const char* expected) {
  const size_t n = strlen(expected);
  if (len != n) return false;
  for (size_t i = 0; i < n; ++i) {
    if (toupper((unsigned char)d[i]) != expected[i]) return false;
  }
  return true;
}

bool NextionBridge::startsWithIgnoreCase(const uint8_t* d, size_t len, const char* prefix) {
  const size_t n = strlen(prefix);
  if (len < n) return false;
  for (size_t i = 0; i < n; ++i) {
    if (toupper((unsigned char)d[i]) != prefix[i]) return false;
  }
  return true;
}

bool NextionBridge::tryParseAmplitudes(const uint8_t* d, size_t len) {
  // Same parser as the original — accepts "Q,H,A,C" CSV from host UART
  // and stores into the shared manual_amps_ buffer with a timestamp. The
  // override window logic lives in main.cpp.
  int commas = 0;
  for (size_t i = 0; i < len; ++i) {
    const char c = (char)d[i];
    if (c == ',') commas++;
    else if (!(isdigit((unsigned char)c) || c == ' ' || c == '-' || c == '\r')) {
      return false;
    }
  }
  if (commas < 3) return false;

  char tmp[64];
  const size_t copyLen = len < sizeof(tmp) - 1 ? len : sizeof(tmp) - 1;
  memcpy(tmp, d, copyLen);
  tmp[copyLen] = '\0';

  String s(tmp);
  const int c1 = s.indexOf(',');
  const int c2 = s.indexOf(',', c1 + 1);
  const int c3 = s.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return false;

  manual_amps_.quadAmp      = s.substring(0,      c1).toInt();
  manual_amps_.hamstringAmp = s.substring(c1 + 1, c2).toInt();
  manual_amps_.antTibAmp    = s.substring(c2 + 1, c3).toInt();
  manual_amps_.calfAmp      = s.substring(c3 + 1    ).toInt();
  manual_stamp_ms_       = millis();
  return true;
}

void NextionBridge::processHostFrame(const uint8_t* d, size_t len) {
  // Host UART is dev-only — it can still send ESTOP/RESET to drive the
  // controller, and TENS amplitude CSVs for manual override.
  if (isControlCommand(d, len, "ESTOP")) {
    controller_.onExternalEstop();
    return;
  }
  if (isControlCommand(d, len, "RESET")) {
    controller_.onExternalReset();
    return;
  }

  if (tryParseAmplitudes(d, len)) return;

  // No more passthrough to Serial2. The single-owner pattern means dev
  // Python cannot directly script the Nextion via the host UART — that is
  // intentional. If you need this for development, add a debug method on
  // NextionDisplay rather than re-opening direct Serial2 writes.
}

void NextionBridge::processHmiFrame(const uint8_t* d, size_t len) {
  // Decide whether this frame parsed as something we recognize. Parse loss
  // observer (page1's packetLoss field) is informed both ways.
  bool parsed_ok = false;

  // Recognized payload types:
  //   1. 0x65 touch event packet (4 bytes minimum)
  //   1b. 0x66 current-page report packet (page changes)
  //   2. ESTOP/RESET text commands
  //   3. Single-byte status responses (0x00, 0x01, 0x1A) — silently
  //      acknowledged as parsed-OK
  if (len >= 4 && d[0] == 0x65) {
    parsed_ok = true;
    controller_.onHmiFrame(d, len);
  } else if (len == 2 && d[0] == 0x66) {
    parsed_ok = true;
  } else if (isControlCommand(d, len, "ESTOP")) {
    parsed_ok = true;
    controller_.onExternalEstop();
  } else if (isControlCommand(d, len, "RESET")) {
    parsed_ok = true;
    controller_.onExternalReset();
  } else if (len == 1 && (d[0] == 0x00 || d[0] == 0x01 || d[0] == 0x1A)) {
    parsed_ok = true;
    // Status code; nothing further to do.
  }

  if (hmi_parse_cb_) hmi_parse_cb_(parsed_ok, hmi_parse_user_);
}
