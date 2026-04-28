#include "NextionBridge.h"
#include "Config.h"
#include <ctype.h>
#include <string.h>

NextionBridge::NextionBridge(TensDriver& tens, TensAmplitudes& manual_amps,
                             uint32_t& manual_amps_stamp_ms)
    : tens_(tens),
      manual_amps_(manual_amps),
      manual_stamp_ms_(manual_amps_stamp_ms) {}

void NextionBridge::begin() {
  Serial.begin(cfg::HMI_BAUD);
  Serial2.begin(cfg::HMI_BAUD, SERIAL_8N1, cfg::NEXTION_RX_PIN, cfg::NEXTION_TX_PIN);
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

bool NextionBridge::isAllowedUiCommandWhileLatched(const uint8_t* d, size_t n) {
  return startsWithIgnoreCase(d, n, "STATUSLED.PIC=")
      || startsWithIgnoreCase(d, n, "STATUSTXT.TXT=")
      || startsWithIgnoreCase(d, n, "STATUSTXT.PCO=")
      || startsWithIgnoreCase(d, n, "STATUSTXT.BORDERC=")
      || startsWithIgnoreCase(d, n, "VIS STATUSTXT,");
}

bool NextionBridge::tryParseAmplitudes(const uint8_t* d, size_t len) {
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
  if (isControlCommand(d, len, "ESTOP")) { tens_.enterEstop("host"); return; }
  if (isControlCommand(d, len, "RESET")) { tens_.clearEstop("host"); return; }

  if (tens_.estopLatched()) {
    if (isAllowedUiCommandWhileLatched(d, len)) {
      Serial2.write(d, len);
      Serial2.write(0xFF); Serial2.write(0xFF); Serial2.write(0xFF);
    }
    return;
  }

  if (tryParseAmplitudes(d, len)) return;

  // Otherwise pass through to the Nextion HMI.
  Serial2.write(d, len);
  Serial2.write(0xFF); Serial2.write(0xFF); Serial2.write(0xFF);
}

void NextionBridge::processHmiFrame(const uint8_t* d, size_t len) {
  if (isControlCommand(d, len, "ESTOP")) { tens_.enterEstop("hmi"); return; }
  if (isControlCommand(d, len, "RESET")) { tens_.clearEstop("hmi"); return; }

  // Forward everything else up to the host.
  Serial.write(d, len);
  Serial.write(0xFF); Serial.write(0xFF); Serial.write(0xFF);
}
