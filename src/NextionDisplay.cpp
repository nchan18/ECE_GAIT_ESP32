#include "NextionDisplay.h"
#include <stdio.h>

void NextionDisplay::begin(uint32_t baud, int rx_pin, int tx_pin) {
  Serial2.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
}

void NextionDisplay::sendFrame(const char* cmd) {
  // Payload, then exactly three 0xFF terminators. No partial writes possible
  // because we're single-threaded and this is the only writer.
  Serial2.print(cmd);
  Serial2.write((uint8_t)0xFF);
  Serial2.write((uint8_t)0xFF);
  Serial2.write((uint8_t)0xFF);
}

void NextionDisplay::setStatusImage(int pic_id) {
  char buf[32];
  snprintf(buf, sizeof(buf), "statusLED.pic=%d", pic_id);
  sendFrame(buf);
}

void NextionDisplay::setStatusText(hmi::SystemState s) {
  // Three writes per Python: pco, borderc, txt. Order matches controller.py
  // so a partial mid-update still leaves the overlay coherent (color set
  // before text avoids a brief flash of old-color new-text).
  const uint16_t color = hmi::statusColorForState(s);
  const char*    label = hmi::statusLabelForState(s);

  char buf[64];
  snprintf(buf, sizeof(buf), "statusTxt.pco=%u",      (unsigned)color);
  sendFrame(buf);
  snprintf(buf, sizeof(buf), "statusTxt.borderc=%u",  (unsigned)color);
  sendFrame(buf);
  snprintf(buf, sizeof(buf), "statusTxt.txt=\"%s\"",  label);
  sendFrame(buf);
}

void NextionDisplay::setOverlayVisible(bool visible) {
  char buf[24];
  snprintf(buf, sizeof(buf), "vis statusTxt,%d", visible ? 1 : 0);
  sendFrame(buf);
}

void NextionDisplay::syncStatusUi(hmi::SystemState s) {
  // Hide → update → show, same sequence as Python's sync_status_ui().
  // Avoids a brief frame where the overlay shows stale text in new color.
  setOverlayVisible(false);
  setStatusText(s);
  setStatusImage(hmi::statusPicForState(s));
  setOverlayVisible(true);
}

void NextionDisplay::goToPage(const char* page_name) {
  char buf[32];
  snprintf(buf, sizeof(buf), "page %s", page_name);
  sendFrame(buf);
}

void NextionDisplay::applyEnButtonStyle(const hmi::EnButtonStyle& style) {
  // Same 5-write sequence as the Python's _set_enable_button_style.
  // Object name "EN" is consistent across all pages even though the
  // component ID differs — we write to "EN" by name, not by id.
  char buf[64];
  snprintf(buf, sizeof(buf), "EN.txt=\"%s\"", style.txt);   sendFrame(buf);
  snprintf(buf, sizeof(buf), "EN.pco=%u",  (unsigned)style.pco);   sendFrame(buf);
  snprintf(buf, sizeof(buf), "EN.pco2=%u", (unsigned)style.pco2);  sendFrame(buf);
  snprintf(buf, sizeof(buf), "EN.bco=%u",  (unsigned)style.bco);   sendFrame(buf);
  snprintf(buf, sizeof(buf), "EN.bco2=%u", (unsigned)style.bco2);  sendFrame(buf);
}

void NextionDisplay::setObjectText(const char* object, const char* value) {
  // Strip embedded quotes — Nextion has no escape mechanism for " inside
  // quoted strings. Same defensive measure as the Python does.
  char escaped[96];
  size_t out = 0;
  for (size_t i = 0; value[i] != '\0' && out < sizeof(escaped) - 1; ++i) {
    if (value[i] == '"') continue;
    escaped[out++] = value[i];
  }
  escaped[out] = '\0';

  char buf[160];
  snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", object, escaped);
  sendFrame(buf);
}

void NextionDisplay::setObjectVal(const char* object, int32_t value) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s.val=%ld", object, (long)value);
  sendFrame(buf);
}
