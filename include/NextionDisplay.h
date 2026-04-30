// NextionDisplay.h — single owner of all Serial2 writes to the Nextion HMI.
//
// The Nextion protocol uses 0xFF 0xFF 0xFF as a frame terminator. If two
// callers interleave bytes onto Serial2, the terminator from one frame can
// land inside another and the display rejects both. This class is the only
// place that should call Serial2.write(); everything else asks it to send.
//
// Methods are intentionally narrow — one method per kind of command — so the
// terminator is always emitted atomically with the payload.
#pragma once

#include "HmiConfig.h"
#include <Arduino.h>
#include <stdint.h>

class NextionDisplay {
public:
  void begin(uint32_t baud, int rx_pin, int tx_pin);

  // ── Status overlay (universal: present on every page) ────────────────────
  void setStatusImage(int pic_id);                   // statusLED.pic=<id>
  void setStatusText(hmi::SystemState s);            // 3-write text+color sync
  void setOverlayVisible(bool visible);              // vis statusTxt,<0|1>
  void syncStatusUi(hmi::SystemState s);             // overlay off → text → image → overlay on

  // ── Page navigation ──────────────────────────────────────────────────────
  void goToPage(const char* page_name);              // page <name>

  // ── EN button (per-page object, but always named "EN") ───────────────────
  void applyEnButtonStyle(const hmi::EnButtonStyle& style);  // 5 writes (txt, pco, pco2, bco, bco2)

  // ── Generic telemetry value writes ───────────────────────────────────────
  void setObjectText(const char* object, const char* value);  // <obj>.txt="<value>"
  void setObjectVal (const char* object, int32_t     value);  // <obj>.val=<int>

private:
  // Low-level: write one Nextion command frame (payload + 0xFF 0xFF 0xFF).
  // Single chokepoint so terminator emission is impossible to forget.
  void sendFrame(const char* cmd);
};
