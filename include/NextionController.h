// NextionController.h — port of controller.py.
//
// Owns the system state machine, the status LED blink scheduling, page
// navigation, the EN button state machine, and routing of touch events to
// state transitions. Talks to the display only through NextionDisplay
// (single-owner pattern for Serial2).
//
// Driven by tick() from main loop(). No blocking calls.
#pragma once

#include "HmiConfig.h"
#include "NextionDisplay.h"
#include <stdint.h>
#include <stddef.h>

class NextionController {
public:
  // Callback invoked for every touch event we receive. Used by the rest of
  // the firmware (Esp32Telemetry's lastCommand field, mainly) without
  // coupling NextionController to those subsystems.
  using TouchCallback = void (*)(uint8_t page_id, uint8_t component_id,
                                  uint8_t event, void* user);

  // Callback invoked whenever the system state changes. Used by TensDriver
  // to gate output and by Esp32Telemetry to update lastCommand.
  using StateChangeCallback = void (*)(hmi::SystemState old_state,
                                       hmi::SystemState new_state,
                                       void* user);

  explicit NextionController(NextionDisplay& display);

  void begin();    // initial UI sync, EN button to idle style
  void tick();     // call every loop iteration

  // Inject HMI bytes parsed from Serial2 by NextionBridge.
  // We do not touch Serial2 for reading directly — bridge owns that side.
  void onHmiFrame(const uint8_t* data, size_t len);

  // ── Public state queries ─────────────────────────────────────────────────
  hmi::SystemState state() const { return state_; }
  int  currentPageIndex() const  { return current_page_index_; }

  // Returns a stable pointer to the internal state member, for subsystems
  // (TensDriver) that need to read it without going through a getter on
  // every check.
  const hmi::SystemState* statePointer() const { return &state_; }

  // ── Telemetry pushes (called by Esp32Telemetry, GaitMetrics) ─────────────
  // Routes through TELEMETRY_BINDINGS; silently skips if no binding exists
  // or the binding's target page isn't currently active.
  void publishTelemetryText(const char* key, const char* value);
  void publishTelemetryVal (const char* key, int32_t     value);

  // Always-push variants (ignore active-page check). Used by the boot-once
  // refresh path so e.g. the firmware version lands as soon as page1 opens
  // even if the binding's target_page_idx happens to equal current page.
  void publishTelemetryTextForce(const char* key, const char* value);

  // ── Callback registration ────────────────────────────────────────────────
  void setTouchCallback      (TouchCallback cb,      void* user) { touch_cb_ = cb; touch_user_ = user; }
  void setStateChangeCallback(StateChangeCallback cb, void* user) { state_cb_ = cb; state_user_ = user; }

  // Called by NextionBridge when an external ESTOP/RESET command arrives
  // from the host UART (dev-Python case). Mirrors what an EN/eStop press
  // would have done.
  void onExternalEstop();
  void onExternalReset();

private:
  // Touch event dispatch
  void handleTouchEvent(uint8_t page_id, uint8_t component_id, uint8_t event);

  // State transitions — single chokepoint, fires the callback.
  void transitionTo(hmi::SystemState new_state);

  // Per-page UI updates (status overlay + EN button style).
  void refreshUiForCurrentState();

  // Blink LED scheduler — called every tick.
  void updateBlink(uint32_t now_ms);

  // EN button press handling — varies by current state.
  void handleEnPress();

  // Page change with settle-time deferred re-sync.
  void handlePageNav(int direction);

  // True if the given component ID is the EN button on the current page.
  bool isEnComponentOnCurrentPage(uint8_t component_id) const;

  // ── Members ──────────────────────────────────────────────────────────────
  NextionDisplay& display_;

  hmi::SystemState state_                = hmi::SystemState::Standby;
  int              current_page_index_    = 0;

  // Blink machine
  bool     blink_phase_         = false;     // true = "lit", false = "blank"
  uint32_t last_blink_ms_       = 0;
  uint32_t blink_hold_until_ms_ = 0;         // brief freeze after a state/page change

  // Page-settle deferred resync (the "pending_sync_deadline" in Python).
  // 0 means no pending sync.
  uint32_t pending_sync_at_ms_  = 0;

  // Callbacks
  TouchCallback        touch_cb_   = nullptr;
  void*                touch_user_ = nullptr;
  StateChangeCallback  state_cb_   = nullptr;
  void*                state_user_ = nullptr;
};
