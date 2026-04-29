#include "NextionController.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

NextionController::NextionController(NextionDisplay& display)
    : display_(display) {}

void NextionController::begin() {
  state_              = hmi::SystemState::Standby;
  current_page_index_ = 0;
  blink_phase_        = false;
  last_blink_ms_      = millis();
  blink_hold_until_ms_ = millis() + hmi::PAGE_SETTLE_MS;

  // Initial UI sync to Standby state, EN button to idle style.
  refreshUiForCurrentState();
}

void NextionController::transitionTo(hmi::SystemState new_state) {
  if (new_state == state_) return;
  const hmi::SystemState old = state_;
  state_ = new_state;
  refreshUiForCurrentState();
  // Reset blink phase so the user sees an immediate visual cue.
  blink_phase_   = true;
  last_blink_ms_ = millis();
  blink_hold_until_ms_ = last_blink_ms_ + hmi::PAGE_SETTLE_MS;

  if (state_cb_) state_cb_(old, new_state, state_user_);
}

void NextionController::refreshUiForCurrentState() {
  display_.syncStatusUi(state_);
  display_.applyEnButtonStyle(hmi::enStyleForState(state_));
}

void NextionController::tick() {
  const uint32_t now = millis();

  updateBlink(now);

  // Deferred page-settle resync.
  if (pending_sync_at_ms_ != 0 && (int32_t)(now - pending_sync_at_ms_) >= 0) {
    refreshUiForCurrentState();
    blink_phase_         = true;
    last_blink_ms_       = now;
    blink_hold_until_ms_ = now + hmi::PAGE_SETTLE_MS;
    pending_sync_at_ms_  = 0;
  }
}

void NextionController::updateBlink(uint32_t now) {
  // Brief freeze after state/page changes — give the display a moment to
  // settle before we start cycling the LED image again.
  if ((int32_t)(now - blink_hold_until_ms_) < 0) return;

  const uint32_t period = hmi::blinkPeriodMs(state_);
  if (period == 0) return;  // safety: never divide-by-zero or spin

  if (now - last_blink_ms_ >= period) {
    const int lit_pic   = hmi::statusPicForState(state_);
    const int blank_pic = (current_page_index_ >= 0 &&
                            current_page_index_ < hmi::NUM_PAGES)
                          ? hmi::PAGES[current_page_index_].status_blank_pic
                          : hmi::STATUS_PIC_BLANK;
    display_.setStatusImage(blink_phase_ ? lit_pic : blank_pic);
    blink_phase_   = !blink_phase_;
    last_blink_ms_ = now;
  }
}

bool NextionController::isEnComponentOnCurrentPage(uint8_t component_id) const {
  if (current_page_index_ < 0 || current_page_index_ >= hmi::NUM_PAGES) {
    return false;
  }
  return component_id == hmi::PAGES[current_page_index_].en_component_id;
}

void NextionController::onHmiFrame(const uint8_t* data, size_t len) {
  // Nextion touch event: 0x65 <page> <component> <event>.
  // Only this packet type is interesting for state transitions; other
  // payloads (status codes, print output) are ignored at this layer.
  if (len < 4 || data[0] != 0x65) return;
  handleTouchEvent(data[1], data[2], data[3]);
}

void NextionController::handleTouchEvent(uint8_t page_id, uint8_t component_id,
                                         uint8_t event) {
  // Update logical page index from the touch packet's page byte. The byte
  // is the Nextion's numeric page ID, which we look up in our PAGES[] table.
  for (int i = 0; i < hmi::NUM_PAGES; ++i) {
    if (hmi::PAGES[i].nextion_id == page_id) {
      current_page_index_ = i;
      break;
    }
  }

  // Fire the external callback — this drives Esp32Telemetry's lastCommand
  // and parse-loss accounting. Always invoked, even for ignored events.
  if (touch_cb_) touch_cb_(page_id, component_id, event, touch_user_);

  // Only press events trigger transitions; release events are noise.
  if (event != hmi::TOUCH_EVENT_PRESS) return;

  // ESTOP wins over everything else. Always allowed, regardless of state.
  if (component_id == hmi::ESTOP_COMPONENT_ID) {
    if (state_ != hmi::SystemState::Error) {
      transitionTo(hmi::SystemState::Error);
    }
    return;
  }

  // EN button: behavior depends on current state.
  if (isEnComponentOnCurrentPage(component_id)) {
    handleEnPress();
    return;
  }

  // Page navigation: only meaningful when not in Error.
  if (state_ != hmi::SystemState::Error) {
    if (component_id == hmi::NEXT_PAGE_COMPONENT_ID) {
      handlePageNav(+1);
    } else if (component_id == hmi::PREV_PAGE_COMPONENT_ID) {
      handlePageNav(-1);
    }
  }
}

void NextionController::handleEnPress() {
  switch (state_) {
    case hmi::SystemState::Standby:
      transitionTo(hmi::SystemState::Active);
      break;
    case hmi::SystemState::Active:
      transitionTo(hmi::SystemState::Standby);
      break;
    case hmi::SystemState::Error:
      // Clearing ESTOP returns to Standby (not Active — user must
      // deliberately re-enable after an emergency stop).
      transitionTo(hmi::SystemState::Standby);
      break;
  }
}

void NextionController::handlePageNav(int direction) {
  const int n = hmi::NUM_PAGES;
  int next = current_page_index_ + direction;
  if (hmi::WRAP_PAGES) {
    next = ((next % n) + n) % n;
  } else {
    if (next < 0)  next = 0;
    if (next >= n) next = n - 1;
  }
  current_page_index_ = next;
  display_.goToPage(hmi::PAGES[next].name);

  // Re-assert status objects after the page settles. The Nextion takes a
  // brief moment to render the new page; writing object commands during
  // that window can land partially.
  pending_sync_at_ms_ = millis() + hmi::PAGE_SETTLE_MS;
}

void NextionController::onExternalEstop() {
  // Identical effect to a physical eStop press. Useful for the dev-Python
  // path that may still send "ESTOP" over the host UART.
  if (state_ != hmi::SystemState::Error) {
    transitionTo(hmi::SystemState::Error);
  }
}

void NextionController::onExternalReset() {
  // Mirrors "press EN while in Error" — clears latch back to Standby.
  if (state_ == hmi::SystemState::Error) {
    transitionTo(hmi::SystemState::Standby);
  }
}

// ─── Telemetry routing ─────────────────────────────────────────────────────

void NextionController::publishTelemetryText(const char* key, const char* value) {
  const auto* b = hmi::findBinding(key);
  if (!b) return;
  if (b->target_page_idx != current_page_index_) return;
  if (b->mode == hmi::BindingMode::Txt) {
    display_.setObjectText(b->nextion_object, value);
  } else {
    // Caller passed a string for a Val-mode binding. Convert; if non-numeric,
    // skip silently — same fail-soft behavior as Python's set_telemetry_value.
    char* end = nullptr;
    long v = strtol(value, &end, 10);
    if (end != value) display_.setObjectVal(b->nextion_object, (int32_t)v);
  }
}

void NextionController::publishTelemetryVal(const char* key, int32_t value) {
  const auto* b = hmi::findBinding(key);
  if (!b) return;
  if (b->target_page_idx != current_page_index_) return;
  if (b->mode == hmi::BindingMode::Val) {
    display_.setObjectVal(b->nextion_object, value);
  } else {
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)value);
    display_.setObjectText(b->nextion_object, buf);
  }
}

void NextionController::publishTelemetryTextForce(const char* key, const char* value) {
  // Used for the boot-once refresh on page navigation. Skips the
  // active-page check — caller has already verified the user is looking at
  // the relevant page.
  const auto* b = hmi::findBinding(key);
  if (!b) return;
  if (b->mode == hmi::BindingMode::Txt) {
    display_.setObjectText(b->nextion_object, value);
  } else {
    char* end = nullptr;
    long v = strtol(value, &end, 10);
    if (end != value) display_.setObjectVal(b->nextion_object, (int32_t)v);
  }
}
