// HostLink.h — abstract transport for host PC communication.
//
// Two implementations are provided:
//   * UartHostLink      — USB-serial (HardwareSerial / `Serial`).
//   * BluetoothHostLink — Bluetooth Classic SPP (BluetoothSerial).
//
// Both expose a `Stream&` so the rest of the firmware can read commands and
// write telemetry/processed-EMG without caring about the underlying medium.
// The active transport is chosen by `cfg::INFERENCE_MODE` and built by
// `makeHostLink()`.
#pragma once

#include "Config.h"
#include <Arduino.h>

#if defined(__has_include)
#  if __has_include(<BluetoothSerial.h>)
#    include <BluetoothSerial.h>
#    define ECE_GAIT_HAS_BT 1
#  else
#    define ECE_GAIT_HAS_BT 0
#  endif
#else
#  include <BluetoothSerial.h>
#  define ECE_GAIT_HAS_BT 1
#endif

class HostLink {
public:
  virtual ~HostLink() = default;
  virtual void    begin()  = 0;
  virtual Stream& stream() = 0;
  virtual bool    ready()  const = 0;       // e.g. BT client connected
  virtual cfg::InferenceMode mode() const = 0;
};

class UartHostLink : public HostLink {
public:
  void    begin() override;
  Stream& stream() override { return Serial; }
  bool    ready() const override { return true; }
  cfg::InferenceMode mode() const override { return cfg::InferenceMode::Uart; }
};

#if ECE_GAIT_HAS_BT
class BluetoothHostLink : public HostLink {
public:
  void    begin() override;
  Stream& stream() override { return bt_; }
  bool    ready() const override;
  cfg::InferenceMode mode() const override { return cfg::InferenceMode::Bluetooth; }

private:
  BluetoothSerial bt_;
};
#endif

// Factory: returns a singleton HostLink for the configured mode.  Falls back
// to UART if the requested mode is Bluetooth but BT is unavailable.
HostLink& makeHostLink();
