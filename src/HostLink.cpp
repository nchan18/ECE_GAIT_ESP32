#include "HostLink.h"

void UartHostLink::begin() {
  Serial.begin(cfg::HOST_BAUD);
}

#if ECE_GAIT_HAS_BT
void BluetoothHostLink::begin() {
  bt_.begin(cfg::BT_DEVICE_NAME);
}

bool BluetoothHostLink::ready() const {
  // BluetoothSerial::hasClient() / connected() are non-const in the SDK;
  // const_cast is safe because they only query state.
  return const_cast<BluetoothSerial&>(bt_).hasClient();
}
#endif

HostLink& makeHostLink() {
  static UartHostLink uart;
#if ECE_GAIT_HAS_BT
  static BluetoothHostLink bt;
  if (cfg::INFERENCE_MODE == cfg::InferenceMode::Bluetooth) {
    return bt;
  }
#endif
  return uart;
}
