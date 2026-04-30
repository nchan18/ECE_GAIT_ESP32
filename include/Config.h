// Config.h — global pin map, sampling rates, and sensor mapping.
#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------------------
// TENS output pins (PWM-capable).
// ---------------------------------------------------------------------------
constexpr int CS_CALF      = 32;
constexpr int CS_ANTTIB    = 33;
constexpr int CS_HAMSTRING = 26;
constexpr int CS_QUAD      = 25;
constexpr int ESTOP    = 14;

// ---------------------------------------------------------------------------
// Nextion HMI bridge (Serial2).
// ---------------------------------------------------------------------------
constexpr uint32_t HMI_BAUD       = 9600;
constexpr int      NEXTION_RX_PIN = 16;
constexpr int      NEXTION_TX_PIN = 17;

// ---------------------------------------------------------------------------
// Sensor counts and analog pins.
// NOTE: pins overlap with CS_CALF/CS_ANTTIB on this default map; adjust to
// match the actual hardware before enabling EMG on those channels.
// ---------------------------------------------------------------------------
constexpr int NUM_EMG = 3;
constexpr int NUM_IMU = 6;

constexpr uint8_t EMG_PINS[NUM_EMG] = {34, 35, 36};

// ---------------------------------------------------------------------------
// I²C addresses.
// ---------------------------------------------------------------------------
constexpr uint8_t TCA_ADDR        = 0x70;
constexpr uint8_t MPU_ADDR        = 0x68;
constexpr uint8_t MPU_PWR_MGMT_1  = 0x6B;
constexpr uint8_t MPU_GYRO_XOUT_H = 0x43;
constexpr float   GYRO_LSB_PER_DPS = 131.0f;

// ---------------------------------------------------------------------------
// Sampling rates (Hz).
// ---------------------------------------------------------------------------
constexpr float EMG_FS = 1000.0f;
constexpr float IMU_FS = 150.0f;

// Active leg whose EMG drives the TENS outputs ('L' or 'R').
constexpr char SELECTED_LEG = 'R';

// Manual host-CSV override duration (ms). After this, sensors regain control.
constexpr uint32_t MANUAL_OVERRIDE_MS = 500;

// ---------------------------------------------------------------------------
// Host link selection.
//   InferenceMode::Uart      — host PC over USB-serial (Serial @ HOST_BAUD).
//   InferenceMode::Bluetooth — host PC over Bluetooth Classic SPP.
// Switch by changing INFERENCE_MODE below.
// ---------------------------------------------------------------------------
enum class InferenceMode : uint8_t { Uart, Bluetooth };
constexpr InferenceMode INFERENCE_MODE   = InferenceMode::Uart;
constexpr uint32_t      HOST_BAUD        = 9600;
constexpr const char*   BT_DEVICE_NAME   = "ECE_GAIT";
constexpr uint32_t      TELEMETRY_PERIOD_MS = 100;

// Strict host-in-the-loop watchdog: if no TENS command arrives from the host
// within this window, all TENS outputs are forced to 0.  There is no on-board
// sensor fallback — the host is the sole authority on amplitudes.
constexpr uint32_t      HOST_COMMAND_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// Sensor index → (leg, muscle).  muscle: 0 = quad, 1 = hamstring, 2 = calf.
// ---------------------------------------------------------------------------
struct SensorMap { char leg; uint8_t muscle; };
constexpr SensorMap SENSOR_MUSCLE_MAP[NUM_EMG] = {
  {'R', 2}, // 0 – Right calf
  // {'L', 1}, // 1 – Left  hamstring
  // {'L', 0}, // 2 – Left  quad
  // {'L', 2}, // 3 – Left  calf
  {'R', 1}, // 4 – Right hamstring
  {'R', 0}, // 5 – Right quad
};

} // namespace cfg
