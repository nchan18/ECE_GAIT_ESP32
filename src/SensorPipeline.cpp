#include "SensorPipeline.h"
#include <Arduino.h>

void SensorPipeline::begin() {
  emg_.begin(cfg::EMG_FS);
  for (int i = 0; i < cfg::NUM_IMU; ++i) imu_[i].begin(cfg::IMU_FS);

  for (int i = 0; i < cfg::NUM_EMG; ++i) pinMode(cfg::EMG_PINS[i], INPUT);

  mpu_.begin();
  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    if (!mpu_.wakeChannel(i)) {
      Serial.print("MPU6050 init failed on mux ch ");
      Serial.println(i);
    }
  }

  emg_period_us_ = (uint32_t)(1e6f / cfg::EMG_FS);
  imu_period_us_ = (uint32_t)(1e6f / cfg::IMU_FS);
  const uint32_t now = micros();
  next_emg_us_ = now + emg_period_us_;
  next_imu_us_ = now + imu_period_us_;
  last_imu_us_ = now;
}

void SensorPipeline::sampleEmgTick() {
  float raw[cfg::NUM_EMG];
  for (int i = 0; i < cfg::NUM_EMG; ++i) {
    // ESP32 ADC is 12-bit (0..4095) — centre to mid-rail.
    const int v = analogRead(cfg::EMG_PINS[i]);
    raw[i] = (float)v - 2048.0f;
  }
  emg_.step(raw);

  latest_amps_.quadAmp      = emg_.amplitudeForMuscle(cfg::SELECTED_LEG, 0);
  latest_amps_.hamstringAmp = emg_.amplitudeForMuscle(cfg::SELECTED_LEG, 1);
  latest_amps_.calfAmp      = emg_.amplitudeForMuscle(cfg::SELECTED_LEG, 2);
  latest_amps_.antTibAmp    = 0; // No EMG sensor mapped to AntTib.
  fresh_amps_ = true;
}

void SensorPipeline::sampleImuTick(uint32_t now_us) {
  const float dt_raw = (now_us - last_imu_us_) * 1e-6f;
  const float dt = dt_raw > 0.f ? dt_raw : 1.0f / cfg::IMU_FS;
  last_imu_us_ = now_us;

  for (int i = 0; i < cfg::NUM_IMU; ++i) {
    float gx, gy, gz;
    if (mpu_.readGyroDps(i, gx, gy, gz)) {
      imu_[i].step(gx, gy, gz, dt);
    }
  }
}

void SensorPipeline::update() {
  const uint32_t now_us = micros();
  if ((int32_t)(now_us - next_emg_us_) >= 0) {
    next_emg_us_ += emg_period_us_;
    sampleEmgTick();
  }
  if ((int32_t)(now_us - next_imu_us_) >= 0) {
    next_imu_us_ += imu_period_us_;
    sampleImuTick(now_us);
  }
}

TensAmplitudes SensorPipeline::consumeAmplitudes() {
  fresh_amps_ = false;
  return latest_amps_;
}
