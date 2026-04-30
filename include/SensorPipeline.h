// SensorPipeline.h — owns EMG + IMU arrays, schedules sampling, and
// produces TensAmplitudes derived from the selected leg.
#pragma once

#include "Config.h"
#include "EmgChannel.h"
#include "ImuChannel.h"
#include "Mpu6050Bus.h"
#include "TensDriver.h"
#include <stdint.h>

class SensorPipeline {
public:
  void begin();
  void update();   // call every loop tick

  bool hasFreshAmplitudes() const { return fresh_amps_; }
  TensAmplitudes consumeAmplitudes();   // clears fresh flag

  // Telemetry accessors.
  const EmgArray& emg() const { return emg_; }
  const ImuChannel& imu(int i) const { return imu_[i]; }

private:
  void sampleEmgTick();
  void sampleImuTick(uint32_t now_us);

  EmgArray   emg_;
  ImuChannel imu_[cfg::NUM_IMU];
  Mpu6050Bus mpu_;
  bool       imu_present_[cfg::NUM_IMU] = {false};

  uint32_t emg_period_us_ = 0;
  uint32_t imu_period_us_ = 0;
  uint32_t next_emg_us_   = 0;
  uint32_t next_imu_us_   = 0;
  uint32_t last_imu_us_   = 0;

  TensAmplitudes latest_amps_;
  bool           fresh_amps_ = false;
};
