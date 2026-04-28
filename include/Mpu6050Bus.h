// Mpu6050Bus.h — TCA9548A multiplexer + MPU6050 gyro reader.
#pragma once

#include <stdint.h>

class Mpu6050Bus {
public:
  void begin();
  bool wakeChannel(uint8_t channel);                          // selects + wakes
  bool readGyroDps(uint8_t channel, float& gx, float& gy, float& gz);

private:
  void selectChannel(uint8_t channel);
};
