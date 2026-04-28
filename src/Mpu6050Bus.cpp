#include "Mpu6050Bus.h"
#include "Config.h"
#include <Arduino.h>
#include <Wire.h>

void Mpu6050Bus::begin() {
  Wire.begin();
  Wire.setClock(400000);
}

void Mpu6050Bus::selectChannel(uint8_t ch) {
  Wire.beginTransmission(cfg::TCA_ADDR);
  Wire.write(1u << ch);
  Wire.endTransmission();
}

bool Mpu6050Bus::wakeChannel(uint8_t ch) {
  selectChannel(ch);
  delay(2);
  Wire.beginTransmission(cfg::MPU_ADDR);
  Wire.write(cfg::MPU_PWR_MGMT_1);
  Wire.write(0x00); // wake, internal 8 MHz osc
  return Wire.endTransmission() == 0;
}

bool Mpu6050Bus::readGyroDps(uint8_t ch, float& gx, float& gy, float& gz) {
  selectChannel(ch);
  Wire.beginTransmission(cfg::MPU_ADDR);
  Wire.write(cfg::MPU_GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)cfg::MPU_ADDR, 6) != 6) return false;

  const int16_t rx = (Wire.read() << 8) | Wire.read();
  const int16_t ry = (Wire.read() << 8) | Wire.read();
  const int16_t rz = (Wire.read() << 8) | Wire.read();
  gx = rx / cfg::GYRO_LSB_PER_DPS;
  gy = ry / cfg::GYRO_LSB_PER_DPS;
  gz = rz / cfg::GYRO_LSB_PER_DPS;
  return true;
}
