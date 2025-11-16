
#include "imu.hpp"


IMU::IMU() : accelX(0), accelY(0), accelZ(0) {
}


bool IMU::begin() {
  Wire1.begin();
  Wire1.setClock(400000);  // Fast mode

  uint8_t whoAmI = readRegister(LSM9DS1_AG_ADDR, WHO_AM_I);
  if (whoAmI != WHO_AM_I_VAL) {
    Serial.println("ERROR: LSM9DS1 AG not found on Wire1!");
    Serial.print("WHO_AM_I returned: 0x");
    Serial.println(whoAmI, HEX);
    return false;
  }

  // Configure Accelerometer
  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG5_XL, 0x38); // Enable all axes
  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, 0x60); // ODR 119 Hz, ±2g
  writeRegister(LSM9DS1_AG_ADDR, CTRL_REG7_XL, 0x00); // High-res mode

  delay(10);

  Serial.println("LSM9DS1 initialized successfully on Wire1");
  return true;
}


void IMU::readAcceleration() {
  uint8_t rawData[6];
  readRegisters(LSM9DS1_AG_ADDR, OUT_X_L_XL, rawData, 6);

  accelX = (int16_t)((rawData[1] << 8) | rawData[0]);
  accelY = (int16_t)((rawData[3] << 8) | rawData[2]);
  accelZ = (int16_t)((rawData[5] << 8) | rawData[4]);
}


int16_t IMU::getRawAccelX() { return accelX; }
int16_t IMU::getRawAccelY() { return accelY; }
int16_t IMU::getRawAccelZ() { return accelZ; }


//Convert raw to milli-g
int32_t IMU::getAccelX_mg() { return rawToMilliG(accelX); }
int32_t IMU::getAccelY_mg() { return rawToMilliG(accelY); }
int32_t IMU::getAccelZ_mg() { return rawToMilliG(accelZ); }

//Magnitude |a| = sqrt(x^2 + y^2 + z^2)
uint32_t IMU::getAccelMagnitude_mg() {
  int32_t x = rawToMilliG(accelX);
  int32_t y = rawToMilliG(accelY);
  int32_t z = rawToMilliG(accelZ);

  uint32_t sum = (uint32_t)x * x + (uint32_t)y * y + (uint32_t)z * z;

  return integerSqrt(sum);
}


void IMU::printAcceleration() {
  Serial.print("Accel X: "); Serial.print(getAccelX_mg());
  Serial.print(" mg, Y: "); Serial.print(getAccelY_mg());
  Serial.print(" mg, Z: "); Serial.print(getAccelZ_mg());
  Serial.println(" mg");
}


bool IMU::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  return (Wire1.endTransmission() == 0);
}


uint8_t IMU::readRegister(uint8_t address, uint8_t reg) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.endTransmission(false);

  Wire1.requestFrom(address, (uint8_t)1);
  if (Wire1.available())
    return Wire1.read();

  return 0;
}


uint8_t IMU::readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.endTransmission(false);

  Wire1.requestFrom(address, length);
  uint8_t count = 0;

  while (Wire1.available() && count < length) {
    buffer[count++] = Wire1.read();
  }

  return count;
}


int32_t IMU::rawToMilliG(int16_t rawValue) {
  return (int32_t(rawValue) * 61) / 1000;
}

//Integer square root (Newton method)
uint32_t IMU::integerSqrt(uint32_t value) {
  if (value == 0) return 0;

  uint32_t x = value;
  uint32_t y = (x + 1) >> 1;

  while (y < x) {
    x = y;
    y = (x + value / x) >> 1;
  }

  return x;
}
