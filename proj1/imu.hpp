
#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <Wire.h>


#define LSM9DS1_AG_ADDR 0x6B  // Accelerometer/Gyroscope
#define LSM9DS1_M_ADDR  0x1E  // Magnetometer

// LSM9DS1 Register Map
#define CTRL_REG5_XL     0x1F
#define CTRL_REG6_XL     0x20
#define CTRL_REG7_XL     0x21

#define OUT_X_L_XL       0x28
#define OUT_X_H_XL       0x29
#define OUT_Y_L_XL       0x2A
#define OUT_Y_H_XL       0x2B
#define OUT_Z_L_XL       0x2C
#define OUT_Z_H_XL       0x2D

#define WHO_AM_I         0x0F
#define WHO_AM_I_VAL     0x68   // Expected AG WHO_AM_I

class IMU {
public:
  IMU();

  bool begin();
  void readAcceleration();

  int16_t getRawAccelX();
  int16_t getRawAccelY();
  int16_t getRawAccelZ();

  int32_t getAccelX_mg();
  int32_t getAccelY_mg();
  int32_t getAccelZ_mg();

  uint32_t getAccelMagnitude_mg();

  void printAcceleration();

private:
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;

  bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);
  uint8_t readRegister(uint8_t address, uint8_t reg);
  uint8_t readRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length);

  int32_t rawToMilliG(int16_t rawValue);
  uint32_t integerSqrt(uint32_t value);
};

#endif // IMU_HPP
