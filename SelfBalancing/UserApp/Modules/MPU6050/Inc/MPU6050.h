
/******************************************************************
  @file       ReefwingMPU6050.h
  @brief      Arduino Library for MPU6050 IMU using I2C
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

   Version:     1.0.0
   Date:        31/07/23

   1.0.0    Original Release            31/07/23

   Credit:  This Library is a fork of Arduino-MPU6050
            ref: https://github.com/jarzebski/Arduino-MPU6050/tree/dev
            Author: Korneliusz Jarzębski

 ******************************************************************/
#ifndef MPU6050_h
#define MPU5060_h

#include "stdbool.h"

#include "MPU6050_Registers.h"
#include "MPU6050_Types.h"



/******************************************************************
 *
 * MPU6050 Class Definition -
 *
 ******************************************************************/

//bool begin(GyroScale scale = GyroScale::MPU6050_SCALE_2000DPS, AccelScale range = AccelScale::MPU6050_RANGE_2G, int mpua = MPU6050_ADDRESS);
bool MPU6050_IMU_begin(GyroScale scale, AccelScale range, int mpu_address);
void MPU6050_IMU_reset(void);
RawData MPU6050_IMU_readRawAccel(void);

bool MPU6050_IMU_setClockSource(ClockSource source);
ClockSource MPU6050_IMU_getClockSource(void);

bool MPU6050_IMU_setScale(GyroScale scale);
GyroScale MPU6050_IMU_getScale();

bool MPU6050_IMU_setRange(AccelScale range);
AccelScale MPU6050_IMU_getRange();

bool MPU6050_IMU_getSleepEnabled();
void MPU6050_IMU_setSleepEnabled(bool state);


uint8_t readRegister8(uint8_t reg);
bool readRegisterBit(uint8_t reg, uint8_t pos);
void readMultipleRegisters(uint8_t reg, uint8_t * retBuf, uint8_t size);
void writeRegister8(uint8_t reg, uint8_t value);
void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);








bool connected();
bool dataAvailable();
void updateSensorData();



void setDHPFMode(HighPassFilter dhpf);
void setDLPFMode(LowPassFilter dlpf);
OnDelay getAccelPowerOnDelay();
void setAccelPowerOnDelay(OnDelay delay);

uint8_t getIntStatus();

bool getIntZeroMotionEnabled();
void setIntZeroMotionEnabled(bool state);
bool getIntMotionEnabled();
void setIntMotionEnabled(bool state);
bool getIntFreeFallEnabled();
void setIntFreeFallEnabled(bool state);

uint8_t getMotionDetectionThreshold();
void setMotionDetectionThreshold(uint8_t threshold);
uint8_t getMotionDetectionDuration();
void setMotionDetectionDuration(uint8_t duration);

uint8_t getZeroMotionDetectionThreshold();
void setZeroMotionDetectionThreshold(uint8_t threshold);
uint8_t getZeroMotionDetectionDuration();
void setZeroMotionDetectionDuration(uint8_t duration);

uint8_t getFreeFallDetectionThreshold();
void setFreeFallDetectionThreshold(uint8_t threshold);
uint8_t getFreeFallDetectionDuration();
void setFreeFallDetectionDuration(uint8_t duration);


bool getI2CMasterModeEnabled();
void setI2CMasterModeEnabled(bool state);
bool getI2CBypassEnabled();
void setI2CBypassEnabled(bool state);

TempData readTemperature();
Activites readActivites();

int16_t getGyroOffsetX();
void setGyroOffsetX(int16_t offset);
int16_t getGyroOffsetY();
void setGyroOffsetY(int16_t offset);
int16_t getGyroOffsetZ();
void setGyroOffsetZ(int16_t offset);

int16_t getAccelOffsetX();
void setAccelOffsetX(int16_t offset);
int16_t getAccelOffsetY();
void setAccelOffsetY(int16_t offset);
int16_t getAccelOffsetZ();
void setAccelOffsetZ(int16_t offset);

void calibrateGyro(uint8_t samples); //=50
void setThreshold(uint8_t multiple); //=1
uint8_t getThreshold(void);

RawData readRawGyro();
ScaledData readNormalizeGyro();

RawData readRawAccel();
ScaledData readNormalizeAccel();
ScaledData readScaledAccel();
InertialMessage getInertial();


//uint8_t fastRegister8(uint8_t reg);



int16_t readRegister16(uint8_t reg);
void writeRegister16(uint8_t reg, int16_t value);

bool readRegisterBit(uint8_t reg, uint8_t pos);
void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

#endif
