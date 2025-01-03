/******************************************************************
  @file       MPU6050_Types.h
  @brief      Types and Enumerations for the MPU6050 6-axis IMU
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:       David Such
  Version:    1.0.0
  Date:       31/07/23

  1.0.0   Original Release.           31/07/23

******************************************************************/
#include "stdbool.h"
#include "stdint.h"

#ifndef MPU6050_TYPES_h
#define MPU5060_TYPES_h

/******************************************************************
 *
 * ENUM Definitions -
 *
 ******************************************************************/

typedef enum  {
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
}ClockSource ;

typedef enum  {
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
}GyroScale;

typedef enum  {
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00
}AccelScale;

typedef enum  {
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00
}OnDelay;

typedef enum  {
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000
}HighPassFilter;

typedef enum  {
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
}LowPassFilter;

/******************************************************************
 *
 * Struct Definitions -
 *
 ******************************************************************/

typedef struct  {
    int16_t x, y, z;
}BiasOffsets;

typedef struct  {
    bool isOverflow;
    bool isFreeFall;
    bool isInactivity;
    bool isActivity;
    bool isPosActivityOnX;
    bool isPosActivityOnY;
    bool isPosActivityOnZ;
    bool isNegActivityOnX;
    bool isNegActivityOnY;
    bool isNegActivityOnZ;
    bool isDataReady;
}Activites;


typedef struct {
  float celsius;
  uint32_t timeStamp;
}TempData;


typedef struct
{

  int16_t rx;
  int16_t ry;
  int16_t rz;
  uint32_t timeStamp;
}RawData;


typedef struct
{
  int16_t sx;
  int16_t sy;
  int16_t sz;
  uint32_t timeStamp;
}ScaledData;


typedef struct
{
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint32_t timeStamp;

  int16_t ax;
  int16_t ay;
  int16_t az;

}InertialMessage;


typedef struct
{
  int16_t gx;
  int16_t gy;
  int16_t gz;
  uint32_t gTimeStamp;

  int16_t ax;
  int16_t ay;
  int16_t az;
  uint32_t aTimeStamp;
}SensorData;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}VectorData;
#endif


