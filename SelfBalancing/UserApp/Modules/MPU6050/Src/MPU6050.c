/******************************************************************************
 * File Name.c
 *
 *  Created on: April 08, 2023
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
//Components

//Custom
#include "MPU6050.h"
#include "I2C_Driver.h"

/*******************************************************************************
 * Data types
 ******************************************************************************/

VectorData tg, dg, th;      // Threshold and Delta for Gyro
RawData rawAcc, rawGyro;             // Raw vectors

bool useCalibrate;

int mpuAddress;
float actualThreshold;
float dpsPerDigit, rangePerDigit;

/*******************************************************************************
 * Extern
 ******************************************************************************/


/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*******************************************************************************
 * Interface Functions
 ******************************************************************************/
/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
bool MPU6050_IMU_begin(GyroScale scale, AccelScale range, int mpu_address)
{
  bool retRes = true;

  // Set Address
  mpuAddress = mpu_address << 1;

  // Reset calibrate values
  dg.x = 0;
  dg.y = 0;
  dg.z = 0;
  useCalibrate = false;

  // Reset threshold values
  tg.x = 0;
  tg.y = 0;
  tg.z = 0;
  actualThreshold = 0;

  // Check MPU6050 Who Am I Register
  if (readRegister8(MPU6050_REG_WHO_AM_I) != 0x68)
    {
      return false;
    }

  // Set Clock Source
  retRes &= MPU6050_IMU_setClockSource(MPU6050_CLOCK_PLL_XGYRO);


  // Set Scale & Range
  retRes &= MPU6050_IMU_setScale(scale);
  retRes &= MPU6050_IMU_setRange(range);

  // Disable Sleep Mode
  MPU6050_IMU_setSleepEnabled(false);

  return retRes;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
RawData MPU6050_IMU_readRawAccel(void)
{
  uint8_t buf[6] = {0};
  readMultipleRegisters(MPU6050_REG_ACCEL_XOUT_H, buf, 6);

  uint8_t xoutHighAcc = buf[0];
  uint8_t xoutLowAcc  = buf[1];
  uint8_t youtHighAcc = buf[2];
  uint8_t youtLowAcc  = buf[3];
  uint8_t zoutHighAcc = buf[4];
  uint8_t zoutLowAcc  = buf[5];

  rawAcc.rx = (int16_t)(xoutHighAcc << 8 | xoutLowAcc);
  rawAcc.ry = (int16_t)(youtHighAcc << 8 | youtLowAcc);
  rawAcc.rz = (int16_t)(zoutHighAcc << 8 | zoutLowAcc);
//  ra.timeStamp = micros();

  return rawAcc;
}

/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
bool MPU6050_IMU_setClockSource(ClockSource source)
{
  uint8_t value, temp = 0;

  value = readRegister8(MPU6050_REG_PWR_MGMT_1);
  value &= 0b11111000;
  value |= source;

  writeRegister8(MPU6050_REG_PWR_MGMT_1, value);
  temp = readRegister8(MPU6050_REG_PWR_MGMT_1);

  if(temp != value)
    return false;
  return true;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
ClockSource MPU6050_IMU_getClockSource(void)
{
  uint8_t value;

  value = readRegister8(MPU6050_REG_PWR_MGMT_1);
  value &= 0b00000111;
  return (ClockSource)value;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
bool MPU6050_IMU_setScale(GyroScale scale)
{
  uint8_t value, temp;

  switch (scale)
  {
    case MPU6050_SCALE_250DPS:
      dpsPerDigit = .007633f;
      break;
    case MPU6050_SCALE_500DPS:
      dpsPerDigit = .015267f;
      break;
    case MPU6050_SCALE_1000DPS:
      dpsPerDigit = .030487f;
      break;
    case MPU6050_SCALE_2000DPS:
      dpsPerDigit = .060975f;
      break;
    default:
      break;
  }

  value = readRegister8(MPU6050_REG_GYRO_CONFIG);
  value &= 0b11100111;
  value |= (scale << 3);

  writeRegister8(MPU6050_REG_GYRO_CONFIG, value);
  temp =  readRegister8(MPU6050_REG_GYRO_CONFIG);

  if(temp != value)
    return false;
  return true;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
GyroScale MPU6050_IMU_getScale(void)
{
  uint8_t value;

  value = readRegister8(MPU6050_REG_GYRO_CONFIG);
  value &= 0b00011000;
  value >>= 3;
  return (GyroScale)value;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
bool MPU6050_IMU_setRange(AccelScale range)
{
  uint8_t value, temp;

  switch (range) {
    case MPU6050_RANGE_2G:
      rangePerDigit = .000061f;
      break;
    case MPU6050_RANGE_4G:
      rangePerDigit = .000122f;
      break;
    case MPU6050_RANGE_8G:
      rangePerDigit = .000244f;
      break;
    case MPU6050_RANGE_16G:
      rangePerDigit = .0004882f;
      break;
    default:
      break;
  }

  value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
  value &= 0b11100111;
  value |= (range << 3);

  writeRegister8(MPU6050_REG_ACCEL_CONFIG, value);
  temp =  readRegister8(MPU6050_REG_GYRO_CONFIG);

  if(temp != value)
    return false;
  return true;
}


/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
AccelScale MPU6050_IMU_getRange(void)
{
  uint8_t value;

  value = readRegister8(MPU6050_REG_ACCEL_CONFIG);
  value &= 0b00011000;
  value >>= 3;
  return (AccelScale)value;
}



/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
void MPU6050_IMU_setSleepEnabled(bool state)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

/*******************************************************************************
 * Function name:
 *
 * Description  :
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
bool MPU6050_IMU_getSleepEnabled(void)
{
  return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}





/*******************************************************************************
 * Function name:
 *
 * Description  : Read 8-bit from register
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
uint8_t readRegister8(uint8_t reg)
{
  uint8_t value = 0;
  I2Cdrv_ReadBlocking(mpuAddress, reg, &value, 1); //this is how the address should be used
  return value;
}

/*******************************************************************************
 * Function name:
 *
 * Description  : Read 8-bit from register
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
void readMultipleRegisters(uint8_t reg, uint8_t * retBuf, uint8_t size)
{
  uint8_t value = 0;
  I2Cdrv_ReadBlocking(mpuAddress, reg, &retBuf, size); //this is how the address should be used
}


/*******************************************************************************
 * Function name:
 *
 * Description  : Write 8-bit to register
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
void writeRegister8(uint8_t reg, uint8_t value)
{
  I2Cdrv_WriteBlocking(mpuAddress, reg, value); //this is how the address should be used
}

/*******************************************************************************
 * Function name:
 *
 * Description  : Write register bit
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 * TODO         : check null dereferencing
 ******************************************************************************/
void writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
  uint8_t value;

  value = readRegister8(reg);

  if (state)
    {
      value |= (1 << pos);
    }
  else
    {
      value &= ~(1 << pos);
    }

  writeRegister8(reg, value);
}
