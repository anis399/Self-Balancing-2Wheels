/******************************************************************
  @file       MPU6050_Registers.h
  @brief      Register Map for the MPU6050 6-axis IMU
  @author     David Such
  @copyright  Please see the accompanying LICENSE file

  Code:        David Such
  Version:     1.0.0
  Date:        31/07/23

  1.0.0     Original Release.       31/07/23

******************************************************************/
#ifndef MPU6050_REGS_h
#define MPU5060_REGS_h

#define MPU6050_ADDRESS 0x68

#define MPU6050_REG_ACCEL_XOFFS_H     0x06
#define MPU6050_REG_ACCEL_XOFFS_L     0x07
#define MPU6050_REG_ACCEL_YOFFS_H     0x08
#define MPU6050_REG_ACCEL_YOFFS_L     0x09
#define MPU6050_REG_ACCEL_ZOFFS_H     0x0A
#define MPU6050_REG_ACCEL_ZOFFS_L     0x0B
#define MPU6050_REG_GYRO_XOFFS_H      0x13
#define MPU6050_REG_GYRO_XOFFS_L      0x14
#define MPU6050_REG_GYRO_YOFFS_H      0x15
#define MPU6050_REG_GYRO_YOFFS_L      0x16
#define MPU6050_REG_GYRO_ZOFFS_H      0x17
#define MPU6050_REG_GYRO_ZOFFS_L      0x18
#define MPU6050_REG_CONFIG            0x1A
#define MPU6050_REG_GYRO_CONFIG       0x1B
#define MPU6050_REG_ACCEL_CONFIG      0x1C
#define MPU6050_REG_FF_THRESHOLD      0x1D
#define MPU6050_REG_FF_DURATION       0x1E
#define MPU6050_REG_MOT_THRESHOLD     0x1F
#define MPU6050_REG_MOT_DURATION      0x20
#define MPU6050_REG_ZMOT_THRESHOLD    0x21
#define MPU6050_REG_ZMOT_DURATION     0x22
#define MPU6050_REG_INT_PIN_CFG       0x37
#define MPU6050_REG_INT_ENABLE        0x38
#define MPU6050_REG_INT_STATUS        0x3A
#define MPU6050_REG_ACCEL_XOUT_H      0x3B  //59U
#define MPU6050_REG_ACCEL_XOUT_L      0x3C
#define MPU6050_REG_ACCEL_YOUT_H      0x3D
#define MPU6050_REG_ACCEL_YOUT_L      0x3E
#define MPU6050_REG_ACCEL_ZOUT_H      0x3F
#define MPU6050_REG_ACCEL_ZOUT_L      0x40  //64U
#define MPU6050_REG_TEMP_OUT_H        0x41
#define MPU6050_REG_TEMP_OUT_L        0x42
#define MPU6050_REG_GYRO_XOUT_H       0x43
#define MPU6050_REG_GYRO_XOUT_L       0x44
#define MPU6050_REG_GYRO_YOUT_H       0x45
#define MPU6050_REG_GYRO_YOUT_L       0x46
#define MPU6050_REG_GYRO_ZOUT_H       0x47
#define MPU6050_REG_GYRO_ZOUT_L       0x48
#define MPU6050_REG_MOT_DETECT_STATUS 0x61
#define MPU6050_REG_MOT_DETECT_CTRL   0x69
#define MPU6050_REG_USER_CTRL         0x6A  //106U
#define MPU6050_REG_PWR_MGMT_1        0x6B  //107U
#define MPU6050_REG_PWR_MGMT_2        0x6C  //108U

#define MPU6050_REG_WHO_AM_I          0x75


#endif
