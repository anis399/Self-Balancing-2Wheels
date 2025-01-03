/*******************************************************************************
 *                     _           _           _    _                           *
 *         /\         (_)         | |         | |  | |                          *
 *        /  \   _ __  _ ___   ___| |__   __ _| | _| | _____  _   _ _ __        *
 *       / /\ \ | '_ \| / __| / __| '_ \ / _` | |/ / |/ / _ \| | | | '__|       *
 *      / ____ \| | | | \__ \ \__ \ | | | (_| |   <|   < (_) | |_| | |          *
 *     /_/    \_\_| |_|_|___/ |___/_| |_|\__,_|_|\_\_|\_\___/ \__,_|_|          *
 *                                                                              *
 *******************************************************************************/
/******************************************************************************
 * File Name.c
 *
 *  Created on: November 03, 2024
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "I2C_Driver.h"

/*******************************************************************************
 * Data types
 ******************************************************************************/
I2C_Handler i2c_handler;


/*******************************************************************************
 * Extern
 ******************************************************************************/

/*******************************************************************************
 * Interface Functions
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void I2Cdrv_Init(void)
{
  // Enable clocks to the I2C and GPIO
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Use default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;


  // Setting HW pins
  GPIO_PinModeSet(I2C_SDA_PIN_PORT,
                  I2C_SDA_PIN_NUM,
                  gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(I2C_SCL_PIN_PORT,
                  I2C_SCL_PIN_NUM,
                  gpioModeWiredAndPullUpFilter, 1);


  // Route I2C pins to GPIO
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                            | (I2C_SDA_PIN_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                                | (I2C_SDA_PIN_NUM << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                            | (I2C_SCL_PIN_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                                | (I2C_SCL_PIN_NUM << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;


  // Initialize the I2C
  I2C_Init(I2C0, &i2cInit);

  // Set the status flags and index
  i2c_handler.i2c_startTx = false;

  // Enable automatic STOP on NACK
  I2C0->CTRL = I2C_CTRL_AUTOSN;
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
void I2Cdrv_ReadBlocking(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t numBytes)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initialize I2C transfer
  i2cTransfer.addr          = followerAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading
  i2cTransfer.buf[0].data   = &targetAddress;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = rxBuff;
  i2cTransfer.buf[1].len    = numBytes;

  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Send data
  while (result == i2cTransferInProgress)
    {
      result = I2C_Transfer(I2C0);
    }
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
void I2Cdrv_WriteBlocking(uint16_t slaveAddress, uint8_t regAddress, uint8_t value)
{

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t write[2] = {regAddress, value};
  uint8_t read[2] = {9, 9};
  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE; // must write target address before reading
  i2cTransfer.buf[0].data   = write;
  i2cTransfer.buf[0].len    = 2;
  i2cTransfer.buf[1].data   = read;
  i2cTransfer.buf[1].len    = 1;

  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Send data
  while (result == i2cTransferInProgress)
    {
      result = I2C_Transfer(I2C0);
    }

}



/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
