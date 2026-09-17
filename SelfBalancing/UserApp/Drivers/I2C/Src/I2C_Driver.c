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
  GPIO_PinModeSet(I2C_SDA_PIN_PORT,
                  I2C_SDA_PIN_NUM,
                  gpioModeWiredAndPullUpFilter, 1);


  // Route I2C pins to GPIO
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                        | (gpioPortA << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                        | (5 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                        | (gpioPortA << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                        | (6 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;


  // Initialize the I2C
  I2C_Init(I2C0, &i2cInit);

  // Set the status flags and index
  i2c_handler.i2c_startTx = false;

  // Enable automatic STOP on NACK
  I2C0->CTRL = I2C_CTRL_AUTOSN;

}
/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
