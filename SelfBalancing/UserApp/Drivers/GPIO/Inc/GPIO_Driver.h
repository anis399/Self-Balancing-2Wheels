/*******************************************************************************
*                      _____ _    _          ____                              *
*                     / ____| |  | |   /\   / __ \                             *
*                    | (___ | |__| |  /  \ | |  | |___                         *
*                     \___ \|  __  | / /\ \| |  | / __|                        *
*                     ____) | |  | |/ ____ \ |__| \__ \                        *
*                    |_____/|_|  |_/_/    \_\___\_\___/                        *
*                                                                              *
********************************************************************************
* All Software (C)
*******************************************************************************/
/*******************************************************************************
*                     _           _           _    _                           *
*         /\         (_)         | |         | |  | |                          *
*        /  \   _ __  _ ___   ___| |__   __ _| | _| | _____  _   _ _ __        *
*       / /\ \ | '_ \| / __| / __| '_ \ / _` | |/ / |/ / _ \| | | | '__|       *
*      / ____ \| | | | \__ \ \__ \ | | | (_| |   <|   < (_) | |_| | |          *
*     /_/    \_\_| |_|_|___/ |___/_| |_|\__,_|_|\_\_|\_\___/ \__,_|_|          *
*                                                                              *
*******************************************************************************/
/*******************************************************************************
 * File Template.h
 *
 *  Created on: Jan 6th, 2025
 *
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 *
 *  @brief :
 *  @note  :
 *  @todo  :
 *
 * For more information, please refer to the <a href="https://example.com/my_document.pdf">documentation</a>.
 *******************************************************************************/

/*******************************************************************************
 * Multiple include protection
 ******************************************************************************/
#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "SelfBalancing_Common.h"

//#include "cmsis_gcc.h"
//#include "gpiointerrupt.h"
#include "em_gpio.h"
#include "em_cmu.h"

/*******************************************************************************
 * Extern
 ******************************************************************************/

/*******************************************************************************
 * Macros
 ******************************************************************************/

/*******************************************************************************
 * Defines
 ******************************************************************************/
/*
 *
 * PA01 SWCLK (enabled after reset built in pulldown)
 * PA02 SWDIO (enabled after reset built in pullup)
 * PA03 N/A   (disabled after reset)
 * PA04 N/A   (disabled after reset, once enabled has a built in pull-up)
 *
 * PB01 PWM ENA
 * PB02 PWM ENB
 * PB03 I2C
 * PB04 I2C
 *
 * PC00 MotorA_IN1
 * PC01 MotorA_IN2
 * PC02 MotorB_IN1
 * PC03 MotorB_IN1
 */

/****************************** I2C ******************************/
#if defined(ENABLE__BGM_BLE_LED)
#define I2C_SDA_port           ((uint8_t)gpioPortB )             /*  */
#define I2C_SDA_pin            ((uint8_t)3         )             /*  */
#define I2C_SDA_mode           ((uint8_t)gpioModePushPull)       /*  */
#define I2C_SDA_init_state     ((uint8_t)0         )             /*  */

#define I2C_SCL_port           ((uint8_t)gpioPortB )             /*  */
#define I2C_SCL_pin            ((uint8_t)4         )             /*  */
#define I2C_SCL_mode           ((uint8_t)gpioModePushPull)       /*  */
#define I2C_SCL_init_state     ((uint8_t)0         )             /*  */

#endif


/****************************** MOTOR A ******************************/
#if defined(ENABLE__MOTOR_CONTROLLER)
#define MOTOR_IN1_port           ((uint8_t)gpioPortC )             /*  */
#define MOTOR_IN1_pin            ((uint8_t)0         )             /*  */
#define MOTOR_IN1_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_IN1_init_state     ((uint8_t)0         )             /*  */

#define MOTOR_IN2_port           ((uint8_t)gpioPortC )             /*  */
#define MOTOR_IN2_pin            ((uint8_t)1         )             /*  */
#define MOTOR_IN2_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_IN2_init_state     ((uint8_t)0         )             /*  */

#define MOTOR_ENA_port           ((uint8_t)gpioPortB )             /*  */
#define MOTOR_ENA_pin            ((uint8_t)1         )             /*  */
#define MOTOR_ENA_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_ENA_init_state     ((uint8_t)0         )             /*  */

#endif


/****************************** MOTOR A ******************************/
#if defined(ENABLE__MOTOR_CONTROLLER)
#define MOTOR_IN3_port           ((uint8_t)gpioPortC )             /*  */
#define MOTOR_IN3_pin            ((uint8_t)2         )             /*  */
#define MOTOR_IN3_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_IN3_init_state     ((uint8_t)0         )             /*  */

#define MOTOR_IN4_port           ((uint8_t)gpioPortC )             /*  */
#define MOTOR_IN4_pin            ((uint8_t)3         )             /*  */
#define MOTOR_IN4_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_IN4_init_state     ((uint8_t)0         )             /*  */

#define MOTOR_ENB_port           ((uint8_t)gpioPortB )             /*  */
#define MOTOR_ENB_pin            ((uint8_t)2         )             /*  */
#define MOTOR_ENB_mode           ((uint8_t)gpioModePushPull)       /*  */
#define MOTOR_ENB_init_state     ((uint8_t)0         )             /*  */

#endif


/****************************** UART  ******************************/
#if defined(ENABLE__UART)
#define BGM_UART_TX_port           ((uint8_t)gpioPortA )             /*  */
#define BGM_UART_TX_pin            ((uint8_t)5         )             /*  */
#define BGM_UART_TX_mode           ((uint8_t)gpioModePushPull)       /*  */
#define BGM_UART_TX_init_state     ((uint8_t)0         )             /*  */

#define BGM_UART_RX_port           ((uint8_t)gpioPortA )             /*  */
#define BGM_UART_RX_pin            ((uint8_t)6         )             /*  */
#define BGM_UART_RX_mode           ((uint8_t)gpioModePushPull)       /*  */
#define BGM_UART_RX_init_state     ((uint8_t)0         )             /*  */

#endif


/****************************** IMU Int ******************************/
#if defined(ENABLE__BGM_ID1)
#define BGM_ID1_INT_port          ((uint8_t)gpioPortA )             /*  */
#define BGM_ID1_INT_pin           ((uint8_t)8         )             /*  */
#define BGM_ID1_INT_mode          ((uint8_t)gpioModeInput)          /*  */
#define BGM_ID1_INT_init_state    ((uint8_t)0         )             /*  */
#define BGM_ID1_INT_enable        (true)
#define BGM_ID1_INT_ONrisingEdge  (true)
#define BGM_ID1_INT_ONfallingEdge (true)

#define BGM_ID1_INT_intNo          ((uint8_t)8)
#endif



/*******************************************************************************
 * Typedef & Enums
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
void gpioCallback_Interrupts(uint8_t intNo);


/*******************************************************************************
 * Interface Functions
 ******************************************************************************/
void GPIO_Init(void);
void GPIO_EnableInt(void);
void GPIO_DisableInt(void);



/*******************************************************************************
 * END
 ******************************************************************************/
#endif /* GPIO_DRIVER_H_ */
