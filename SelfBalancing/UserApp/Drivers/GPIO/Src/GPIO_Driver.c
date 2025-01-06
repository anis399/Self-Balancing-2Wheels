/******************************************************************************
 * File Name.c
 *
 *  Created on: Aug 08, 2023
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 *****************************************************************************/
/******************************************************************************
 * Includes
 *****************************************************************************/
#include "GPIO_Driver.h"

/*******************************************************************************
 * Data types
 ******************************************************************************/

/*******************************************************************************
 * Interface Functions
 ******************************************************************************/
/******************************************************************************
 * @ingroup GPIO Init
 * @brief Inits most of boards GPIO
 *
 * Components GPIOs are not initialized here
 *
 * @param[in]   NONE
 * @param[out]  NONE
 * @return      NONE
 *
 * @author     Anis Shakkour
 * @date       October 17, 2024
 *
 * @warning:    components GPIOS are not initialized here. SiLabs takes care of it.
 *              maybe its better if we do it.
 *
 ******************************************************************************/
void GPIO_Init(void)
{
  // make sure clock is enabled to
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_DisableInt();


  //******************************** MotorA Direction Control 1
  /* Regular I/O */
#if defined(MOTOR_IN1_port)
  GPIO_PinModeSet(MOTOR_IN1_port,
                  MOTOR_IN1_pin,
                  MOTOR_IN1_mode,
                  MOTOR_IN1_init_state);
#endif

  //******************************** MotorA Direction Control 2
  /* Regular I/O */
#if defined(MOTOR_IN2_port)
  GPIO_PinModeSet(MOTOR_IN2_port,
                  MOTOR_IN2_pin,
                  MOTOR_IN2_mode,
                  MOTOR_IN2_init_state);
#endif

  //******************************** MotorA Speed Control
  /* Alternative function -> PWM */
#if defined(MOTOR_ENA_port)
  GPIO_PinModeSet(MOTOR_ENA_port,
                  MOTOR_ENA_pin,
                  MOTOR_ENA_mode,
                  MOTOR_ENA_init_state);
#endif






  //******************************** MotorB Direction Control 1
  /* Regular I/O */
#if defined(MOTOR_IN3_port)
  GPIO_PinModeSet(MOTOR_IN3_port,
                  MOTOR_IN3_pin,
                  MOTOR_IN3_mode,
                  MOTOR_IN3_init_state);
#endif

  //******************************** MotorB Direction Control 2
  /* Regular I/O */
#if defined(MOTOR_IN4_port)
  GPIO_PinModeSet(MOTOR_IN4_port,
                  MOTOR_IN4_pin,
                  MOTOR_IN4_mode,
                  MOTOR_IN4_init_state);
#endif

  //******************************** MotorB Speed Control
  /* Alternative function -> PWM */
#if defined(MOTOR_ENB_port)
  GPIO_PinModeSet(MOTOR_ENB_port,
                  MOTOR_ENB_pin,
                  MOTOR_ENB_mode,
                  MOTOR_ENB_init_state);
#endif






  //******************************** IMU Interrupt
  /* BGM_ID1 */
#if defined(BGM_ID1_INT_port)
  GPIO_PinModeSet(BGM_ID1_INT_port,
                  BGM_ID1_INT_pin,
                  BGM_ID1_INT_mode,
                  BGM_ID1_INT_init_state);

  GPIO_ExtIntConfig(BGM_ID1_INT_port,
                    BGM_ID1_INT_pin,
                    BGM_ID1_INT_intNo,
                    BGM_ID1_INT_ONrisingEdge,
                    BGM_ID1_INT_ONfallingEdge,
                    BGM_ID1_INT_enable);

  GPIOINT_CallbackRegister(BGM_ID1_INT_intNo,
                           (GPIOINT_IrqCallbackPtr_t) gpioCallback_Interrupts);

#endif


  GPIO_EnableInt();

}


/******************************************************************************
 * @ingroup GPIO Init
 * @brief Inits most of boards GPIO
 *
 * Components GPIOs are not initialized here
 *
 * @param[in]   NONE
 * @param[out]  NONE
 * @return      NONE
 *
 * @author     Anis Shakkour
 * @date       December 8, 2024
 *
 * @warning:    components GPIOS are not initialized here. SiLabs takes care of it.
 *              maybe its better if we do it.
 *
 ******************************************************************************/
void GPIO_EnableInt(void)
{

  uint32_t IntFlags = 0;

  GPIO_IntClear(GPIO_IntGet());

  //*********************************** Enable Interrupts
#ifdef BGM_EEG1_INT_intNo
  IntFlags += 1 <<  BGM_EEG1_INT_intNo;
#endif
#ifdef BGM_EEG2_INT_intNo
  IntFlags += 1 <<  BGM_EEG2_INT_intNo;
#endif
#ifdef BGM_IMU_INT_intNo
  IntFlags += 1 <<  BGM_IMU_INT_intNo;
#endif
#ifdef BGM_BT_INT_intNo
  IntFlags += 1 <<  BGM_BT_INT_intNo;
#endif
#ifdef BGM_ID_PIN_INT_intNo
  IntFlags += 1 <<  BGM_ID_PIN_INT_intNo;
#endif


  GPIO_IntEnable(IntFlags);
  //enable_irq();
}


/******************************************************************************
 * @ingroup GPIO Init
 * @brief Inits most of boards GPIO
 *
 * Components GPIOs are not initialized here
 *
 * @param[in]   NONE
 * @param[out]  NONE
 * @return      NONE
 *
 * @author     Anis Shakkour
 * @date       October 17, 2024
 *
 * @warning:    components GPIOS are not initialized here. SiLabs takes care of it.
 *              maybe its better if we do it.
 *
 ******************************************************************************/
void GPIO_DisableInt(void)
{

  uint32_t IntFlags = 0;

  GPIO_IntClear(GPIO_IntGet());

  //*********************************** Disable Interrupts
#ifdef BGM_EEG1_INT_intNo
  IntFlags += 1 <<  BGM_EEG1_INT_intNo;
#endif
#ifdef BGM_EEG2_INT_intNo
  IntFlags += 1 <<  BGM_EEG2_INT_intNo;
#endif
#ifdef BGM_IMU_INT_intNo
  IntFlags += 1 <<  BGM_IMU_INT_intNo;
#endif
#ifdef BGM_BT_INT_intNo
  IntFlags += 1 <<  BGM_BT_INT_intNo;
#endif
#ifdef BGM_ID_PIN_INT_intNo
  IntFlags += 1 <<  BGM_ID_PIN_INT_intNo;
#endif
  GPIO_IntDisable(IntFlags);
  //__disable_irq();

}


/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
