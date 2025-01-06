/*******************************************************************************
 * File PWM.C
 *
 *  Created on: Feb 03, 2023
 *  Author: Anis Shakkour
 *  Email:  anis.shakkour399@gmail.com
 ******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "PWM.h"

/*******************************************************************************
 * Data types
 ******************************************************************************/

/*******************************************************************************
 * Extern
 ******************************************************************************/

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
CMU_Clock_TypeDef PWM_D_get_timer_clock(TIMER_TypeDef *timer)
{
#if defined(_CMU_HFCLKSEL_MASK) || defined(_CMU_CMD_HFCLKSEL_MASK)
  CMU_Clock_TypeDef timer_clock = cmuClock_HF;
#elif defined(_CMU_SYSCLKCTRL_MASK)
  CMU_Clock_TypeDef timer_clock = cmuClock_SYSCLK;
#else
#error "Unknown root of clock tree"
#endif

  switch ((uint32_t)timer) {
#if defined(TIMER0_BASE)
    case TIMER0_BASE:
      timer_clock = cmuClock_TIMER0;
      break;
#endif
#if defined(TIMER1_BASE)
    case TIMER1_BASE:
      timer_clock = cmuClock_TIMER1;
      break;
#endif
#if defined(TIMER2_BASE)
    case TIMER2_BASE:
      timer_clock = cmuClock_TIMER2;
      break;
#endif
#if defined(TIMER3_BASE)
    case TIMER3_BASE:
      timer_clock = cmuClock_TIMER3;
      break;
#endif
#if defined(TIMER4_BASE)
    case TIMER4_BASE:
      timer_clock = cmuClock_TIMER4;
      break;
#endif
#if defined(TIMER5_BASE)
    case TIMER5_BASE:
      timer_clock = cmuClock_TIMER5;
      break;
#endif
#if defined(TIMER6_BASE)
    case TIMER6_BASE:
      timer_clock = cmuClock_TIMER6;
      break;
#endif
#if defined(WTIMER0_BASE)
    case WTIMER0_BASE:
      timer_clock = cmuClock_WTIMER0;
      break;
#endif
#if defined(WTIMER1_BASE)
    case WTIMER1_BASE:
      timer_clock = cmuClock_WTIMER1;
      break;
#endif
#if defined(WTIMER2_BASE)
    case WTIMER2_BASE:
      timer_clock = cmuClock_WTIMER2;
      break;
#endif
#if defined(WTIMER3_BASE)
    case WTIMER3_BASE:
      timer_clock = cmuClock_WTIMER3;
      break;
#endif
    default:
      EFM_ASSERT(0);
      break;
  }
  return timer_clock;
}

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_ConfigTimer(Timer_t *timerP)
{
  if(!TIMER_REF_VALID(timerP->timer))
    {
#ifdef DEBUG_PWM
      printf("PWM D:Invalid timer\n");
#endif
      return FuncFailed;
    }

  /* Reset timer module*/
  //PWM_D_DisableTimer(timerP);
  //TIMER_Reset(timerP->timer);

  /* Enable clock for timer module */
  CMU_Clock_TypeDef timer_clock = PWM_D_get_timer_clock(timerP->timer);
  CMU_ClockEnable(timer_clock, true);

  /* Initialize TIMER module */
  TIMER_Init_TypeDef timer_init =
      {
          .enable     = false,
          .debugRun   = false,
          .prescale   = timerPrescale4,
          .clkSel     = timerClkSelHFPerClk,
          .count2x    = false,
          .ati        = false,
          .fallAction = timerInputActionNone,
          .riseAction = timerInputActionNone,
          .mode       = timerModeUpDown,
          .dmaClrAct  = true,
          .quadModeX4 = false,
          .oneShot    = false,
          .sync       = false,
      };
  TIMER_Init(timerP->timer, &timer_init);

  /** Set PWM Period */
  uint32_t timerFreq = CMU_ClockFreqGet(timer_clock) / (1 << timer_init.prescale);
  uint32_t top_value = (timerFreq / timerP->frequency);
  TIMER_TopSet(timerP->timer, top_value);

  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_ConfigAll_CH
 *
 * Description  : configures 3 channels at once
 * Parameters   :
 * Returns      :
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_ConfigAll_CH(Timer_t *timerP, PWM_Output_t *output0, PWM_Output_t *output1, PWM_Output_t *output2)
{
  /* Enable clock for timer module */
   CMU_Clock_TypeDef timer_clock = PWM_D_get_timer_clock(timerP->timer);
   CMU_ClockEnable(timer_clock, true);

   /* Initialize TIMER module */
   TIMER_Init_TypeDef timer_init =
       {
           .enable     = false,
           .debugRun   = false,
           .prescale   = timerPrescale4,
           .clkSel     = timerClkSelHFPerClk,
           .count2x    = false,
           .ati        = false,
           .fallAction = timerInputActionNone,
           .riseAction = timerInputActionNone,
           .mode       = timerModeUpDown,
           .dmaClrAct  = true,
           .quadModeX4 = false,
           .oneShot    = false,
           .sync       = false,
       };
   TIMER_Init(timerP->timer, &timer_init);

   /** Set PWM Period */
   uint32_t timerFreq = CMU_ClockFreqGet(timer_clock) / (1 << timer_init.prescale);
   uint32_t top_value = (timerFreq / timerP->frequency);
   TIMER_TopSet(timerP->timer, top_value);

  /** Select CC0 channel parameters */
  TIMER_InitCC_TypeDef channel_init0 =
      {
          .eventCtrl  = timerEventEveryEdge,
          .edge       = timerEdgeNone,
          .prsSel     = timerPRSSELCh0,
          .cufoa      = timerOutputActionNone,
          .cofoa      = timerOutputActionNone,
          .cmoa       = timerOutputActionToggle,
          .mode       = timerCCModePWM,
          .filter     = false,
          .prsInput   = false,
          .coist      = false,
          .outInvert  = false,
      };
  TIMER_InitCC(timerP->timer, output0->ch, &channel_init0);

  /** Select CC1 channel parameters */
  TIMER_InitCC_TypeDef channel_init1 =
      {
          .eventCtrl  = timerEventEveryEdge,
          .edge       = timerEdgeNone,
          .prsSel     = timerPRSSELCh1,
          .cufoa      = timerOutputActionNone,
          .cofoa      = timerOutputActionNone,
          .cmoa       = timerOutputActionToggle,
          .mode       = timerCCModePWM,
          .filter     = false,
          .prsInput   = false,
          .coist      = false,
          .outInvert  = false,
      };
  TIMER_InitCC(timerP->timer, output1->ch, &channel_init1);

  /** Select CC channel parameters */
  TIMER_InitCC_TypeDef channel_init2 =
      {
          .eventCtrl  = timerEventEveryEdge,
          .edge       = timerEdgeNone,
          .prsSel     = timerPRSSELCh2,
          .cufoa      = timerOutputActionNone,
          .cofoa      = timerOutputActionNone,
          .cmoa       = timerOutputActionToggle,
          .mode       = timerCCModePWM,
          .filter     = false,
          .prsInput   = false,
          .coist      = false,
          .outInvert  = false,
      };
  TIMER_InitCC(timerP->timer, output2->ch, &channel_init2);

  /** Set PWM DC */
  uint32_t top_val = TIMER_TopGet(timerP->timer);
  TIMER_CompareBufSet(timerP->timer, output0->ch, PWM_D_angle_to_DC(top_val, output0->angle));
  TIMER_CompareBufSet(timerP->timer, output1->ch, PWM_D_angle_to_DC(top_val, output1->angle));
  TIMER_CompareBufSet(timerP->timer, output2->ch, PWM_D_angle_to_DC(top_val, output2->angle));

  /** Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet(output0->port, output0->pin, gpioModePushPull, 0);
  GPIO_PinModeSet(output0->port, output0->pin, gpioModePushPull, 0);
  GPIO_PinModeSet(output0->port, output0->pin , gpioModePushPull, 0);

  /** Set CCn location as outputs */
  GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN | GPIO_TIMER_ROUTEEN_CC2PEN;

  // tibia
  GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC0ROUTE = (output0->port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                                                 | (output0->pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  // fimur
  GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC1ROUTE = (output1->port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
                                                 | (output1->pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);

  // coxa
  GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC2ROUTE = (output2->port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
                                                   | (output2->pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);

  TIMER_Enable(timerP->timer, true);
  return FuncSuccess;
}


/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_ConfigCH(Timer_t *timerP, PWM_Output_t *output, float default_angle)
{

  /** Validate timer */
  if(!TIMER_REF_VALID(timerP->timer))
    {
#ifdef DEBUG_PWM
      printf("PWM D:Invalid timer\n");
#endif
      return FuncFailed;
    }

  /** Validate channel */
  if(!TIMER_CH_VALID(output->ch))
    {
#ifdef DEBUG_PWM
      printf("PWM D:Invalid timer channel\n");
#endif
      return FuncFailed;
    }

  /** Select CCn channel parameters */
  TIMER_InitCC_TypeDef channel_initN =
      {
          .eventCtrl  = timerEventEveryEdge,
          .edge       = timerEdgeNone,
          .prsSel     = timerPRSSELCh0 << output->ch,
          .cufoa      = timerOutputActionNone,
          .cofoa      = timerOutputActionNone,
          .cmoa       = timerOutputActionToggle,
          .mode       = timerCCModePWM,
          .filter     = false,
          .prsInput   = false,
          .coist      = false,
          .outInvert  = false,
      };
  TIMER_InitCC(timerP->timer, output->ch, &channel_initN);

  /** Set PWM DC */
  uint32_t top_val = TIMER_TopGet(timerP->timer);
  uint32_t DC = PWM_D_angle_to_DC(top_val, default_angle);
  TIMER_CompareBufSet(timerP->timer, output->ch, DC);

  /** Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /** Enable GPIO pin as output */
  GPIO_PinModeSet(output->port, output->pin, gpioModePushPull, 0);

  /** Enable route CCn location to output */
  GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].ROUTEEN  |= GPIO_TIMER_ROUTEEN_CC0PEN << output->ch;

  /**Route CCn location to output */
  if(output->ch == 0){
      GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC0ROUTE = (output->port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
              | (output->pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  }
  if(output->ch == 1){
      GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC1ROUTE = (output->port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
              | (output->pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
  }
  if(output->ch == 2){
      GPIO->TIMERROUTE[TIMER_NUM(timerP->timer)].CC2ROUTE = (output->port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
              | (output->pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);
  }
  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_EnableTimer(Timer_t *timerP)
{
  TIMER_Enable(timerP->timer, true);
  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_DisableTimer(Timer_t *timerP)
{
  TIMER_Enable(timerP->timer, false);
  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_EnableCH(Timer_t *timerP, PWM_Output_t *output)
{
  (void)timerP;
  (void)output;

  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_get_timer_clock
 *
 * Description  : search for timer clock that corresponds to the timer needed
 * Parameters   : Timer
 * Returns      : cmu clock
 *
 * Known issues :
 * Note         :
 ******************************************************************************/
RetValue_t PWM_D_DisableCH(Timer_t *timerP, PWM_Output_t *output)
{
  (void)timerP;
  (void)output;
  return FuncSuccess;
}

/*******************************************************************************
 * Function name: PWM_D_angle_to_DC
 *
 * Description  : Converts angles from degrees to the duty cycle value
 * Parameters   : topvalue which is set for timer period and the desired angle
 * Returns      : DC which, timer counter value set for the timer channel to toggle
 *                at when reached
 *
 * Known issues :
 * Note         : The divided range to get all possible DC that aligns with the
 *                angle may need some modifications
 *
 * TODO         : Review the division of the angles,
 *                Review the desired angle limits
 ******************************************************************************/
uint32_t PWM_D_angle_to_DC(uint32_t topvalue, float angle)
{
  //angle = MAX(MIN(angle,MAX_ANGLE), MIN_ANGLE);
  uint32_t DC = ((float)topvalue/20.0)*(1.0 + ((float)angle/(float)MAX_ANGLE));

#ifdef DEBUG_PWM
  printf("PWM:Angle %f   DC %f\n",angle ,DC);
#endif
  return DC;
}
