/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm Limited or its affiliates.
 *
 *            (C) COPYRIGHT 2010-2011 Arm Limited or its affiliates.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Arm Limited or its affiliates.
 *
 *      SVN Information
 *
 *      Checked In          : $Date: 2017-10-10 15:55:38 +0100 (Tue, 10 Oct 2017) $
 *
 *      Revision            : $Revision: 371321 $
 *
 *      Release Information : Cortex-M System Design Kit-r1p1-00rel0
 *-----------------------------------------------------------------------------
 */
/*************************************************************************//**
 * @file     CMSDK_driver.h
 * @brief    CMSDK Example Device Driver Header File
 * @version  $State:$
 * @date     $Date: 2017-10-10 15:55:38 +0100 (Tue, 10 Oct 2017) $
 *
 ******************************************************************************/


/** @addtogroup CMSIS_CM0_CMSDK_Driver_definitions CMSDK Driver definitions
  This file defines all CMSDK Driver functions for CMSIS core for the following modules:
    - UART
    - SPI
    - Timer
    - Dual Timer
    - RTC
    - GPIO
  @{
 */

 #include "CMSDK_CM0.h"


 /*UART Driver Declarations*/

  /**
   * @brief Initializes UART module.
   */

 extern void CMSDK_uart_init(CMSDK_UART_TypeDef *CMSDK_UART, uint16_t divider, uint8_t break_en, uint8_t stick_en, uint8_t even_en, uint8_t parity_en, uint8_t stop_len, uint8_t word_len, uint8_t fifo_en);

  /**
   * @brief Sends a character to the UART TX Buffer.
   */

 extern void CMSDK_uart_SendChar(CMSDK_UART_TypeDef *CMSDK_UART, char txchar); 

 /*SPI driver Declarations*/

  /**
   * @brief  Config CTRL1.
   */

 extern void CMSDK_SPI_CTRL1_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t duplex_mode, uint8_t tx_rx_dir, uint8_t baud, uint8_t lsb_sel, uint8_t loopback,
			uint8_t nss_toggle, uint8_t nss_ctrl, uint8_t nss_sw_input, uint8_t cpol, uint8_t cpha, uint8_t mst_slv_sel, uint8_t spi_en);
  /**
   * @brief  Config CTRL2.
   */

 extern void CMSDK_SPI_CTRL2_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t t2c_delay, uint8_t c2t_delay, uint8_t nss_en, uint8_t samp_phase, uint8_t tx_dma_en, uint8_t rx_dma_en, uint8_t char_len);

  /**
   * @brief  Config FCR.
   */

 extern void CMSDK_SPI_FCR_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t tx_fifo_th, uint8_t rx_fifo_th, uint8_t tx_fifo_clr, uint8_t rx_fifo_clr, uint8_t fifo_en);

 /*Timer Driver Declarations*/

  /**
   * @brief Set CMSDK Timer for multi-shoot mode with internal clock
   */

 extern void CMSDK_timer_Init(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload, uint32_t irq_en);

  /**
   * @brief Set CMSDK Timer for multi-shoot mode with external clock
   */

 extern void CMSDK_timer_Init_ExtClock(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload,
 uint32_t irq_en);


  /**
   * @brief Set CMSDK Timer for multi-shoot mode with external enable
   */

 extern void CMSDK_timer_Init_ExtEnable(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload,
 uint32_t irq_en);

  /**
   * @brief CMSDK Timer interrupt clear
   */

 extern void CMSDK_timer_ClearIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Returns timer IRQ status
   */

 uint32_t  CMSDK_timer_StatusIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Returns Timer Reload value.
   */

 extern uint32_t CMSDK_timer_GetReload(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Sets Timer Reload value.
   */

 extern void CMSDK_timer_SetReload(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t value);

  /**
   * @brief Returns Timer current value.
   */

 uint32_t CMSDK_timer_GetValue(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Sets Timer current value.
   */

 extern void CMSDK_timer_SetValue(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t value);

  /**
   * @brief Stops CMSDK Timer.
   */

 extern void CMSDK_timer_StopTimer(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Starts CMSDK Timer.
   */

 extern void CMSDK_timer_StartTimer(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Enables CMSDK Timer Interrupt requests.
   */

 extern void CMSDK_timer_EnableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Disables CMSDK Timer Interrupt requests.
   */

 extern void CMSDK_timer_DisableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER); 

 /*DUAL Timer Driver Declarations*/

  /**
   * @brief Setup Free running mode.
   */

 extern void CMSDK_dualtimer_setup_freerunning(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx, unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size);

  /**
   * @brief Setup Periodic mode.
   */

 extern void CMSDK_dualtimer_setup_periodic(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx, unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size);

  /**
   * @brief Setup One shot mode.
   */

 extern void CMSDK_dualtimer_setup_oneshot(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx, unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size);

  /**
   * @brief Start Timer.
   */

 extern void CMSDK_dualtimer_start(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx);

  /**
   * @brief Start Timer.
   */

 extern void CMSDK_dualtimer_stop(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx);

  /**
   * @brief Clear Interrupt.
   */

 extern void CMSDK_dualtimer_irq_clear(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx); 

 /*RTC driver Declarations*/

  /**
   * @brief  Init Calender.
   */

 extern void CMSDK_RTC_Init_Calender(uint16_t prescaler, uint8_t data_mode, uint8_t hour_mode, uint32_t init_time, uint32_t init_date);

  /**
   * @brief  Config Alarm.
   */

 extern void CMSDK_RTC_Config_Alarm(uint16_t prescaler, uint8_t data_mode, uint8_t hour_mode, uint32_t init_time, uint32_t init_date, uint32_t alarm_time, uint32_t alarm_date);

  /**
   * @brief  Config Periodic Wakeup.
   */

 extern void CMSDK_RTC_Config_PeriodWake(uint8_t clock_sel, uint16_t prescaler, uint16_t period_time);

 
  // /*ADC CTRL Driver Declarations*/
 //
    /**
    * @brief  Config ADC_CTRL REG
    */

 extern void CMSDK_ADC_SetADC_EN_AND_STARTBIT(uint8_t value1, uint8_t value2);
   
  /**
   * @brief  Config ADC Interrupt Enable Register
   */

 extern void CMSDK_ADC_INT_EN(uint8_t value1,uint8_t value2);
    
  /**
   * @brief  Config ADC Config register to set single/contineous/wait mode
   */

 extern void CMSDK_ADC_SetCONT_CONV_MODE_EN(uint8_t value);
        
  /**
   * @brief  Config ADC Config register to set single/contineous/wait mode
   */

 extern void CMSDK_SETADC_CONV_MODE_EN(uint8_t value);
    
  /**
   * @brief  Disable ADC_EN of ADC_CTRL register
   */


 extern void CMSDK_ADC_DISABLE_ADC_EN_BIT();
    
  /**
   * @brief  Config ADC Clock Divider Value
   */


 extern void CMSDK_SET_ADCCLK_DIV_VAL(uint8_t value);
    
  /**
   * @brief  Config ADC Sampling Time(sample rst time)
   */
 


 extern void CMSDK_SET_ADCSAMP_TIME(uint8_t value);
    
  /**
   * @brief  Config ADC Channel Select
   */



 extern void CMSDK_SET_ADC_CHANNEL_SEL(uint8_t value);
    
  /**
   * @brief  Config ADC EOC CONFIG bit
   */


 extern void CMSDK_SETADC_CONVINC_EOC(uint8_t value);
    
  /**
   * @brief  Config ADC Wait Mode
   */


 extern void CMSDK_ADC_Set_WAIT_MODE_EN(uint8_t value);
    
  /**
   * @brief  Config ADC Overrun Mode
   */


 extern void CMSDK_ADC_Set_OVERRUN_MODE_EN(uint8_t value);

 // /*SYSTEM CTRL Driver Declarations*/
 //
    /**
    * @brief  Config SYSCTRL Clock Config Reg(AHB prescalar)
    */

 extern void CMSDK_SYSCON_SetHCLK_DIV(uint8_t value);

   /**
    * @brief  Config System Clock Freq of SYSCTRL HSICTRL Reg 
    */

 extern void CMSDK_SYSCON_SetHSI_CTRL(uint8_t value);

   /**
    * @brief  Config PCLK Divider Value of Clock Config Reg
    * */

 extern void CMSDK_SYSCON_SetPCLK_DIV(uint8_t value);

   /**
    * @brief  Config SYSCTRL CLOCK CONFIG Reg
    */
 extern void CMSDK_SYSCON_SetSYSCLK_SEL(uint8_t value);

  /**
    * @brief  Config SYSCTRL HSICTRL Reg
    */

 extern void CMSDK_SYSCON_SetHSI_EN(uint8_t value);

  /**
    * @brief  Config SYSCTRL CLOCK CONFIG REG
    */ 

 extern void CMSDK_SYSCON_SetLFCLK_SEL(uint8_t value);

  /**
    * @brief  Config SYSTRL LSE CTRL Reg
    */

 extern void CMSDK_SYSCON_SetLSE_EN(uint8_t value);

  /**
    * @brief  Config SYSCTRL LSI CTRL Reg
    */

 extern void CMSDK_SYSCON_DisableLSI_EN();

  /**
    * @brief  Config SYSCTRL CLOCK CONFIG Reg
    */

 extern void CMSDK_SYSCON_SetMCO_SEL(uint8_t value);
 


// /*GPIO Driver Declarations*/
//
//  /**
//   * @brief Set CMSDK GPIO Output Enable.
//   */
//
// extern void CMSDK_gpio_SetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableset);
//
//  /**
//   * @brief Clear CMSDK GPIO Output Enable.
//   */
//
// extern void CMSDK_gpio_ClrOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableclr);
//
//  /**
//   * @brief Returns CMSDK GPIO Output Enable.
//   */
//
// extern uint32_t CMSDK_gpio_GetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO);
//
//  /**
//   * @brief Set CMSDK GPIO Alternate function Enable.
//   */
//
// extern void CMSDK_gpio_SetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncset);
//
//  /**
//   * @brief Clear CMSDK GPIO Alternate function Enable.
//   */
//
// extern void CMSDK_gpio_ClrAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncclr);
//
//  /**
//   * @brief Returns CMSDK GPIO Alternate function Enable.
//   */
//
// extern uint32_t CMSDK_gpio_GetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO);
//
//  /**
//   * @brief Clear CMSDK GPIO Interrupt request.
//   */
//
// extern uint32_t CMSDK_gpio_IntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//  /**
//   * @brief Enable CMSDK GPIO Interrupt request.
//   */
//
// extern uint32_t CMSDK_gpio_SetIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//  /**
//   * @brief Disable CMSDK GPIO Interrupt request.
//   */
//
// extern uint32_t CMSDK_gpio_ClrIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//  /**
//   * @brief Setup CMSDK GPIO Interrupt as high level.
//   */
//
// extern void CMSDK_gpio_SetIntHighLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//   /**
//   * @brief Setup CMSDK GPIO Interrupt as rising edge.
//   */
//
// extern void CMSDK_gpio_SetIntRisingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//     /**
//   * @brief Setup CMSDK GPIO Interrupt as low level.
//   */
//
// extern void CMSDK_gpio_SetIntLowLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
//
//    /**
//   * @brief Setup CMSDK GPIO Interrupt as falling edge.
//   */
//
// extern void CMSDK_gpio_SetIntFallingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
 

  /*@}*/ /* end of group CMSIS_CM0_CMSDK_Driver_definitions CMSDK Driver definitions */
