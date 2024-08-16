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
    - Timer
    - UART
    - GPIO
  @{
 */

 #include "CMSDK_CM0.h"


 /*UART Driver Declarations*/

  /**
   * @brief Initializes UART module.
   */

 extern uint32_t CMSDK_uart_init(CMSDK_UART_TypeDef *CMSDK_UART, uint32_t divider, uint32_t tx_en,
                           uint32_t rx_en, uint32_t tx_irq_en, uint32_t rx_irq_en, uint32_t tx_ovrirq_en, uint32_t rx_ovrirq_en);

  /**
   * @brief Returns whether the UART RX Buffer is Full.
   */

 extern uint32_t CMSDK_uart_GetRxBufferFull(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Returns whether the UART TX Buffer is Full.
   */

 extern uint32_t CMSDK_uart_GetTxBufferFull(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Sends a character to the UART TX Buffer.
   */


 extern void CMSDK_uart_SendChar(CMSDK_UART_TypeDef *CMSDK_UART, char txchar);

  /**
   * @brief Receives a character from the UART RX Buffer.
   */

 extern char CMSDK_uart_ReceiveChar(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Returns UART Overrun status.
   */

 extern uint32_t CMSDK_uart_GetOverrunStatus(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Clears UART Overrun status Returns new UART Overrun status.
   */

 extern uint32_t CMSDK_uart_ClearOverrunStatus(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Returns UART Baud rate Divider value.
   */

 extern uint32_t CMSDK_uart_GetBaudDivider(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Return UART TX Interrupt Status.
   */

 extern uint32_t CMSDK_uart_GetTxIRQStatus(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Return UART RX Interrupt Status.
   */

 extern uint32_t CMSDK_uart_GetRxIRQStatus(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Clear UART TX Interrupt request.
   */

 extern void CMSDK_uart_ClearTxIRQ(CMSDK_UART_TypeDef *CMSDK_UART);

  /**
   * @brief Clear UART RX Interrupt request.
   */

 extern void CMSDK_uart_ClearRxIRQ(CMSDK_UART_TypeDef *CMSDK_UART);
 

 /*Timer Driver Declarations*/

 /**
   * @brief Set CMSDK Timer for multi-shoot mode with internal clock (Timer_0/1/2/3)
   */
 extern void CMSDK_timer_Init(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload, uint32_t irq_en, uint32_t wake_en);

  /**
   * @brief Set CMSDK Timer for multi-shoot mode with external enable (Timer_0 only)
   */

 extern void CMSDK_timer_Init_ExtClock(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload,
 uint32_t irq_en);


  /**
   * @brief Set CMSDK Timer for multi-shoot mode with external clock (Timer_0 only)
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
   * @brief Reset CMSDK Timer 1Hz (1/2/3).
   */

 extern void CMSDK_timer_ResetTimer1Hz(CMSDK_TIMER_TypeDef *CMSDK_TIMER);
 
  /**
   * @brief Clear Reset CMSDK Timer 1Hz (1/2/3).
   */

 extern void CMSDK_timer_ClrResetTimer1Hz(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Enables CMSDK Timer Interrupt requests.
   */

 extern void CMSDK_timer_EnableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Disables CMSDK Timer Interrupt requests.
   */

 extern void CMSDK_timer_DisableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Enables CMSDK Timer wakeup from deepsleep.
   */

 extern void CMSDK_timer_EnableWAKE(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

  /**
   * @brief Disables CMSDK Timer wakeup from deepsleep.
   */

 extern void CMSDK_timer_DisableWAKE(CMSDK_TIMER_TypeDef *CMSDK_TIMER); 

 /**
   * @brief Disables CMSDK Timer wakeup from deepsleep.
   */

 extern void CMSDK_timer_Stop_EXTEN_Timer(CMSDK_TIMER_TypeDef *CMSDK_TIMER);

/* SYSCON Driver Declarations*/
/*
 * @brief  configures AHB CLK divider value.
 */
 extern void CMSDK_SYSCON_SetHCLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value);
/*
 * @brief  configures the peripheral CLK divider value.
 */
 extern void CMSDK_SYSCON_SetPCLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value);
/*
 * @brief  configures IMEAS ADC CLK divider value.
 */
 extern void CMSDK_SYSCON_SetICLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value);
/*
 * @brief  Enable NFC POR wake from deepsleep.
 */
 extern void CMSDK_SYSCON_EnableNFC_WAKE(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Disable NFC POR wake from deepsleep.
 */
 extern void CMSDK_SYSCON_DisableNFC_WAKE(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Enable NFC POR interrupt.
 */
 extern void CMSDK_SYSCON_EnableNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Disable NFC POR interrupt.
 */
 extern void CMSDK_SYSCON_DisableNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get NFC POR interrupt status.
 */
 extern uint32_t CMSDK_SYSCON_GetstatusNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Clear NFC POR interrupt.
 */
 extern void CMSDK_SYSCON_ClearNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get REMAP value.
 */
 extern uint32_t CMSDK_SYSCON_GetREMAP(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Set REMAP to select SRAM.
 */
 extern void CMSDK_SYSCON_SetREMAP_SRAM(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Set REMAP to select PFLASH.
 */
 extern void CMSDK_SYSCON_SetREMAP_PFLASH(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Set REMAP to select DFLASH.
 */
 extern void CMSDK_SYSCON_SetREMAP_DFLASH(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Config PMUCTRL for EXTERNAL WAKE Function.
 */
 extern uint32_t CMSDK_SYSCON_PMUCTRL_EXT_WAKE_Config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get EXTERNAL WAKE STATUS bit.
 */
 extern uint32_t CMSDK_SYSCON_GetstatusEXT_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Clear EXTERNAL WAKE STATUS bit.
 */
 extern void CMSDK_SYSCON_ClearEXT_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Config PMUCTRL for PSW WAKE Function.
 */
 extern uint32_t CMSDK_SYSCON_PMUCTRL_PSW_WAKE_Config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get PSW WAKE STATUS bit.
 */
 extern uint32_t CMSDK_SYSCON_GetstatusPSW_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Clear PSW WAKE STATUS bit.
 */
 extern void CMSDK_SYSCON_ClearPSW_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  initialize System control register of field ANACTRL for bat_off.
 */
 extern void CMSDK_SYSCON_ANACTRL_BATOFF_config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get status of System control register of field ANACTRL for bat_off.
 */
 extern uint32_t CMSDK_SYSCON_GetANACTRL_BATOFF_Status(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  initialize System control register of field ANACTRL for selecting Current Measurment mode.
 */ 
 extern void CMSDK_SYSCON_ANACTRL_MEASMODE_IMEASselect(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  initialize System control register of field ANACTRL for selecting Current Measurment mode.
 */
 extern void CMSDK_SYSCON_ANACTRL_MEASMODE_ZMEASselect(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);
/*
 * @brief  Get status of System control register of field ANACTRL for MEAS_MODE_SEL.
 */
 extern uint32_t CMSDK_SYSCON_GetANACTRL_MEASMODE_Status(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON);

/* IMEAS Driver Declarations*/
/*
 * @brief  configures IMEAS control register.
 */
 extern void CMSDK_IMEAS_config(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en);
/*
 * @brief  Enable IMEAS interrupt.
 */
 extern void CMSDK_IMEAS_EnableIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Disable IMEAS interrupt.
 */
 extern void CMSDK_IMEAS_DisableIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Get IMEAS INTSTATUS.
 */
 extern uint32_t CMSDK_IMEAS_GetInterrupt_Status(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Clear IMEAS interrupt.
 */
 extern void CMSDK_IMEAS_ClearIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  set IMEAS channel mode.
 */
 extern void CMSDK_IMEAS_SetchannelMODE(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t cha_mode);
/*
 * @brief  get IMEAS channel mode.
 */
 extern uint32_t CMSDK_IMEAS_GetchannelMODE(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  set IMEAS channel number.
 */
 extern void CMSDK_IMEAS_SetchannelNUM(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t cha_num);
/*
 * @brief  set IMEAS channel format.
 */
 extern void CMSDK_IMEAS_SetchannelFORMAT(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t format_sel);
/*
 * @brief  Enable imeas A/D Software reset control.
 */
 extern void CMSDK_IMEAS_EnableSD16RST(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Disable imeas A/D Software reset control.
 */
 extern void CMSDK_IMEAS_DisableSD16RST(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Enable imeas A/D Sleep mode control.
 */
 extern void CMSDK_IMEAS_EnableSD16SLP(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Disable imeas A/D Sleep mode control.
 */
 extern void CMSDK_IMEAS_DisableSD16SLP(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Enable imeas A/D power control.
 */
 extern void CMSDK_IMEAS_EnableSD16OFF(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Disable imeas A/D power control.
 */
 extern void CMSDK_IMEAS_DisableSD16OFF(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Get imeas CH0_DATA.
 */
 extern uint32_t CMSDK_IMEAS_GetCH0_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Get imeas CH1_DATA.
 */
 extern uint32_t CMSDK_IMEAS_GetCH1_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);
/*
 * @brief  Get imeas CH2_DATA.
 */
 extern uint32_t CMSDK_IMEAS_GetCH2_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS);

/*
 * Initializes & start Measurement, single mode
 */
 extern uint32_t CMSDK_IMEAS_InitandStart_SingleChannelMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel, uint8_t cha_num);

/*
 * Initializes & start Measurement, single continous mode
 */
 extern uint32_t CMSDK_IMEAS_InitandStart_SingleChannelContinousMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel, uint8_t cha_num);

/*
 * Initializes & start Measurement, group mode
 */
 extern uint32_t CMSDK_IMEAS_InitandStart_GroupChannelMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel);

/* ZMEAS Driver Declarations*/
/*
 * @brief Zmeas configuaration of number of repeat cycle, settling time, reg frequency value
   *mode of operation. repeat enable, zmeas interruot enable, adc interrupt enable,
   *calibration/measurement enable, config voltage value, pga gain, clock, reset 
 */
 extern  void CMSDK_ZMEAS_config(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_number_of_repeat_cycle_val, uint8_t reg_settling_time_val, uint8_t reg_freq_val, uint8_t reg_mode,
			uint8_t repeat_calculation_en, uint8_t enable_intr, uint8_t enable_adc_intr, uint8_t measure_calibrate, uint8_t config_output_voltage_range, 
			uint8_t pga_gain, uint8_t Reset_measurement);
/*
 * @brief set MODE among "standby, power down, no operation, initialization, start calculation". 
 *
 */
 extern void CMSDK_ZMEAS_Setmode(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_mode);

/*
 * ZMEAS calculation finshed ZMEAS output data
 * provided in REG_DATA_OUT
 * REG_DATAOUT contain Imaginary value 
 * and Real value
 */   
 extern uint32_t CMSDK_ZMEAS_GetREG_DATAOUT(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 /*
 * ZMEAS REG_STATUS provide staus of configuaration 
 * at INIT mode and calculation completion
 * status also valid real and imaginary data
 *
 */ 
 extern uint32_t CMSDK_ZMEAS_GetREG_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetMeasDone_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 extern uint32_t CMSDK_ZMEAS_GetADC_enable_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetDDs_enable_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetPGA_gain_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_Getmeasure_or_calibrate_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_Getconfig_volt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetFreq_val_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetRepeat_val_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS ADC_ROM_REG register provides cosine
 * value, sine value and SARADC data
 * of 10bit each
 *
 */ 
 extern uint32_t CMSDK_ZMEAS_GetADC_DATA_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetSINE_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetCOSINE_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS SUMMATION_OFFSET register field provides 
 * 29bit summation_offset_foreal value
 *
 */ 
 extern uint32_t CMSDK_ZMEAS_GetSummation_Offset_Forreal(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS SUMMATION_REAL register field provides 
 * 29bit SUMMATION_REAL value for back up
 * calculation
 */ 
 extern uint32_t CMSDK_ZMEAS_GetSummation_Real(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS SUMMATION_IMAG register field provides 
 * 29bit SUMMATION_IMAG value for back up
 * calculation
 *
 */
 extern uint32_t CMSDK_ZMEAS_GetSummation_Imag(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS DFT_CNT_REG register field provides 
 * DFT sample count value and 17 bit shifted
 * real_inter value.
 *
 */
 extern uint32_t CMSDK_ZMEAS_GetDFT_CNT_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 extern uint32_t CMSDK_ZMEAS_GetSHIFTED_REAL_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);
 
 /*
 * ZMEAS INTSTATUS register field provides 
 * status of zmeas interrupt
 *
 */
 extern uint32_t CMSDK_ZMEAS_GetInterrupt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 /*
 * Whenever zmeas interrupt occurs  
 * clear zmeas interrupt
 */ 
 extern void CMSDK_ZMEAS_ClearIRQ(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 /*
 * ZMEAS ADCINTSTATUS register field provides 
 * status of zmeas ADC interrupt
 *
 */
 extern uint32_t CMSDK_ZMEAS_GetADCInterrupt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 /*
 * Whenever zmeas adc interrupt occurs  
 * clear zmeas adc interrupt
 */ 
 extern void CMSDK_ZMEAS_ClearADCIRQ(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS);

 /*
 * Initializes & start Measurement/Calibration
 */
 extern uint32_t CMSDK_ZMEAS_InitandStartMeas(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_number_of_repeat_cycle_val, uint8_t reg_settling_time_val, 
			               uint8_t reg_freq_val, uint8_t repeat_calculation_en, uint8_t enable_intr, uint8_t enable_adc_intr, 
                                       uint8_t measure_calibrate, uint8_t config_output_voltage_range, uint8_t pga_gain, uint8_t Reset_measurement);
/*FLASH driver Declarations*/
/*
 * @brief  Enable NVR.
 */
 extern void CMSDK_FLASH_EnableNVR(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Disable NVR.
 */
 extern void CMSDK_FLASH_DisableNVR(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Enable Program.
 */
 extern void CMSDK_FLASH_EnablePG(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Disable Program.
 */
 extern void CMSDK_FLASH_DisablePG(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Enable Sector Erase.
 */
 extern void CMSDK_FLASH_EnableSER(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Disable Sector Erase.
 */
 extern void CMSDK_FLASH_DisableSER(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Enable Chip Erase.
 */
 extern void CMSDK_FLASH_EnableCER(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Disable Chip Erase.
 */
 extern void CMSDK_FLASH_DisableCER(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Set Full word.
 */
 extern void CMSDK_FLASH_SetFullWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Set Upper Half word.
 */
 extern void CMSDK_FLASH_SetUPPHalfWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Set Lower Half word.
 */
 extern void CMSDK_FLASH_SetLOWHalfWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Set ADDR.
 */
 extern void CMSDK_FLASH_SetAddress(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t addr);
/*
 * @brief  Set WDATA.
 */
 extern void CMSDK_FLASH_SetWData(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t wdata);
/*
 * @brief  Get CMD value.
 */
 extern uint32_t CMSDK_FLASH_GetCMDvalue(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Goto IDLE.
 */
 extern void CMSDK_FLASH_GotoIDLE(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Goto INIT.
 */
 extern void CMSDK_FLASH_GotoINIT(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Goto START.
 */
 extern void CMSDK_FLASH_GotoSTART(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Get PBusy status.
 */
 extern uint32_t CMSDK_FLASH_GetPBusystatus(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Get PDone status.
 */
 extern uint32_t CMSDK_FLASH_GetPDonestatus(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  Get Sector number.
 */
 extern uint32_t CMSDK_FLASH_GetSectornum(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief Configure flash control to process a write , & wait until the write is done.
 */
 extern void CMSDK_FLASH_status_check(CMSDK_FLASH_TypeDef *CMSDK_FLASH);
/*
 * @brief  set flash MAGIC WORD.
 */
 extern void CMSDK_FLASH_SetMAGICWORD(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t value);
/*
 * @brief  unlock flash BADSECTOR registers.
 */
 extern void CMSDK_FLASH_unlockBADSECTORregs(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t value);
/*
 * @brief  initialize BADSECTOR registers.
 */
 extern void badsector_initialize(CMSDK_FLASH_TypeDef *CMSDK_FLASH);

 /*GPIO Driver Declarations*/

  /**
   * @brief Set CMSDK GPIO Input Enable.
   */

 extern uint32_t CMSDK_gpio_SetInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t inenableset);

  /**
   * @brief Clear CMSDK GPIO Input Enable.
   */

 extern void CMSDK_gpio_ClrInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t inenableclr);
 
 /**
 * @brief Get INPUT Enable register
 */

 extern uint32_t CMSDK_gpio_GetInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Get Open_DRAINEN register
 */

 extern void CMSDK_gpio_SetOpen_DRAINEN(CMSDK_GPIO_TypeDef *CMSDK_GPIO,uint32_t opendrain);
 
 /**
 * @brief Get Open_DRAINEN register
 */

 extern uint32_t CMSDK_gpio_GetOpen_DRAINEN(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Set DATATOPAD register value
 */

 extern uint32_t CMSDK_gpio_SetData_ToPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t datatopad);
 
 /**
 * @brief Get DATATOPAD register value
 */

 extern uint32_t CMSDK_gpio_GetData_ToPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Set PULLUPEN register value
 */

 extern void CMSDK_gpio_SetPull_UpEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t pullupen);

 /**
 * @brief Get PULLUPEN register value
 */

 extern uint32_t CMSDK_gpio_GetPull_UpEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Set PULLDOWNEN register value
 */

 extern void CMSDK_gpio_SetPull_DownEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t pulldownen);

 /**
 * @brief Get PULLDOWNEN register value
 */

 extern uint32_t CMSDK_gpio_GetPull_DownEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO);
 
 /**
 * @brief Set DRVSTRENGTH register value
 */

 extern void CMSDK_gpio_SetDrvStrength(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t drvstrength);

 /**
 * @brief Get DRVSTRENGTH register value
 */

 extern uint32_t CMSDK_gpio_GetDrvStrength(CMSDK_GPIO_TypeDef *CMSDK_GPIO);
 
 /**
 * @brief Get INTTYPESETD register value
 */

 extern uint32_t CMSDK_gpio_GetIntTypeSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 *  @brief Get INTPOLSET register value
 */

 extern uint32_t CMSDK_gpio_GetIntPolSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Get INTENSET register value
 */

 extern uint32_t CMSDK_gpio_GetIntEnSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Get GetIntStatus register value
 */

 extern uint32_t CMSDK_gpio_GetIntStatus(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Get GetIntClear register value
 */

 extern uint32_t CMSDK_gpio_GetIntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO);


 /**
 * @brief Get GetData_FromPad register value
 */

 extern uint32_t CMSDK_gpio_GetData_FromPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Set AE0 register value
 */

 extern void CMSDK_gpio_SetAE0(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t ae0);
 /**
 * @brief Get AE0 register value
 */

 extern uint32_t CMSDK_gpio_GetAE0(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
 * @brief Set AE1 register value
 */

 extern void CMSDK_gpio_SetAE1(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t ae1);
 
 /**
 * @brief Get AE1 register value
 */

 extern uint32_t CMSDK_gpio_GetAE1(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

 /**
   * @brief Set CMSDK GPIO Output Enable.
   */
 
 extern void CMSDK_gpio_SetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableset);

  /**
   * @brief Clear CMSDK GPIO Output Enable.
   */

 extern void CMSDK_gpio_ClrOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableclr);

  /**
   * @brief Returns CMSDK GPIO Output Enable.
   */

 extern uint32_t CMSDK_gpio_GetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

  /**
   * @brief Set CMSDK GPIO Alternate function Enable.
   */

 extern uint32_t CMSDK_gpio_SetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncset);

  /**
   * @brief Clear CMSDK GPIO Alternate function Enable.
   */

 extern void CMSDK_gpio_ClrAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncclr);

  /**
   * @brief Returns CMSDK GPIO Alternate function Enable.
   */

 extern uint32_t CMSDK_gpio_GetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO);

  /**
   * @brief Clear CMSDK GPIO Interrupt request.
   */

 extern uint32_t CMSDK_gpio_IntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);

  /**
   * @brief Enable CMSDK GPIO Interrupt request.
   */

 extern uint32_t CMSDK_gpio_SetIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);

  /**
   * @brief Disable CMSDK GPIO Interrupt request.
   */

 extern uint32_t CMSDK_gpio_ClrIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);
 
   /**
   * @brief Setup CMSDK GPIO Interrupt as rising edge.
   */

 extern void CMSDK_gpio_SetIntRisingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);

     /**
   * @brief Setup CMSDK GPIO Interrupt as low level.
   */

 extern void CMSDK_gpio_SetIntLowLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);

    /**
   * @brief Setup CMSDK GPIO Interrupt as falling edge.
   */

 extern void CMSDK_gpio_SetIntFallingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num);

/*@}*/ /* end of group CMSIS_CM0_CMSDK_Driver_definitions CMSDK Driver definitions */
