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
/*********************************************************************//******
 * @file     CMSDK_driver.c
 * @brief    CMSDK Example Device Driver C File
 * @version  $State:$
 * @date     $Date: 2017-10-10 15:55:38 +0100 (Tue, 10 Oct 2017) $
 *
 ******************************************************************************/

#include "CMSDK_CM0.h"



/** \mainpage ARM CMSDK LIBRARY
 *
 *
 * This user manual describes the ARM Corex M Series CMSDK Library which utilises the
 * Cortex Microcontroller Software Interface Standard (CMSIS). it also includes drivers
 * for the following modules:
 *
 *    - UART
 *    - Timer
 *    - GPIO
 *
 * The library contains C and assembly functions that have been ported and tested on the MDK
 * toolchain.
 */


/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Enable the microcontroller timer interrupts.
 */

 void CMSDK_timer_EnableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL |= CMSDK_TIMER_CTRL_IRQEN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Disable the microcontroller timer interrutps.
 */

 void CMSDK_timer_DisableIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL &= ~CMSDK_TIMER_CTRL_IRQEN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Enable the microcontroller timer wake up from deepsleep mode.
 */

 void CMSDK_timer_EnableWAKE(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL |= CMSDK_TIMER_CTRL_WAKE_EN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Disable the microcontroller timer wake up from deepsleep mode.
 */

 void CMSDK_timer_DisableWAKE(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL &= ~CMSDK_TIMER_CTRL_WAKE_EN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Start the Timer.
 */

 void CMSDK_timer_StartTimer(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL |= CMSDK_TIMER_CTRL_EN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Stop the Timer.
 */

 void CMSDK_timer_StopTimer(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL &= ~CMSDK_TIMER_CTRL_EN_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Reset the Timer (1/2/3).
 */

 void CMSDK_timer_ResetTimer1Hz(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL |= CMSDK_TIMER_CTRL_RESET_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Clear Timer1Hz reset (1/2/3).
 */

 void CMSDK_timer_ClrResetTimer1Hz(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL &= ~CMSDK_TIMER_CTRL_RESET_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return TimerValue
 *
 * @brief  Returns the current value of the timer.
 */

 uint32_t CMSDK_timer_GetValue(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       return CMSDK_TIMER->VALUE;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param value the value to which the timer is to be set
 * @return TimerValue
 *
 * @brief  Sets the timer to the specified value.
 */

 void CMSDK_timer_SetValue(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t value)
 {
       CMSDK_TIMER->VALUE = value;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return TimerReload
 *
 * @brief  Returns the reload value of the timer. The reload value is the value which the timer is set to after an underflow occurs.
 */

 uint32_t CMSDK_timer_GetReload(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       return CMSDK_TIMER->RELOAD;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param value Value to be loaded
 * @return none
 *
 * @brief  Sets the reload value of the timer to the specified value. The reload value is the value which the timer is set to after an underflow occurs.
 */

 void CMSDK_timer_SetReload(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t value)
 {
       CMSDK_TIMER->RELOAD = value;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Clears the timer IRQ if set.
 */

 void CMSDK_timer_ClearIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->INTCLEAR = CMSDK_TIMER_INTCLEAR_Msk;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Returns the IRQ status of the timer in question.
 */

 uint32_t  CMSDK_timer_StatusIRQ(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       return CMSDK_TIMER->INTSTATUS;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param reload The value to which the timer is to be set after an underflow has occurred
 * @param wake_en Defines whether the timer wake up from deepsleep is to be enabled
 * @param irq_en Defines whether the timer IRQ is to be enabled
 * @return none
 * NOTE: while calling this function, Set wake_en = 0 if Timer 0 is being used, as wake_en is used only for timers 1/2/3
 * clears rst(if enabled) to start timer normally
 * @brief  Initialises the timer, specifies the timer current & reload value ,whether wake up enabled or not; and whether IRQ is enabled or not.
 */

  void CMSDK_timer_Init(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload, uint32_t irq_en, uint32_t wake_en)
 {     uint32_t new_ctrl = 0;
       
       CMSDK_TIMER->VALUE = reload;
       CMSDK_TIMER->RELOAD = reload;
       if (irq_en!=0)   new_ctrl |= CMSDK_TIMER_CTRL_IRQEN_Msk;               /* non zero - enable IRQ */
       if (wake_en!=0)  new_ctrl |= CMSDK_TIMER_CTRL_WAKE_EN_Msk;             /* non zero - enable wakeup */
                        new_ctrl |= CMSDK_TIMER_CTRL_EN_Msk;                  /* enable timer */
       CMSDK_TIMER->CTRL = new_ctrl;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param reload The value to which the timer is to be set after an underflow has occurred
 * @param irq_en Defines whether the timer IRQ is to be enabled
 * @return none
 * Timer 0 only
 * @brief  Initialises the timer to use the external clock and specifies the timer reload value and whether IRQ is enabled or not.
 */

 void CMSDK_timer_Init_ExtClock(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload,
 uint32_t irq_en)
 {
       CMSDK_TIMER->CTRL = 0;
       CMSDK_TIMER->VALUE = reload;
       CMSDK_TIMER->RELOAD = reload;
       if (irq_en!=0)                                                                                  /* non zero - enable IRQ */
            CMSDK_TIMER->CTRL = (CMSDK_TIMER_CTRL_IRQEN_Msk |
                                   CMSDK_TIMER_CTRL_SELEXTCLK_Msk |CMSDK_TIMER_CTRL_EN_Msk);
       else  {                                                                                         /* zero - do not enable IRQ */
            CMSDK_TIMER->CTRL = ( CMSDK_TIMER_CTRL_EN_Msk |
                                    CMSDK_TIMER_CTRL_SELEXTCLK_Msk);                                   /* enable timer */
         }
 }

/**
 *
 * @brief  Initialises the timer to use the internal clock but with an external enable. It also specifies the timer reload value and whether IRQ is enabled or not.
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param reload The value to which the timer is to be set after an underflow has occurred
 * @param irq_en Defines whether the timer IRQ is to be enabled
 * @return none
 * Timer 0 only
 *
 */

 void CMSDK_timer_Init_ExtEnable(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload,
 uint32_t irq_en)
 {
       CMSDK_TIMER->CTRL = 0;
       CMSDK_TIMER->VALUE = reload;
       CMSDK_TIMER->RELOAD = reload;
       if (irq_en!=0)                                                                                  /* non zero - enable IRQ */
            CMSDK_TIMER->CTRL = (CMSDK_TIMER_CTRL_IRQEN_Msk |
                                   CMSDK_TIMER_CTRL_SELEXTEN_Msk | CMSDK_TIMER_CTRL_EN_Msk);
       else  {                                                                                         /* zero - do not enable IRQ */
            CMSDK_TIMER->CTRL = ( CMSDK_TIMER_CTRL_EN_Msk |
                                    CMSDK_TIMER_CTRL_SELEXTEN_Msk);                                    /* enable timer */
         }
 }

 /**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @return none
 *
 * @brief  Stop the Extern Timer.
 */

 void CMSDK_timer_Stop_EXTEN_Timer(CMSDK_TIMER_TypeDef *CMSDK_TIMER)
 {
       CMSDK_TIMER->CTRL &= ~CMSDK_TIMER_CTRL_SELEXTEN_Msk;
 }

  /*UART driver functions*/

/**
 *
 * @brief  Initialises the UART specifying the UART Baud rate divider value and whether the send and recieve functionality is enabled. It also specifies which of the various interrupts are enabled.
 *
 * @param *CMSDK_UART UART Pointer
 * @param divider The value to which the UART baud rate divider is to be set
 * @param tx_en Defines whether the UART transmit is to be enabled
 * @param rx_en Defines whether the UART receive is to be enabled
 * @param tx_irq_en Defines whether the UART transmit buffer full interrupt is to be enabled
 * @param rx_irq_en Defines whether the UART receive buffer full interrupt is to be enabled
 * @param tx_ovrirq_en Defines whether the UART transmit buffer overrun interrupt is to be enabled
 * @param rx_ovrirq_en Defines whether the UART receive buffer overrun interrupt is to be enabled
 * @return 1 if initialisation failed, 0 if successful.
 */

 uint32_t CMSDK_uart_init(CMSDK_UART_TypeDef *CMSDK_UART, uint32_t divider, uint32_t tx_en,
                           uint32_t rx_en, uint32_t tx_irq_en, uint32_t rx_irq_en, uint32_t tx_ovrirq_en, uint32_t rx_ovrirq_en)
 {
       uint32_t new_ctrl=0;

       if (tx_en!=0)        new_ctrl |= CMSDK_UART_CTRL_TXEN_Msk;
       if (rx_en!=0)        new_ctrl |= CMSDK_UART_CTRL_RXEN_Msk;
       if (tx_irq_en!=0)    new_ctrl |= CMSDK_UART_CTRL_TXIRQEN_Msk;
       if (rx_irq_en!=0)    new_ctrl |= CMSDK_UART_CTRL_RXIRQEN_Msk;
       if (tx_ovrirq_en!=0) new_ctrl |= CMSDK_UART_CTRL_TXORIRQEN_Msk;
       if (rx_ovrirq_en!=0) new_ctrl |= CMSDK_UART_CTRL_RXORIRQEN_Msk;

       CMSDK_UART->CTRL = 0;         /* Disable UART when changing configuration */
       CMSDK_UART->BAUDDIV = divider;
       CMSDK_UART->CTRL = new_ctrl;  /* Update CTRL register to new value */

       if((CMSDK_UART->STATE & (CMSDK_UART_STATE_RXOR_Msk | CMSDK_UART_STATE_TXOR_Msk))) return 1;
       else return 0;
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return RxBufferFull
 *
 * @brief  Returns whether the RX buffer is full.
 */

 uint32_t CMSDK_uart_GetRxBufferFull(CMSDK_UART_TypeDef *CMSDK_UART)
 {
        return ((CMSDK_UART->STATE & CMSDK_UART_STATE_RXBF_Msk)>> CMSDK_UART_STATE_RXBF_Pos);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return TxBufferFull
 *
 * @brief  Returns whether the TX buffer is full.
 */

 uint32_t CMSDK_uart_GetTxBufferFull(CMSDK_UART_TypeDef *CMSDK_UART)
 {
        return ((CMSDK_UART->STATE & CMSDK_UART_STATE_TXBF_Msk)>> CMSDK_UART_STATE_TXBF_Pos);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @param txchar Character to be sent
 * @return none
 *
 * @brief  Sends a character to the TX buffer for transmission.
 */

 void CMSDK_uart_SendChar(CMSDK_UART_TypeDef *CMSDK_UART, char txchar)
 {
       while(CMSDK_UART->STATE & CMSDK_UART_STATE_TXBF_Msk);
       CMSDK_UART->DATA = (uint32_t)txchar;
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return rxchar
 *
 * @brief  returns the character from the RX buffer which has been received.
 */

 char CMSDK_uart_ReceiveChar(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       while(!(CMSDK_UART->STATE & CMSDK_UART_STATE_RXBF_Msk));
       return (char)(CMSDK_UART->DATA);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return 0 - No overrun
 * @return 1 - TX overrun
 * @return 2 - RX overrun
 * @return 3 - TX & RX overrun
 *
 * @brief  returns the current overrun status of both the RX & TX buffers.
 */


 uint32_t CMSDK_uart_GetOverrunStatus(CMSDK_UART_TypeDef *CMSDK_UART)
 {
        return ((CMSDK_UART->STATE & (CMSDK_UART_STATE_RXOR_Msk | CMSDK_UART_STATE_TXOR_Msk))>>CMSDK_UART_STATE_TXOR_Pos);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return 0 - No overrun
 * @return 1 - TX overrun
 * @return 2 - RX overrun
 * @return 3 - TX & RX overrun
 *
 * @brief  Clears the overrun status of both the RX & TX buffers and then returns the current overrun status.
 */

 uint32_t CMSDK_uart_ClearOverrunStatus(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       CMSDK_UART->STATE = (CMSDK_UART_STATE_RXOR_Msk | CMSDK_UART_STATE_TXOR_Msk);
        return ((CMSDK_UART->STATE & (CMSDK_UART_STATE_RXOR_Msk | CMSDK_UART_STATE_TXOR_Msk))>>CMSDK_UART_STATE_TXOR_Pos);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return BaudDiv
 *
 * @brief  Returns the current UART Baud rate divider. Note that the Baud rate divider is the difference between the clock frequency and the Baud frequency.
 */

 uint32_t CMSDK_uart_GetBaudDivider(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       return CMSDK_UART->BAUDDIV;
 }

 /**
 *
 * @param *CMSDK_UART UART Pointer
 * @return TXStatus
 *
 * @brief  Returns the TX interrupt status.
 */

 uint32_t CMSDK_uart_GetTxIRQStatus(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       return ((CMSDK_UART->INTSTATUS & CMSDK_UART_CTRL_TXIRQ_Msk)>>CMSDK_UART_CTRL_TXIRQ_Pos);
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return RXStatus
 *
 * @brief  Returns the RX interrupt status.
 */

 uint32_t CMSDK_uart_GetRxIRQStatus(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       return ((CMSDK_UART->INTSTATUS & CMSDK_UART_CTRL_RXIRQ_Msk)>>CMSDK_UART_CTRL_RXIRQ_Pos);
 }

 /**
 *
 * @param *CMSDK_UART UART Pointer
 * @return none
 *
 * @brief  Clears the TX buffer full interrupt status.
 */

 void CMSDK_uart_ClearTxIRQ(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       CMSDK_UART->INTCLEAR = CMSDK_UART_CTRL_TXIRQ_Msk;
 }

/**
 *
 * @param *CMSDK_UART UART Pointer
 * @return none
 *
 * @brief  Clears the RX interrupt status.
 */

 void CMSDK_uart_ClearRxIRQ(CMSDK_UART_TypeDef *CMSDK_UART)
 {
       CMSDK_UART->INTCLEAR = CMSDK_UART_CTRL_RXIRQ_Msk;
 }

 /*SYSTEM CONTROL driver functions*/

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  configures AHB CLK divider value.
 */

 void CMSDK_SYSCON_SetHCLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value)
 {
       CMSDK_SYSCON->CLKDIV = ((CMSDK_SYSCON->CLKDIV & ~CMSDK_SYSCON_HCLK_DIV_Msk) | (value << CMSDK_SYSCON_HCLK_DIV_Pos));
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  configures the peripheral CLK divider value.
 */

 void CMSDK_SYSCON_SetPCLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value)
 {
       CMSDK_SYSCON->CLKDIV = ((CMSDK_SYSCON->CLKDIV & ~CMSDK_SYSCON_PCLK_DIV_Msk) | (value << CMSDK_SYSCON_PCLK_DIV_Pos));
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  configures IMEAS ADC CLK divider value.
 */

 void CMSDK_SYSCON_SetICLK_DIV(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON, uint8_t value)
 {
       CMSDK_SYSCON->CLKDIV = ((CMSDK_SYSCON->CLKDIV & ~CMSDK_SYSCON_IMEAS_ADC_CLK_DIV_Msk) | (value << CMSDK_SYSCON_IMEAS_ADC_CLK_DIV_Pos));
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  Enable NFC POR wake from deepsleep.
 */

 void CMSDK_SYSCON_EnableNFC_WAKE(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCTRL |= CMSDK_SYSCON_NFC_POR_WAKE_EN_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  Disable NFC POR wake from deepsleep.
 */

 void CMSDK_SYSCON_DisableNFC_WAKE(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCTRL &= ~CMSDK_SYSCON_NFC_POR_WAKE_EN_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  Enable NFC POR interrupt.
 */

 void CMSDK_SYSCON_EnableNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCTRL |= CMSDK_SYSCON_NFC_POR_INT_EN_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief  Disable NFC POR interrupt.
 */

 void CMSDK_SYSCON_DisableNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCTRL &= ~CMSDK_SYSCON_NFC_POR_INT_EN_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Get NFC POR interrupt status.
 */

 uint32_t CMSDK_SYSCON_GetstatusNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return ((CMSDK_SYSCON->PMUSTATUS & CMSDK_SYSCON_NFC_POR_STS_Msk) >> CMSDK_SYSCON_NFC_POR_STS_Pos);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Clear NFC POR interrupt.
 */

 void CMSDK_SYSCON_ClearNFC_IRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCLEAR |= CMSDK_SYSCON_NFC_POR_STS_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Get REMAP value.
 */

 uint32_t CMSDK_SYSCON_GetREMAP(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return (CMSDK_SYSCON->FUNCEN & CMSDK_SYSCON_REMAP_Msk);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Set REMAP to select SRAM.
 */

 void CMSDK_SYSCON_SetREMAP_SRAM(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->FUNCEN |= (CMSDK_SYSCON_REMAP1_Msk | CMSDK_SYSCON_REMAP0_Msk);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Set REMAP to select PFLASH.
 */

 void CMSDK_SYSCON_SetREMAP_PFLASH(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->FUNCEN = ((CMSDK_SYSCON->FUNCEN & ~CMSDK_SYSCON_REMAP_Msk) | CMSDK_SYSCON_REMAP0_Msk);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Set REMAP to select DFLASH.
 */

 void CMSDK_SYSCON_SetREMAP_DFLASH(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->FUNCEN &= ~CMSDK_SYSCON_REMAP_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Set External Wake interrupt enable bit, 
 * External Wake pin wake enable for deep sleep state, 
 * External Wake trigger mode.
 */

 uint32_t CMSDK_SYSCON_PMUCTRL_EXT_WAKE_Config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
        CMSDK_SYSCON->PMUCTRL |= CMSDK_SYSCON_EXT_WAKE_INTEN_Msk | CMSDK_SYSCON_EXT_WAKE_MOD_Msk | CMSDK_SYSCON_EXT_WAKE_EN_Msk;
        return(CMSDK_SYSCON->PMUCTRL);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Get EXTERNAL WAKE STATUS bit.
 */

 uint32_t CMSDK_SYSCON_GetstatusEXT_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return ((CMSDK_SYSCON->PMUSTATUS & CMSDK_SYSCON_EXT_WAKE_STS_Msk) >> CMSDK_SYSCON_EXT_WAKE_STS_Pos);

 }
 
 /**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Clear EXTERNAL WAKE STATUS bit.
 */

 void CMSDK_SYSCON_ClearEXT_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCLEAR |= CMSDK_SYSCON_EXT_WAKE_STS_Msk;
 }

 /**
 *
 * @param *CMSDK_SYSCON System Control Pointer wake up from power switch 
 * @return none
 *
 * @brief Set PSW Wake interrupt enable bit, 
 * PSW Wake pin Wake enable , 
 * PSW Wake trigger mode.
 */

 uint32_t CMSDK_SYSCON_PMUCTRL_PSW_WAKE_Config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
        CMSDK_SYSCON->PMUCTRL |= CMSDK_SYSCON_PSW_WAKE_INTEN_Msk | CMSDK_SYSCON_PSW_WAKE_MOD_Msk | CMSDK_SYSCON_PSW_WAKE_EN_Msk;
        return(CMSDK_SYSCON->PMUCTRL);
 }

/**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Get PSW WAKE STATUS bit.
 */

 uint32_t CMSDK_SYSCON_GetstatusPSW_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return ((CMSDK_SYSCON->PMUSTATUS & CMSDK_SYSCON_PSW_WAKE_STS_Msk) >> CMSDK_SYSCON_PSW_WAKE_STS_Pos);

 }

 /**
 *
 * @param *CMSDK_SYSCON System Control Pointer
 * @return none
 *
 * @brief Clear PSW WAKE STATUS bit.
 */

 void CMSDK_SYSCON_ClearPSW_WAKEIRQ(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->PMUCLEAR |= CMSDK_SYSCON_PSW_WAKE_STS_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON system control Pointer
 * @return none
 *
 * @brief configure BAT_OFF bit field of ANACTRL
 */

 void CMSDK_SYSCON_ANACTRL_BATOFF_config(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->ANACTRL |= CMSDK_SYSCON_ANACTRL_BATOFF_Msk;
 }

/**
 *
 * @param *CMSDK_SYSCON system control Pointer
 * @return status of BAT_OFF bit field HIGH/LOW
 *
 * @brief Get status of BAT_OFF bit field of ANACTRL
 */

 uint32_t CMSDK_SYSCON_GetANACTRL_BATOFF_Status(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return((CMSDK_SYSCON->ANACTRL & CMSDK_SYSCON_ANACTRL_BATOFF_Msk) >> CMSDK_SYSCON_ANACTRL_BATOFF_Pos);
 }

 /**
 *
 * @param *CMSDK_SYSCON system control Pointer
 * @return none
 *
 * @brief Configuration to select the current measurement mode(0) 
 *
 */

 void CMSDK_SYSCON_ANACTRL_MEASMODE_IMEASselect(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->ANACTRL &= ~CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Msk;
 }

 /**
 *
 * @param *CMSDK_SYSCON system control Pointer
 * @return none
 *
 * @brief Configuration to select the impedance measurement mode(1)  
 *
 */

 void CMSDK_SYSCON_ANACTRL_MEASMODE_ZMEASselect(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       CMSDK_SYSCON->ANACTRL |= CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Msk;
 }

 /**
 *
 * @param *CMSDK_SYSCON system control Pointer
 * @return status of MEASMODE_SEL bit field HIGH/LOW
 *
 * @brief Get status of MEASMODE_SEL bit field of ANACTRL
 */

 uint32_t CMSDK_SYSCON_GetANACTRL_MEASMODE_Status(CMSDK_SYSCON_TypeDef *CMSDK_SYSCON)
 {
       return((CMSDK_SYSCON->ANACTRL & CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Msk) >> CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Pos);
 }

 /*IMEAS driver functions*/

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  configures IMEAS control register.
 */

 void CMSDK_IMEAS_config(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en)
 {   
       uint32_t new_ctrl=0;
       // set bias, cic_rate & pga_gain
                         new_ctrl |= ((webias_dac << CMSDK_IMEAS_WEBIAS_DAC_Pos) | (rebias_dac << CMSDK_IMEAS_REBIAS_DAC_Pos) | (cic_rate << CMSDK_IMEAS_CIC_RATE_Pos) | (pga_gain << CMSDK_IMEAS_PGA_GAIN_Pos));
       if (gubias_en!=0) new_ctrl |= CMSDK_IMEAS_GUBIAS_EN_Msk;// enable GU_BIAS block
       CMSDK_IMEAS->REG_CTRL = new_ctrl;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Enable IMEAS interrupt.
 */
 void CMSDK_IMEAS_EnableIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->REG_CTRL |= CMSDK_IMEAS_INT_EN_Msk;
 }
/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Disable IMEAS interrupt.
 */
 void CMSDK_IMEAS_DisableIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->REG_CTRL &= ~CMSDK_IMEAS_INT_EN_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Get imeas INTSTATUS
 */
 uint32_t CMSDK_IMEAS_GetInterrupt_Status(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       return (CMSDK_IMEAS->INTSTATUS);
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Clear IMEAS interrupt.
 */
 void CMSDK_IMEAS_ClearIRQ(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->INTCLEAR = CMSDK_IMEAS_INTCLEAR_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Set IMEAS channel mode.
 */

 void CMSDK_IMEAS_SetchannelMODE(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t cha_mode)
 {     
       CMSDK_IMEAS->CHA_MODE = ((CMSDK_IMEAS->CHA_MODE & ~CMSDK_IMEAS_CHA_MODE_Msk) | (cha_mode << CMSDK_IMEAS_CHA_MODE_Pos));
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Get IMEAS channel mode.
 */

 uint32_t CMSDK_IMEAS_GetchannelMODE(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {     
       return ((CMSDK_IMEAS->CHA_MODE & CMSDK_IMEAS_CHA_MODE_Msk) >> CMSDK_IMEAS_CHA_MODE_Pos);
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Set IMEAS channel format.
 */

 void CMSDK_IMEAS_SetchannelFORMAT(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t format_sel)
 {     
       CMSDK_IMEAS->CHA_MODE = ((CMSDK_IMEAS->CHA_MODE & ~CMSDK_IMEAS_FORMAT_SEL_Msk) | (format_sel << CMSDK_IMEAS_FORMAT_SEL_Pos));
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Set IMEAS channel number.
 */

 void CMSDK_IMEAS_SetchannelNUM(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t cha_num)
 {            
       CMSDK_IMEAS->CHA_MODE = ((CMSDK_IMEAS->CHA_MODE & ~CMSDK_IMEAS_CHA_NUM_Msk) | (cha_num << CMSDK_IMEAS_CHA_NUM_Pos));
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Enable imeas A/D Software reset control.
 */

 void CMSDK_IMEAS_EnableSD16RST(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL |= CMSDK_IMEAS_SD16RST_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Disable imeas A/D Software reset control.
 */

 void CMSDK_IMEAS_DisableSD16RST(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL &= ~CMSDK_IMEAS_SD16RST_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Enable imeas A/D Sleep mode control.
 */

 void CMSDK_IMEAS_EnableSD16SLP(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL |= CMSDK_IMEAS_SD16SLP_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Disable imeas A/D Sleep mode control.
 */

 void CMSDK_IMEAS_DisableSD16SLP(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL &= ~CMSDK_IMEAS_SD16SLP_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Enable imeas A/D power control.
 */

 void CMSDK_IMEAS_EnableSD16OFF(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL |= CMSDK_IMEAS_SD16OFF_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Disable imeas A/D power control.
 */

 void CMSDK_IMEAS_DisableSD16OFF(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       CMSDK_IMEAS->SEQ_CTRL &= ~CMSDK_IMEAS_SD16OFF_Msk;
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Get imeas CH0_DATA.
 */

 uint32_t CMSDK_IMEAS_GetCH0_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       return (CMSDK_IMEAS->CHA0_DATA);
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Get imeas CH1_DATA.
 */

 uint32_t CMSDK_IMEAS_GetCH1_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       return (CMSDK_IMEAS->CHA1_DATA);
 }

/**
 *
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @return none
 *
 * @brief  Get imeas CH2_DATA.
 */

 uint32_t CMSDK_IMEAS_GetCH2_data(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS)
 {
       return (CMSDK_IMEAS->CHA2_DATA);
 }

/**
 *
 * @brief  Initialises the imeas module and start filter, Single channel mode.
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @param set reg control to config various parameters
 * @param set channel mode to Single channel mode
 * @param set channel num
 * @param config to start filter
 * @return none
 *
 */
 uint32_t CMSDK_IMEAS_InitandStart_SingleChannelMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel, uint8_t cha_num){ 
        
        CMSDK_IMEAS_config(CMSDK_IMEAS, webias_dac, rebias_dac, cic_rate, pga_gain, gubias_en);// config parameters  //
        CMSDK_IMEAS_EnableIRQ(CMSDK_IMEAS);                                                    // Enable interrupt   //
        CMSDK_IMEAS_SetchannelMODE(CMSDK_IMEAS, 0x0); 		                               // set channel mode   //
        CMSDK_IMEAS_SetchannelFORMAT(CMSDK_IMEAS, format_sel);                                 // set channel format //
        CMSDK_IMEAS_SetchannelNUM(CMSDK_IMEAS, cha_num);                                       // set channel number //
        // start filter       //
        CMSDK_IMEAS_DisableSD16OFF(CMSDK_IMEAS);                                               
        CMSDK_IMEAS_DisableSD16SLP(CMSDK_IMEAS);
        CMSDK_IMEAS_DisableSD16RST(CMSDK_IMEAS); 

  return 0;
 }

/**
 *
 * @brief  Initialises the imeas module and start filter, Single channel continous mode.
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @param set reg control to config various parameters
 * @param set channel mode to Single channel continous mode
 * @param set channel num
 * @param config to start filter
 * @return none
 *
 */
 uint32_t CMSDK_IMEAS_InitandStart_SingleChannelContinousMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel, uint8_t cha_num){ 
        
        CMSDK_IMEAS_config(CMSDK_IMEAS, webias_dac, rebias_dac, cic_rate, pga_gain, gubias_en);// config parameters  //
        CMSDK_IMEAS_DisableIRQ(CMSDK_IMEAS);                                                   // Disable interrupt  //
        CMSDK_IMEAS_SetchannelMODE(CMSDK_IMEAS, 0x1); 		                               // set channel mode   //
        CMSDK_IMEAS_SetchannelFORMAT(CMSDK_IMEAS, format_sel);                                 // set channel format //
        CMSDK_IMEAS_SetchannelNUM(CMSDK_IMEAS, cha_num);                                       // set channel number //
        // start filter       //
        CMSDK_IMEAS_DisableSD16OFF(CMSDK_IMEAS);                                               
        CMSDK_IMEAS_DisableSD16SLP(CMSDK_IMEAS);
        CMSDK_IMEAS_DisableSD16RST(CMSDK_IMEAS); 

  return 0;
 }

/**
 *
 * @brief  Initialises the imeas module and start filter, Group channel mode.
 * @param *CMSDK_IMEAS IMEAS Pointer
 * @param set reg control to config various parameters
 * @param set channel mode to Group channel mode
 * @param config to start filter
 * @return none
 *
 */
 uint32_t CMSDK_IMEAS_InitandStart_GroupChannelMeas(CMSDK_IMEAS_TypeDef *CMSDK_IMEAS, uint8_t webias_dac, uint8_t rebias_dac, uint8_t cic_rate, uint8_t pga_gain, uint8_t gubias_en, uint8_t format_sel){ 
        
        CMSDK_IMEAS_config(CMSDK_IMEAS, webias_dac, rebias_dac, cic_rate, pga_gain, gubias_en);// config parameters  //
        CMSDK_IMEAS_EnableIRQ(CMSDK_IMEAS);                                                    // Enable interrupt   //
        CMSDK_IMEAS_SetchannelMODE(CMSDK_IMEAS, 0x2); 		                               // set channel mode   //
        CMSDK_IMEAS_SetchannelFORMAT(CMSDK_IMEAS, format_sel);                                 // set channel format //
        // start filter       //
        CMSDK_IMEAS_DisableSD16OFF(CMSDK_IMEAS);                                               
        CMSDK_IMEAS_DisableSD16SLP(CMSDK_IMEAS);
        CMSDK_IMEAS_DisableSD16RST(CMSDK_IMEAS); 

  return 0;
 }

/*ZMEAS driver functions*/
/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief configure ZMEAS control register bits
 */
 void CMSDK_ZMEAS_config(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_number_of_repeat_cycle_val, uint8_t reg_settling_time_val, 
			uint8_t reg_freq_val, uint8_t reg_mode, uint8_t repeat_calculation_en, uint8_t enable_intr, uint8_t enable_adc_intr,
			uint8_t measure_calibrate, uint8_t config_output_voltage_range, uint8_t pga_gain, uint8_t Reset_measurement)
 {
  	uint32_t new_reg_ctrl = 0;       
                                  new_reg_ctrl |= reg_number_of_repeat_cycle_val << CMSDK_ZMEAS_REG_CTRL_REPEAT_VAL_Pos;     	
	                          new_reg_ctrl |= reg_settling_time_val << CMSDK_ZMEAS_REG_CTRL_SETTLING_TIME_VAL_Pos;                                    	
	                          new_reg_ctrl |= reg_freq_val << CMSDK_ZMEAS_REG_CTRL_FREQ_VAL_Pos;                    	
                	          new_reg_ctrl |= reg_mode << CMSDK_ZMEAS_REG_CTRL_MODE_Pos;                        	
	                          new_reg_ctrl |= repeat_calculation_en << CMSDK_ZMEAS_REG_CTRL_REPEAT_CAL_EN_Pos;           	
	if(enable_intr !=0)       new_reg_ctrl |= CMSDK_ZMEAS_REG_CTRL_ENABLE_INTR_Msk;                     	
	if(enable_adc_intr !=0)   new_reg_ctrl |= CMSDK_ZMEAS_REG_CTRL_ENABLE_ADC_INTR_Msk;                 	
	if(measure_calibrate !=0) new_reg_ctrl |= CMSDK_ZMEAS_REG_CTRL_MEAS_CALIB_Msk;
	                          new_reg_ctrl |= config_output_voltage_range << CMSDK_ZMEAS_REG_CTRL_CONFIG_VOLT_Pos;
	if(pga_gain !=0)          new_reg_ctrl |= CMSDK_ZMEAS_REG_CTRL_PGA_GAIN_Msk;
	if(Reset_measurement !=0) new_reg_ctrl |= CMSDK_ZMEAS_REG_CTRL_RESET_MEASUREMENT_Msk;

	CMSDK_ZMEAS->REG_CTRL = new_reg_ctrl;                                                                               	
 }                                	

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief configure ZMEAS mode among "standby, power down, no operation, initialization, start calculation"
 */
 void CMSDK_ZMEAS_Setmode(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_mode)
 {	
        CMSDK_ZMEAS->REG_CTRL = ((CMSDK_ZMEAS->REG_CTRL & ~CMSDK_ZMEAS_REG_CTRL_MODE_Msk) | (reg_mode << CMSDK_ZMEAS_REG_CTRL_MODE_Pos));
  		     
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas REG_DATAOUT.
 */
 uint32_t CMSDK_ZMEAS_GetREG_DATAOUT(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->REG_DATAOUT);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas REG_STATUS.
 */
 uint32_t CMSDK_ZMEAS_GetREG_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->REG_STATUS);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas measurement done status.
 */
 uint32_t CMSDK_ZMEAS_GetMeasDone_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_FREQ_MEAS_DONE_Msk) >> CMSDK_ZMEAS_REG_STATUS_FREQ_MEAS_DONE_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas ADC enable status.
 */
 uint32_t CMSDK_ZMEAS_GetADC_enable_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_ADC_EN_Msk) >> CMSDK_ZMEAS_REG_STATUS_ADC_EN_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas DDS enable status.
 */
 uint32_t CMSDK_ZMEAS_GetDDs_enable_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_DDS_EN_Msk) >> CMSDK_ZMEAS_REG_STATUS_DDS_EN_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas PGA gain status.
 */
 uint32_t CMSDK_ZMEAS_GetPGA_gain_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_PGA_GAIN_Msk) >> CMSDK_ZMEAS_REG_STATUS_PGA_GAIN_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas measure/calibrate status.
 */
 uint32_t CMSDK_ZMEAS_Getmeasure_or_calibrate_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_MEAS_CALIB_Msk) >> CMSDK_ZMEAS_REG_STATUS_MEAS_CALIB_Pos);
 }       

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas config o/p voltage range status.
 */
 uint32_t CMSDK_ZMEAS_Getconfig_volt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_CONFIG_VOLT_Msk) >> CMSDK_ZMEAS_REG_STATUS_CONFIG_VOLT_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas reg frequency value status.
 */
 uint32_t CMSDK_ZMEAS_GetFreq_val_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_FREQ_VAL_Msk) >> CMSDK_ZMEAS_REG_STATUS_FREQ_VAL_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas Repeat cycle value status.
 */
 uint32_t CMSDK_ZMEAS_GetRepeat_val_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return((CMSDK_ZMEAS->REG_STATUS & CMSDK_ZMEAS_REG_STATUS_REPEAT_VAL_Msk) >> CMSDK_ZMEAS_REG_STATUS_REPEAT_VAL_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas ADC_DATA (ADC_ROM_REG)
 */
 uint32_t CMSDK_ZMEAS_GetADC_DATA_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return ((CMSDK_ZMEAS->ADC_ROM_REG & CMSDK_ZMEAS_ADC_DATA_Msk) >> CMSDK_ZMEAS_ADC_DATA_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas SINE (ADC_ROM_REG)
 */
 uint32_t CMSDK_ZMEAS_GetSINE_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return ((CMSDK_ZMEAS->ADC_ROM_REG & CMSDK_ZMEAS_SINE_Msk) >> CMSDK_ZMEAS_SINE_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas COSINE (ADC_ROM_REG)
 */
 uint32_t CMSDK_ZMEAS_GetCOSINE_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return ((CMSDK_ZMEAS->ADC_ROM_REG & CMSDK_ZMEAS_COSINE_Msk) >> CMSDK_ZMEAS_COSINE_Pos);
 }
 
/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas SUMMATION_OFFSET
 */
 uint32_t CMSDK_ZMEAS_GetSummation_Offset_Forreal(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->SUMMATION_OFFSET);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas SUMMATION_REAL
 */
 uint32_t CMSDK_ZMEAS_GetSummation_Real(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->SUMMATION_REAL);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas SUMMATION_IMAG
 */
 uint32_t CMSDK_ZMEAS_GetSummation_Imag(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->SUMMATION_IMAG);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas DFT_CNT value (DFT_CNT_REG)
 */
 uint32_t CMSDK_ZMEAS_GetDFT_CNT_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->DFT_CNT_REG & CMSDK_ZMEAS_DFT_CNT_Msk);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas shifted REAL value (DFT_CNT_REG)
 */
 uint32_t CMSDK_ZMEAS_GetSHIFTED_REAL_Value(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return ((CMSDK_ZMEAS->DFT_CNT_REG & CMSDK_ZMEAS_SHIFTED_REAL_Msk) >> CMSDK_ZMEAS_SHIFTED_REAL_Pos);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas INTSTATUS
 */
 uint32_t CMSDK_ZMEAS_GetInterrupt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->INTSTATUS);
 }

/*
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Clear ZMEAS interrupt.
 */
 void CMSDK_ZMEAS_ClearIRQ(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       CMSDK_ZMEAS->INTCLEAR = CMSDK_ZMEAS_INTCLEAR_Msk;
 } 

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Get zmeas ADCINTSTATUS
 */
 uint32_t CMSDK_ZMEAS_GetADCInterrupt_Status(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       return (CMSDK_ZMEAS->ADCINTSTATUS);
 }

/**
 *
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @return none
 *
 * @brief  Clear ZMEAS ADC interrupt
 */ 
 void CMSDK_ZMEAS_ClearADCIRQ(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS)
 {
       CMSDK_ZMEAS->ADCINTCLEAR = CMSDK_ZMEAS_INTCLEAR_Msk;
 }

/**
 *
 * @brief  Initialises the zmeas module and set control mode to start measurement/calibration.
 * @param *CMSDK_ZMEAS ZMEAS Pointer
 * @param set NO operation
 * @param config various parameters
 * @param set INIT
 * @param set START_CALC
 * @return none
 *
 */
 uint32_t CMSDK_ZMEAS_InitandStartMeas(CMSDK_ZMEAS_TypeDef *CMSDK_ZMEAS, uint8_t reg_number_of_repeat_cycle_val, uint8_t reg_settling_time_val, 
			               uint8_t reg_freq_val, uint8_t repeat_calculation_en, uint8_t enable_intr, uint8_t enable_adc_intr, 
                                       uint8_t measure_calibrate, uint8_t config_output_voltage_range, uint8_t pga_gain, uint8_t Reset_measurement){ 
        

	CMSDK_ZMEAS_config(CMSDK_ZMEAS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);                                                                 // NOOP mode 
    			
	CMSDK_ZMEAS_config(CMSDK_ZMEAS, reg_number_of_repeat_cycle_val, reg_settling_time_val, reg_freq_val, 1, repeat_calculation_en,    // INIT mode
					enable_intr, enable_adc_intr, measure_calibrate, config_output_voltage_range, pga_gain, Reset_measurement);

        CMSDK_ZMEAS_Setmode(CMSDK_ZMEAS, 3);                                                                                              // START_CALC mode

  return 0;
 }

/*FLASH driver functions*/
/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Enable NVR
 */
 void CMSDK_FLASH_EnableNVR(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL |= CMSDK_FLASH_NVR_ACCESS_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Disable NVR
 */
 void CMSDK_FLASH_DisableNVR(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL &= ~CMSDK_FLASH_NVR_ACCESS_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 * @brief Disable SER/CER
 * @brief Enable Program
 */
 void CMSDK_FLASH_EnablePG(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL = (CMSDK_FLASH->FLASH_CTRL & ~CMSDK_FLASH_CTRL_FUNCTION_Msk) | CMSDK_FLASH_CTRL_PG_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Disable Program
 */
 void CMSDK_FLASH_DisablePG(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL &= ~CMSDK_FLASH_CTRL_PG_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 * @brief Disable PG/CER
 * @brief Enable Sector Erase
 */
 void CMSDK_FLASH_EnableSER(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL = (CMSDK_FLASH->FLASH_CTRL & ~CMSDK_FLASH_CTRL_FUNCTION_Msk) | CMSDK_FLASH_CTRL_SER_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Disable Sector Erase
 */
 void CMSDK_FLASH_DisableSER(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL &= ~CMSDK_FLASH_CTRL_SER_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 * @brief Disable PG/SER
 * @brief Enable Chip Erase
 */
 void CMSDK_FLASH_EnableCER(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL = (CMSDK_FLASH->FLASH_CTRL & ~CMSDK_FLASH_CTRL_FUNCTION_Msk) | CMSDK_FLASH_CTRL_CER_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Disable Chip Erase
 */
 void CMSDK_FLASH_DisableCER(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL &= ~CMSDK_FLASH_CTRL_CER_EN_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set Full word
 */
 void CMSDK_FLASH_SetFullWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL |= (CMSDK_FLASH_CTRL_UPPER16_Msk | CMSDK_FLASH_CTRL_LOWER16_Msk);
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set Upper Half word
 */
 void CMSDK_FLASH_SetUPPHalfWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL = (CMSDK_FLASH->FLASH_CTRL & ~(CMSDK_FLASH_CTRL_UPPER16_Msk | CMSDK_FLASH_CTRL_LOWER16_Msk)) | CMSDK_FLASH_CTRL_UPPER16_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set Lower Half word
 */
 void CMSDK_FLASH_SetLOWHalfWord(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL = (CMSDK_FLASH->FLASH_CTRL & ~(CMSDK_FLASH_CTRL_UPPER16_Msk | CMSDK_FLASH_CTRL_LOWER16_Msk)) | CMSDK_FLASH_CTRL_LOWER16_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set ADDR
 */
 void CMSDK_FLASH_SetAddress(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t addr)
 {            
       CMSDK_FLASH->FLASH_WADDR = addr;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set WDATA
 */
 void CMSDK_FLASH_SetWData(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t wdata)
 {            
       CMSDK_FLASH->FLASH_WDATA = wdata;
 }


/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Get FLASH_CTRL CMD value
 */
 uint32_t CMSDK_FLASH_GetCMDvalue(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       return ((CMSDK_FLASH->FLASH_CTRL & CMSDK_FLASH_CTRL_CMD_Msk) >> CMSDK_FLASH_CTRL_CMD_Pos);
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set FLASH_CTRL to Goto IDLE
 * NOTE: Always invoke GotoIDLE before setting Goto INIT/Goto START commands
 */
 void CMSDK_FLASH_GotoIDLE(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL &= ~CMSDK_FLASH_CTRL_CMD_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set FLASH_CTRL to Goto INIT
 */
 void CMSDK_FLASH_GotoINIT(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL |= CMSDK_FLASH_CTRL_CMD_INIT_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Set FLASH_CTRL to Goto START
 */
 void CMSDK_FLASH_GotoSTART(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       CMSDK_FLASH->FLASH_CTRL |= CMSDK_FLASH_CTRL_CMD_START_Msk;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Get FLASH_CTRL PBusy status
 */
 uint32_t CMSDK_FLASH_GetPBusystatus(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       return ((CMSDK_FLASH->FLASH_STATUS & CMSDK_FLASH_STATUS_PBUSY_Msk) >> CMSDK_FLASH_STATUS_PBUSY_Pos);
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Get FLASH_CTRL PDone status
 */
 uint32_t CMSDK_FLASH_GetPDonestatus(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {            
       return ((CMSDK_FLASH->FLASH_STATUS & CMSDK_FLASH_STATUS_PDONE_Msk) >> CMSDK_FLASH_STATUS_PDONE_Pos);
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Get FLASH Sector number
 */
 uint32_t CMSDK_FLASH_GetSectornum(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {
       return ((CMSDK_FLASH->FLASH_WADDR & CMSDK_FLASH_CTRL_SECTOR_SEL_Msk) >> CMSDK_FLASH_CTRL_SECTOR_SEL_Pos);
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief Configure flash control cmd field(IDLE/INIT/START)
 *        Check Pbusy & Pdone status
 */
 void CMSDK_FLASH_status_check(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {
       if(CMSDK_FLASH_GetCMDvalue(CMSDK_FLASH) != 0x0) {//if not in IDLE state
          CMSDK_FLASH_GotoIDLE(CMSDK_FLASH); //goto IDLE
       }
       CMSDK_FLASH_GotoINIT(CMSDK_FLASH); //goto INIT
   //// check for busy status
       while(CMSDK_FLASH_GetPBusystatus(CMSDK_FLASH) == 0);
   //// pbusy is turned high
       CMSDK_FLASH_GotoSTART(CMSDK_FLASH); //goto START 
   //// check for done status
       while(CMSDK_FLASH_GetPDonestatus(CMSDK_FLASH) == 0);
   //// pdone is turned high
       CMSDK_FLASH_GotoIDLE(CMSDK_FLASH); //go back to IDLE, ie:clear init/start bits
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief set flash MAGIC WORD
 */
 void CMSDK_FLASH_SetMAGICWORD(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t value)
 {            
       CMSDK_FLASH->FLASH_MAGICW = value;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief unlock flash BADSECTOR registers
 */
 void CMSDK_FLASH_unlockBADSECTORregs(CMSDK_FLASH_TypeDef *CMSDK_FLASH, uint32_t value)
 {            
       CMSDK_FLASH->FLASH_BADSECTREGS_UNLOCKWORD = value;
 }

/**
 *
 * @param *CMSDK_FLASH Flash Pointer
 * @return none
 *
 * @brief initialize BADSECTOR registers
 */
 void badsector_initialize(CMSDK_FLASH_TypeDef *CMSDK_FLASH)
 {
       CMSDK_FLASH->FLASH_BADSECTOR0 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FF];
       CMSDK_FLASH->FLASH_BADSECTOR1 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FE]; 
       CMSDK_FLASH->FLASH_BADSECTOR2 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FD];
       CMSDK_FLASH->FLASH_BADSECTOR3 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FC];
       CMSDK_FLASH->FLASH_BADSECTOR4 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FB];
       CMSDK_FLASH->FLASH_BADSECTOR5 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05FA];
       CMSDK_FLASH->FLASH_BADSECTOR6 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05F9];
       CMSDK_FLASH->FLASH_BADSECTOR7 = ((uint32_t *)(CMSDK_PFLASH_BASE))[0x05F8];
 }

/*GPIO driver functions*/


/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param inenable Bit pattern to be used to set INPUT Enable register
 * @return none
 *
 * @brief  Sets pins on a port as an input. Set the bit corresponding to the pin number to 1 for output i.e. Set bit 1 of inenable to 1 to set pin 1 as an input. This function is thread safe.
 */

 uint32_t CMSDK_gpio_SetInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t inenableset)
 {
       CMSDK_GPIO->INENABLESET |= inenableset;
       return(CMSDK_GPIO->INENABLESET);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param inenable Bit pattern to be used to set INPUT Enable register
 * @return none
 *
 * @brief  Sets pins on a port as an input. Set the bit corresponding to the pin number to 1 for output i.e. Set bit 1 of inenable to 1 to set pin 1 as an input. This function is thread safe.
 */

 void CMSDK_gpio_ClrInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t inenableclr)
 {
       CMSDK_GPIO->INENABLESET = inenableclr;
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get INPUT Enable register
 *
 */

 uint32_t CMSDK_gpio_GetInEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INENABLESET);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set OPENDRAINEN register
 *
 */

 void CMSDK_gpio_SetOpen_DRAINEN(CMSDK_GPIO_TypeDef *CMSDK_GPIO,uint32_t opendrain)
 {
       CMSDK_GPIO->OPENDRAINEN |= opendrain;
 }
 
 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get OPENDRAINEN register
 *
 */

 uint32_t CMSDK_gpio_GetOpen_DRAINEN(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->OPENDRAINEN);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set DATATOPAD register value
 *
 */

 void CMSDK_gpio_SetData_ToPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t datatopad)
 {
       CMSDK_GPIO->DATATOPAD |= datatopad;
 }
 
 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get DATATOPAD register value
 *
 */

 uint32_t CMSDK_gpio_GetData_ToPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->DATATOPAD);
 }
 

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set PULLUPEN register value
 *
 */

 void CMSDK_gpio_SetPull_UpEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t pullupen)
 {
       CMSDK_GPIO->PULLUPEN |= pullupen;
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get PULLUPEN register value
 *
 */

 uint32_t CMSDK_gpio_GetPull_UpEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->PULLUPEN);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set PULLDOWNEN register value
 *
 */

 void CMSDK_gpio_SetPull_DownEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t pulldownen)
 {
       CMSDK_GPIO->PULLDOWNEN |= pulldownen;
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get PULLDOWNEN register value
 *
 */

 uint32_t CMSDK_gpio_GetPull_DownEn(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->PULLDOWNEN);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set DRVSTRENGTH register value
 *
 */

 void CMSDK_gpio_SetDrvStrength(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t drvstrength)
 {
       CMSDK_GPIO->DRVSTRENGTH |= drvstrength;
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get DRVSTRENGTH register value
 *
 */

 uint32_t CMSDK_gpio_GetDrvStrength(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->DRVSTRENGTH);
 }
 
 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get INTTYPESETD register value
 *
 */

 uint32_t CMSDK_gpio_GetIntTypeSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INTTYPESET);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get INTPOLSET register value
 *
 */

 uint32_t CMSDK_gpio_GetIntPolSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INTPOLSET);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get INTENSET register value
 *
 */

 uint32_t CMSDK_gpio_GetIntEnSet(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INTENSET);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get GetIntStatus register value
 *
 */

 uint32_t CMSDK_gpio_GetIntStatus(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INTSTATUS);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get GetIntClear register value
 *
 */

 uint32_t CMSDK_gpio_GetIntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->INTCLEAR);
 }

 
 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get GetData_FromPad register value
 *
 */

 uint32_t CMSDK_gpio_GetData_FromPad(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->DATAFROMPAD);
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set AE0 register
 *
 */

 void CMSDK_gpio_SetAE0(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t ae0)
 {
       CMSDK_GPIO->AE0 |= ae0;
 }


 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get AE0 register
 *
 */

 uint32_t CMSDK_gpio_GetAE0(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->AE0);
 }

 
 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Set AE0 register
 *
 */

 void CMSDK_gpio_SetAE1(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t ae1)
 {
       CMSDK_GPIO->AE1 |= ae1;
 }

 /**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Get AE1 register
 *
 */

 uint32_t CMSDK_gpio_GetAE1(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return(CMSDK_GPIO->AE1);
 }


/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param outenable Bit pattern to be used to set output enable register
 * @return none
 *
 * @brief  Sets pins on a port as an output. Set the bit corresponding to the pin number to 1 for output i.e. Set bit 1 of outenable to 1 to set pin 1 as an output. This function is thread safe.
 */

 void CMSDK_gpio_SetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableset)
 {
       CMSDK_GPIO->OUTENABLESET |= outenableset;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param outenable Bit pattern to be used to set output enable register
 * @return none
 *
 * @brief  Sets pins on a port as an input. Set the bit corresponding to the pin number to 1 for input i.e. Set bit 1 of outenable to 1 to set pin 1 as an input. This function is thread safe.
 */

 void CMSDK_gpio_ClrOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableclr)
 {
       CMSDK_GPIO->OUTENABLESET = outenableclr;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @return outputstatus
 *
 * @brief  returns a uint32_t which defines the whether pins on a port are set as inputs or outputs i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is an output.
 */

 uint32_t CMSDK_gpio_GetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return CMSDK_GPIO->OUTENABLESET;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param AltFunc uint32_t to specify whether the alternate function for the pins on the port is enabled
 * @return none
 *
 * @brief  enables the alternative function for pins. Set the bit corresponding to the pin number to 1 for alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to its alternative function. This function is thread safe.
 */

 uint32_t CMSDK_gpio_SetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncset)
 {  
       CMSDK_GPIO->ALTFUNCSET |= AltFuncset;
       return(CMSDK_GPIO->ALTFUNCSET);  
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param AltFunc uint32_t to specify whether the alternate function for the pins on the port is enabled
 * @return none
 *
 * @brief  disables the alternative function for pins. Set the bit corresponding to the pin number to 1 to disable alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to the orignal output function. This function is thread safe.
 */

 void CMSDK_gpio_ClrAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncclr)
 {
       CMSDK_GPIO->ALTFUNCSET = AltFuncclr;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @return AltFuncStatus
 *
 * @brief  returns a uint32_t which defines the whether pins on a port are set to their alternative or their original output functionality i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is set to its alternative function.
 */

 uint32_t CMSDK_gpio_GetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
 {
       return CMSDK_GPIO->ALTFUNCSET;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to clear the Interrupt
 * @return NewIntStatus
 *
 * @brief  Clears the interrupt flag for the specified pin and then returns the new interrupt status of the pin. This function is thread safe.
 */

 uint32_t CMSDK_gpio_IntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {  
       CMSDK_GPIO->INTCLEAR |=  Num;
       return CMSDK_GPIO->INTCLEAR;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to enable the Interrupt
 * @return NewIntEnStatus
 *
 * @brief  Enables interrupts for the specified pin and then returns the new interrupt enable status of the pin. This function is thread safe.
 */

 uint32_t CMSDK_gpio_SetIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {
       CMSDK_GPIO->INTENSET  |=Num;
       return CMSDK_GPIO->INTENSET;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to disable the Interrupt
 * @return NewIntEnStatus
 *
 * @brief  Disables interrupts for the specified pin and then returns the new interrupt enable status of the pin. This function is thread safe.
 */

  uint32_t CMSDK_gpio_ClrIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {
       CMSDK_GPIO->INTENSET  |=Num;
       return  CMSDK_GPIO->INTENSET;
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to set the Interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to a rising edge interrupt. This function is thread safe.
 */

 void CMSDK_gpio_SetIntRisingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {
       CMSDK_GPIO->INTTYPESET |= Num;  /* Set INT TYPE bit */
       
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to set the Interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to a low level interrupt. This function is thread safe.
 */

 void CMSDK_gpio_SetIntLowLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {
       CMSDK_GPIO->INTPOLSET  |= Num;            /* Set INT POLarity bit */
 }

/**
 *
 * @param *CMSDK_GPIO GPIO Pointer
 * @param Num The pin number for which to set the Interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to a falling edge interrupt. This function is thread safe.
 */

 void CMSDK_gpio_SetIntFallingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
 {
       CMSDK_GPIO->INTTYPESET |= Num;  /* Set INT TYPE bit */
       CMSDK_GPIO->INTPOLSET  |= Num;   /* Set INT POLarity bit */
 }

/*@}*/ /* end of group CMSIS_CM0_CMSDK_Driver_definitions CMSDK Driver definitions */
