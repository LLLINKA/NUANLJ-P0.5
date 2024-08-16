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
 *    - SPI
 *    - Timer
 *    - Dual Timer
 *    - RTC
 *    - GPIO
 *
 * The library contains C and assembly functions that have been ported and tested on the MDK
 * toolchain.
 */


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

 void CMSDK_uart_init(CMSDK_UART_TypeDef *CMSDK_UART, uint16_t divider, uint8_t break_en, uint8_t stick_en, uint8_t even_en, uint8_t parity_en, uint8_t stop_len, uint8_t word_len, uint8_t fifo_en)
 {
       uint32_t new_ctrl=0;

       CMSDK_UART->DLL = divider & 0x00FF;
       CMSDK_UART->DLH = divider & 0xFF00;
				new_ctrl = word_len << CMSDK_UART_LCR_WORD_LEN_Pos; 
       if (break_en != 0)       new_ctrl |= CMSDK_UART_LCR_BREAK_EN_Msk;
       if (stick_en != 0)       new_ctrl |= CMSDK_UART_LCR_STICK_EN_Msk;
       if (even_en  != 0)       new_ctrl |= CMSDK_UART_LCR_EVEN_EN_Msk;
       if (parity_en!= 0)       new_ctrl |= CMSDK_UART_LCR_PARITY_EN_Msk;
       if (stop_len != 0)       new_ctrl |= CMSDK_UART_LCR_STOP_LEN_Msk;
       
       CMSDK_UART->LCR = new_ctrl;
       
       if (fifo_en != 0)        CMSDK_UART->FCR = CMSDK_UART_FCR_FIFO_EN_Msk;
       
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
       while(!(CMSDK_UART->LSR & CMSDK_UART_LSR_THR_EMPTY_Msk));
       CMSDK_UART->THR = txchar;
 }


 /*SPI driver functions*/

/**
 *
 * @param *CMSDK_SPI SPI Pointer
 * @return none
 *
 * @brief Configure SPI CTRL1 register
 */
 void CMSDK_SPI_CTRL1_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t duplex_mode, uint8_t tx_rx_dir, uint8_t baud, uint8_t lsb_sel, uint8_t loopback,
			uint8_t nss_toggle, uint8_t nss_ctrl, uint8_t nss_sw_input, uint8_t cpol, uint8_t cpha, uint8_t mst_slv_sel, uint8_t spi_en)
 {
  	uint32_t new_reg_ctrl = 0;       
				new_reg_ctrl = baud << CMSDK_SPI_BAUD_RATE_Pos;     	
	if(duplex_mode == 1) {
		if(tx_rx_dir) {
				new_reg_ctrl |= 0x2 << CMSDK_SPI_UNIDI_MODE_Pos;
		}
		else {
				new_reg_ctrl |= 0x1 << CMSDK_SPI_UNIDI_MODE_Pos;
		}
	}
	if(duplex_mode == 2) {
		new_reg_ctrl |= CMSDK_SPI_BIDI_EN_Msk;
		if(tx_rx_dir) {
				new_reg_ctrl |= CMSDK_SPI_BIDI_MODE_Msk;
		}
	}
	if(lsb_sel != 0)          new_reg_ctrl |= CMSDK_SPI_LSB_SEL_Msk;
	if(loopback != 0)         new_reg_ctrl |= CMSDK_SPI_LOOP_BACK_EN_Msk;
 	if(nss_toggle != 0)       new_reg_ctrl |= CMSDK_SPI_NSS_TOGGLE_Msk;
	if(nss_ctrl != 0)         new_reg_ctrl |= CMSDK_SPI_NSS_MST_CTRL_Msk;
	if(nss_sw_input != 0)     new_reg_ctrl |= CMSDK_SPI_NSS_MST_SW_Msk;
	if(cpol != 0)             new_reg_ctrl |= CMSDK_SPI_CPOL_Msk;
	if(cpha != 0)             new_reg_ctrl |= CMSDK_SPI_CPHA_Msk;	                                     	
	if(mst_slv_sel !=0)       new_reg_ctrl |= CMSDK_SPI_MST_SLV_SEL_Msk;                     	
	if(spi_en !=0)            new_reg_ctrl |= CMSDK_SPI_EN_Msk;	

	CMSDK_SPI->CTRL1 = new_reg_ctrl;                                                                               	
 }
 
/**
 *
 * @param *CMSDK_SPI SPI Pointer
 * @return none
 *
 * @brief Configure SPI CTRL2 register
 */
 void CMSDK_SPI_CTRL2_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t t2c_delay, uint8_t c2t_delay, uint8_t nss_en, uint8_t samp_phase, uint8_t tx_dma_en, uint8_t rx_dma_en, uint8_t char_len)
 {
  	uint32_t new_reg_ctrl = 0;
				new_reg_ctrl = char_len << CMSDK_SPI_CHAR_LEN_Pos;
				new_reg_ctrl |= samp_phase << CMSDK_SPI_SAMP_PHASE_Pos;       
				new_reg_ctrl |= t2c_delay << CMSDK_SPI_T2C_DELAY_Pos;
				new_reg_ctrl |= c2t_delay << CMSDK_SPI_C2T_DELAY_Pos; 	
	if(nss_en == 1) {
				new_reg_ctrl |= CMSDK_SPI_NSS0_EN_Msk;
	}
	if(nss_en == 2) {
				new_reg_ctrl |= CMSDK_SPI_NSS1_EN_Msk;
	}
	if(nss_en == 3) {
				new_reg_ctrl |= CMSDK_SPI_NSS2_EN_Msk;
	}
	if(nss_en == 4) {
				new_reg_ctrl |= CMSDK_SPI_NSS3_EN_Msk;
	}
					
	if(tx_dma_en != 0)      new_reg_ctrl |= CMSDK_SPI_TX_DMA_EN_Msk;
	if(rx_dma_en != 0)      new_reg_ctrl |= CMSDK_SPI_RX_DMA_EN_Msk;	

	CMSDK_SPI->CTRL2 = new_reg_ctrl;                                                                               	
 }

/**
 *
 * @param *CMSDK_SPI SPI Pointer
 * @return none
 *
 * @brief Configure SPI FCR register
 */
 void CMSDK_SPI_FCR_config(CMSDK_SPI_TypeDef *CMSDK_SPI, uint8_t tx_fifo_th, uint8_t rx_fifo_th, uint8_t tx_fifo_clr, uint8_t rx_fifo_clr, uint8_t fifo_en)
 {
  	uint32_t new_reg_ctrl = 0;
				new_reg_ctrl |= tx_fifo_th << CMSDK_SPI_TX_FIFO_TH_Pos;
				new_reg_ctrl |= rx_fifo_th << CMSDK_SPI_RX_FIFO_TH_Pos;	
					
	if(tx_fifo_clr != 0)      new_reg_ctrl |= CMSDK_SPI_TX_FIFO_CLR_Msk;
	if(rx_fifo_clr != 0)      new_reg_ctrl |= CMSDK_SPI_RX_FIFO_CLR_Msk;
	if(fifo_en != 0)          new_reg_ctrl |= CMSDK_SPI_FIFO_EN_Msk;

	CMSDK_SPI->FCR = new_reg_ctrl;                                                                               	
 }


 /* TImer driver functions*/

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
 * @param irq_en Defines whether the timer IRQ is to be enabled
 * @return none
 * @brief  Initialises the timer, specifies the timer current & reload value , and whether IRQ is enabled or not.
 */

  void CMSDK_timer_Init(CMSDK_TIMER_TypeDef *CMSDK_TIMER, uint32_t reload, uint32_t irq_en)
 {     uint32_t new_ctrl = 0;
       
       CMSDK_TIMER->VALUE = reload;
       CMSDK_TIMER->RELOAD = reload;
       if (irq_en!=0)   new_ctrl |= CMSDK_TIMER_CTRL_IRQEN_Msk;               /* non zero - enable IRQ */
       new_ctrl |= CMSDK_TIMER_CTRL_EN_Msk;                                   /* enable timer */
       CMSDK_TIMER->CTRL = new_ctrl;
 }

/**
 *
 * @param *CMSDK_TIMER Timer Pointer
 * @param reload The value to which the timer is to be set after an underflow has occurred
 * @param irq_en Defines whether the timer IRQ is to be enabled
 * @return none
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


 /*DUAL Timer driver functions*/

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Start timer in dual timers.
 */

 /* Start Timer */
 void CMSDK_dualtimer_start(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx)
 {
  CMSDK_DUALTIMERx->TimerControl |= CMSDK_DUALTIMER_CTRL_EN_Msk;
 }

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Stop timer in dual timers.
 */

 /* Stop Timer */
 void CMSDK_dualtimer_stop(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx)
 {
  CMSDK_DUALTIMERx->TimerControl &= ~CMSDK_DUALTIMER_CTRL_EN_Msk;
 }

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Clear the interrupt request in dual timers.
 */

 /* Clear the Interrupt */
 void CMSDK_dualtimer_irq_clear(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx)
 {
  CMSDK_DUALTIMERx->TimerIntClr = 0;
 }

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Setup Free running mode in dual timers.
 */

 /* Free running timer mode */
 void CMSDK_dualtimer_setup_freerunning(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx,
 unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size)
 {
  int ctrl_val;
  CMSDK_DUALTIMERx->TimerControl = 0; /* Disable during programming */
  /* Previous timer activities might have trigger interrupt flag,
   so need to clear it */
  CMSDK_dualtimer_irq_clear(CMSDK_DUALTIMERx);
  CMSDK_DUALTIMERx->TimerLoad    = cycle;

  ctrl_val = (prescale  & 0x3) << CMSDK_DUALTIMER_CTRL_PRESCALE_Pos |
             (interrupt & 0x1) << CMSDK_DUALTIMER_CTRL_INTEN_Pos |
             (size      & 0x1) << CMSDK_DUALTIMER_CTRL_SIZE_Pos |
            CMSDK_DUALTIMER_CTRL_EN_Msk;

  CMSDK_DUALTIMERx->TimerControl = ctrl_val;
 }

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Setup Periodic mode in dual timers.
 */

 /* Periodic timer mode */
 void CMSDK_dualtimer_setup_periodic(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx,
 unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size)
 {
  int ctrl_val;
  CMSDK_DUALTIMERx->TimerControl = 0; /* Disable during programming */
  /* Previous timer activities might have trigger interrupt flag,
   so need to clear it */
  CMSDK_dualtimer_irq_clear(CMSDK_DUALTIMERx);
  CMSDK_DUALTIMERx->TimerLoad    = cycle;

  ctrl_val = (prescale  & 0x3) << CMSDK_DUALTIMER_CTRL_PRESCALE_Pos |
             (interrupt & 0x1) << CMSDK_DUALTIMER_CTRL_INTEN_Pos |
             (size      & 0x1) << CMSDK_DUALTIMER_CTRL_SIZE_Pos |
            CMSDK_DUALTIMER_CTRL_EN_Msk |
            CMSDK_DUALTIMER_CTRL_MODE_Msk;

  CMSDK_DUALTIMERx->TimerControl = ctrl_val;
 }

/**
 *
 * @param *CMSDK_DUALTIMER DUAL Timer Pointer
 * @return none
 *
 * @brief  Setup One shot mode in dual timers.
 */

 /* One shot timer mode */
 void CMSDK_dualtimer_setup_oneshot(CMSDK_DUALTIMER_TypeDef *CMSDK_DUALTIMERx,
 unsigned int cycle, unsigned int prescale, unsigned int interrupt, unsigned int size)
 {
  int ctrl_val;
  CMSDK_DUALTIMERx->TimerControl = 0; /* Disable during programming */
  /* Previous timer activities might have trigger interrupt flag,
   so need to clear it */
  CMSDK_dualtimer_irq_clear(CMSDK_DUALTIMERx);
  CMSDK_DUALTIMERx->TimerLoad    = cycle;

  ctrl_val = (prescale  & 0x3) << CMSDK_DUALTIMER_CTRL_PRESCALE_Pos |
             (interrupt & 0x1) << CMSDK_DUALTIMER_CTRL_INTEN_Pos |
             (size      & 0x1) << CMSDK_DUALTIMER_CTRL_SIZE_Pos |
            CMSDK_DUALTIMER_CTRL_EN_Msk |
            CMSDK_DUALTIMER_CTRL_ONESHOT_Msk;

  CMSDK_DUALTIMERx->TimerControl = ctrl_val;
 }


 /*RTC driver functions*/

/**
 *
 * @param *CMSDK_RTC RTC Pointer
 * @return none
 *
 * @brief Initialize RTC Calender
 */
 void CMSDK_RTC_Init_Calender(uint16_t prescaler, uint8_t data_mode, uint8_t hour_mode, uint32_t init_time, uint32_t init_date)
 {
	uint32_t new_reg_ctrl = 0;
	//wait for rtc prescaler sync ready
	while(!((CMSDK_RTC->SR & CMSDK_RTC_PRES_SYNC_READY_Msk) >> CMSDK_RTC_PRES_SYNC_READY_Pos));
	//set prescaler
	CMSDK_RTC->PR = prescaler;	
	//config data/hour mode
	if(data_mode != 0)      new_reg_ctrl |= CMSDK_RTC_DATA_MODE_Msk;
	if(hour_mode != 0)      new_reg_ctrl |= CMSDK_RTC_HOUR_MODE_Msk;
	CMSDK_RTC->CR = new_reg_ctrl;
	//wait for init sync ready
	while(!(CMSDK_RTC->SR & CMSDK_RTC_INIT_SYNC_READY_Msk));
	//set INIT
	CMSDK_RTC->CR |= CMSDK_RTC_INIT_EN_Msk;	
	//Set Initial Time & Date
	CMSDK_RTC->TR = init_time;
	CMSDK_RTC->DR = init_date;
	//Clear INIT bit
	CMSDK_RTC->CR &= ~CMSDK_RTC_INIT_EN_Msk;
	//wait for init sync ready
	while(!(CMSDK_RTC->SR & CMSDK_RTC_INIT_SYNC_READY_Msk));

	return;                                                                               	
 }
/**
 *
 * @param *CMSDK_RTC RTC Pointer
 * @return none
 *
 * @brief Config RTC Alarm
 */
 void CMSDK_RTC_Config_Alarm(uint16_t prescaler, uint8_t data_mode, uint8_t hour_mode, uint32_t init_time, uint32_t init_date, uint32_t alarm_time, uint32_t alarm_date)
 {
	uint32_t new_reg_ctrl = 0;
	//wait for rtc prescaler sync ready
	while(!((CMSDK_RTC->SR & CMSDK_RTC_PRES_SYNC_READY_Msk) >> CMSDK_RTC_PRES_SYNC_READY_Pos));
	//set prescaler
	CMSDK_RTC->PR = prescaler;		
	//config data/hour mode
	if(data_mode != 0)      new_reg_ctrl |=  CMSDK_RTC_DATA_MODE_Msk;
	if(hour_mode != 0)      new_reg_ctrl |= CMSDK_RTC_HOUR_MODE_Msk;
	CMSDK_RTC->CR = new_reg_ctrl;
	//wait for init sync ready
	while(!(CMSDK_RTC->SR & CMSDK_RTC_INIT_SYNC_READY_Msk));
	//set INIT
	CMSDK_RTC->CR |= CMSDK_RTC_INIT_EN_Msk;	
	//Set Initial Time & Date
	CMSDK_RTC->TR = init_time;
	CMSDK_RTC->DR = init_date;
	//Set Alarm Time & Date
	CMSDK_RTC->TAR = alarm_time;
	CMSDK_RTC->DAR = alarm_date;
	//Alarm enabled
	CMSDK_RTC->CR |= CMSDK_RTC_ALARM_EN_Msk;
	//Clear INIT bit
	CMSDK_RTC->CR &= ~CMSDK_RTC_INIT_EN_Msk;

	return;                                                                               	
 }
/**
 *
 * @param *CMSDK_RTC RTC Pointer
 * @return none
 *
 * @brief Config Period Wakeup
 */
 void CMSDK_RTC_Config_PeriodWake(uint8_t clock_sel, uint16_t prescaler, uint16_t period_time)
 {
    if(clock_sel) {
	CMSDK_RTC->CR |= CMSDK_RTC_WUT_CLK_EN_Msk;
	//wait for rtc prescaler sync ready
	while(!((CMSDK_RTC->SR & CMSDK_RTC_PRES_SYNC_READY_Msk) >> CMSDK_RTC_PRES_SYNC_READY_Pos));
	//set prescaler
	CMSDK_RTC->PR = prescaler;
    }
    else {
	CMSDK_RTC->CR &= ~CMSDK_RTC_WUT_CLK_EN_Msk;
	//wait for wut prescaler sync ready
	while(!((CMSDK_RTC->SR & CMSDK_RTC_WUT_PRES_SYNC_READY_Msk) >> CMSDK_RTC_WUT_PRES_SYNC_READY_Pos));
	//set prescaler
	CMSDK_RTC->WPR = prescaler;
    }	
    //wait for wut value sync ready
    while(!((CMSDK_RTC->SR & CMSDK_RTC_WUT_VAL_SYNC_READY_Msk) >> CMSDK_RTC_WUT_VAL_SYNC_READY_Pos));	
    //set Wakeup time register
    CMSDK_RTC->WTR = period_time;
    //Config CR with Periodic Wakeup timer enabled
    CMSDK_RTC->CR |= CMSDK_RTC_WUT_EN_Msk;	 

    return;                                                                               	
 }

// /*ADC driver functions*/
//
///**
// *
// * @param *CMSDK_ADC_CTRL ADC_CTRL Pointer
// * @param 
// * @return none
// *
// * @brief 
// */
//
/* --------------------------------------------------------------- */
/*  Set ADC_EN bit and ADC_START bit                          	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_SetADC_EN_AND_STARTBIT(uint8_t value1, uint8_t value2){
     CMSDK_ADC->ADC_CTRL = (((CMSDK_ADC->ADC_CTRL) & ~CMSDK_ADC_EN_Msk & ~CMSDK_ADC_START_Msk) | (value1 <<  CMSDK_ADC_EN_Pos) | (value2 << CMSDK_ADC_START_Pos));
     
}

/* --------------------------------------------------------------- */
/*  EOC AND OVERRUN INTERRUPT ENABLE/DISABLE                           	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_INT_EN(uint8_t value1, uint8_t value2){
     CMSDK_ADC->ADC_IER = (((CMSDK_ADC->ADC_IER) & ~CMSDK_ADC_EOC_INT_EN_Msk & ~CMSDK_ADC_OVERRUN_INT_EN_Msk) | (value1 << CMSDK_ADC_EOC_INT_EN_Pos) | (value2 << CMSDK_ADC_OVERRUN_INT_EN_Pos));
     
}

/* --------------------------------------------------------------- */
/*  Set Contineous Conversion Mode Enable bit                  	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_SetCONT_CONV_MODE_EN(uint8_t value){
     CMSDK_ADC->ADC_CONFG = (((CMSDK_ADC->ADC_CONFG) & ~CMSDK_ADC_COV_MODE_Msk) | (value <<  CMSDK_ADC_COV_MODE_Pos) );
     
}

/* --------------------------------------------------------------- */
/*  Set SINGLE/Contineous Conversion Mode 	                	   */
/* --------------------------------------------------------------- */
void CMSDK_SETADC_CONV_MODE_EN(uint8_t value){
     CMSDK_ADC->ADC_CONFG = (((CMSDK_ADC->ADC_CONFG) & ~CMSDK_ADC_COV_MODE_Msk) | (value <<  CMSDK_ADC_COV_MODE_Pos) );
     
}

/* --------------------------------------------------------------- */
/*  Disable ADC_EN bit                          	   	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_DISABLE_ADC_EN_BIT(){
     CMSDK_ADC->ADC_CTRL = (((CMSDK_ADC->ADC_CTRL) & ~CMSDK_ADC_EN_Msk ) | (0x00 <<  CMSDK_ADC_EN_Pos) );
     
}
/* --------------------------------------------------------------- */
/*  SEt ADC_CLOCK DIVIDER VALUE                        	   	   */
/* --------------------------------------------------------------- */
void CMSDK_SET_ADCCLK_DIV_VAL(uint8_t value){
     CMSDK_ADC->ADC_CLK_DIV = (((CMSDK_ADC->ADC_CLK_DIV) & ~CMSDK_ADC_CLK_DIV_Msk) | (value <<  CMSDK_ADC_CLK_DIV_Pos) );
     
}
/* --------------------------------------------------------------- */
/*  SEt ADC_SAMPLING TIME VALUE                        	   	   */
/* --------------------------------------------------------------- */
void CMSDK_SET_ADCSAMP_TIME(uint8_t value){
     CMSDK_ADC->ADC_SAMP_TIME = (((CMSDK_ADC->ADC_SAMP_TIME) &  ~CMSDK_ADC_SAMPLING_TIME_Msk) | (value <<  CMSDK_ADC_SAMPLING_TIME_Pos) );

}
/* --------------------------------------------------------------- */
/*  SEt ADC_CH_SEL VALUE                        	   	   */
/* --------------------------------------------------------------- */
void CMSDK_SET_ADC_CHANNEL_SEL(uint8_t value){
     CMSDK_ADC->ADC_CH_SEL = (((CMSDK_ADC->ADC_CH_SEL) &  ~CMSDK_ADC_CHANNEL_SEL_Msk) | (value <<  CMSDK_ADC_CHANNEL_SEL_Pos) );

}
/* --------------------------------------------------------------- */
/*  start next conversion with/without receiving EOC           	   */
/* --------------------------------------------------------------- */
void CMSDK_SETADC_CONVINC_EOC(uint8_t value){
     CMSDK_ADC->ADC_EOC_CONFG = (((CMSDK_ADC->ADC_EOC_CONFG) & ~CMSDK_ADC_CONVERSION_INC_EOC_Msk) | (value <<  CMSDK_ADC_CONVERSION_INC_EOC_Pos) );

}
/* --------------------------------------------------------------- */
/*  Set Wait Mode 			                  	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_Set_WAIT_MODE_EN(uint8_t value){
     CMSDK_ADC->ADC_CONFG = (((CMSDK_ADC->ADC_CONFG) & ~CMSDK_WAIT_MODE_EN_Msk) | (value <<  CMSDK_WAIT_MODE_EN_Pos) );

}
/* --------------------------------------------------------------- */
/*  Set OVERRUN Mode 			                  	   */
/* --------------------------------------------------------------- */
void CMSDK_ADC_Set_OVERRUN_MODE_EN(uint8_t value){
     CMSDK_ADC->ADC_CONFG = (((CMSDK_ADC->ADC_CONFG) & ~CMSDK_ADC_OVERRUN_MODE_Msk) | (value <<  CMSDK_ADC_OVERRUN_MODE_Pos) );
     
}



// /*SYSTEM CONTROL driver functions*/
//
///**
// *
// * @param *CMSDK_CMSDK_SYSCON SYS_CTRL Pointer
// * @param 
// * @return none
// *
// * @brief  
// */
//

void CMSDK_SYSCON_SetHCLK_DIV(uint8_t value)
 {
       CMSDK_SYSCON->CLK_CFG = (((CMSDK_SYSCON->CLK_CFG) & ~CMSDK_SYSCON_AHB_PRES_Msk) | (value <<  CMSDK_SYSCON_AHB_PRES_Pos)); //(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }


void CMSDK_SYSCON_SetHSI_CTRL(uint8_t value)
 {
       CMSDK_SYSCON->HSI_CTRL = (((CMSDK_SYSCON->HSI_CTRL) & ~CMSDK_SYSCON_HSI_FREQ_Msk) | (value << CMSDK_SYSCON_HSI_FREQ_Pos));
       //CMSDK_SYSCON->HSI_CTRL = (value << CMSDK_SYSCON_HSI_FREQ_Pos);
 }


void CMSDK_SYSCON_SetPCLK_DIV(uint8_t value)
 {
       CMSDK_SYSCON->CLK_CFG = (((CMSDK_SYSCON->CLK_CFG) & ~CMSDK_SYSCON_APB_PRES_Msk) | (value <<  CMSDK_SYSCON_APB_PRES_Pos)); //(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }


void CMSDK_SYSCON_SetSYSCLK_SEL(uint8_t value)
{
       CMSDK_SYSCON->CLK_CFG = (((CMSDK_SYSCON->CLK_CFG) & ~CMSDK_SYSCON_SYSCLK_SEL_Msk) | (value <<  CMSDK_SYSCON_SYSCLK_SEL_Pos));//(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
}


void CMSDK_SYSCON_SetHSI_EN(uint8_t value)
 {
       CMSDK_SYSCON->HSI_CTRL = (((CMSDK_SYSCON->HSI_CTRL) & ~CMSDK_SYSCON_HSI_EN_Msk) | (value <<  CMSDK_SYSCON_HSI_EN_Pos));//(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }

void CMSDK_SYSCON_SetMCO_SEL(uint8_t value)
 {
       CMSDK_SYSCON->CLK_CFG = (((CMSDK_SYSCON->CLK_CFG) & ~CMSDK_SYSCON_MCO_SEL_Msk) | (value <<  CMSDK_SYSCON_MCO_SEL_Pos));//(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }
 

void CMSDK_SYSCON_SetLFCLK_SEL(uint8_t value)
 {
       CMSDK_SYSCON->CLK_CFG = (((CMSDK_SYSCON->CLK_CFG) & ~CMSDK_SYSCON_LFCLK_SEL_Msk) | (value <<  CMSDK_SYSCON_LFCLK_SEL_Pos));//(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }


void CMSDK_SYSCON_SetLSE_EN(uint8_t value)
 {
       CMSDK_SYSCON->LSE_CTRL = (((CMSDK_SYSCON->LSE_CTRL) & ~CMSDK_SYSCON_LSE_EN_Msk) | (value <<  CMSDK_SYSCON_LSE_EN_Pos));//(((CMSDK_SYSCON->CLK_CFG) & ~(value <<  CMSDK_SYSCON_CLK_CFG_Pos)) | (value <<  CMSDK_SYSCON_CLK_CFG_Pos));
 }


void CMSDK_SYSCON_DisableLSI_EN()
{
    CMSDK_SYSCON->LSI_CTRL = CMSDK_SYSCON_LSI_EN_Pos;
}  


// /*GPIO driver functions*/
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param outenable Bit pattern to be used to set output enable register
// * @return none
// *
// * @brief  Sets pins on a port as an output. Set the bit corresponding to the pin number to 1 for output i.e. Set bit 1 of outenable to 1 to set pin 1 as an output. This function is thread safe.
// */
//
// void CMSDK_gpio_SetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableset)
// {
//       CMSDK_GPIO->OUTENABLESET = outenableset;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param outenable Bit pattern to be used to set output enable register
// * @return none
// *
// * @brief  Sets pins on a port as an input. Set the bit corresponding to the pin number to 1 for input i.e. Set bit 1 of outenable to 1 to set pin 1 as an input. This function is thread safe.
// */
//
// void CMSDK_gpio_ClrOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t outenableclr)
// {
//       CMSDK_GPIO->OUTENABLECLR = outenableclr;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @return outputstatus
// *
// * @brief  returns a uint32_t which defines the whether pins on a port are set as inputs or outputs i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is an output.
// */
//
// uint32_t CMSDK_gpio_GetOutEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
// {
//       return CMSDK_GPIO->OUTENABLESET;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param AltFunc uint32_t to specify whether the alternate function for the pins on the port is enabled
// * @return none
// *
// * @brief  enables the alternative function for pins. Set the bit corresponding to the pin number to 1 for alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to its alternative function. This function is thread safe.
// */
//
// void CMSDK_gpio_SetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncset)
// {
//       CMSDK_GPIO->ALTFUNCSET = AltFuncset;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param AltFunc uint32_t to specify whether the alternate function for the pins on the port is enabled
// * @return none
// *
// * @brief  disables the alternative function for pins. Set the bit corresponding to the pin number to 1 to disable alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to the orignal output function. This function is thread safe.
// */
//
// void CMSDK_gpio_ClrAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t AltFuncclr)
// {
//       CMSDK_GPIO->ALTFUNCCLR = AltFuncclr;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @return AltFuncStatus
// *
// * @brief  returns a uint32_t which defines the whether pins on a port are set to their alternative or their original output functionality i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is set to its alternative function.
// */
//
// uint32_t CMSDK_gpio_GetAltFunc(CMSDK_GPIO_TypeDef *CMSDK_GPIO)
// {
//       return CMSDK_GPIO->ALTFUNCSET;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to clear the Interrupt
// * @return NewIntStatus
// *
// * @brief  Clears the interrupt flag for the specified pin and then returns the new interrupt status of the pin. This function is thread safe.
// */
//
// uint32_t CMSDK_gpio_IntClear(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTCLEAR = (1 << Num);
//
//       return CMSDK_GPIO->INTSTATUS;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to enable the Interrupt
// * @return NewIntEnStatus
// *
// * @brief  Enables interrupts for the specified pin and then returns the new interrupt enable status of the pin. This function is thread safe.
// */
//
// uint32_t CMSDK_gpio_SetIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTENSET = (1 << Num);
//
//       return CMSDK_GPIO->INTENSET;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to disable the Interrupt
// * @return NewIntEnStatus
// *
// * @brief  Disables interrupts for the specified pin and then returns the new interrupt enable status of the pin. This function is thread safe.
// */
//
//  uint32_t CMSDK_gpio_ClrIntEnable(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTENCLR = (1 << Num);
//
//       return CMSDK_GPIO->INTENCLR;
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to set the Interrupt type
// * @return none
// *
// * @brief  Changes the interrupt type for the specified pin to a high level interrupt. This function is thread safe.
// */
//
// void CMSDK_gpio_SetIntHighLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTTYPECLR = (1 << Num); /* Clear INT TYPE bit */
//       CMSDK_GPIO->INTPOLSET = (1 << Num);  /* Set INT POLarity bit */
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to set the Interrupt type
// * @return none
// *
// * @brief  Changes the interrupt type for the specified pin to a rising edge interrupt. This function is thread safe.
// */
//
// void CMSDK_gpio_SetIntRisingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTTYPESET = (1 << Num); /* Set INT TYPE bit */
//       CMSDK_GPIO->INTPOLSET = (1 << Num);  /* Set INT POLarity bit */
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to set the Interrupt type
// * @return none
// *
// * @brief  Changes the interrupt type for the specified pin to a low level interrupt. This function is thread safe.
// */
//
// void CMSDK_gpio_SetIntLowLevel(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTTYPECLR = (1 << Num);  /* Clear INT TYPE bit */
//       CMSDK_GPIO->INTPOLCLR = (1 << Num);   /* Clear INT POLarity bit */
// }
//
///**
// *
// * @param *CMSDK_GPIO GPIO Pointer
// * @param Num The pin number for which to set the Interrupt type
// * @return none
// *
// * @brief  Changes the interrupt type for the specified pin to a falling edge interrupt. This function is thread safe.
// */
//
// void CMSDK_gpio_SetIntFallingEdge(CMSDK_GPIO_TypeDef *CMSDK_GPIO, uint32_t Num)
// {
//       CMSDK_GPIO->INTTYPESET = (1 << Num);  /* Set INT TYPE bit */
//       CMSDK_GPIO->INTPOLCLR = (1 << Num);   /* Clear INT POLarity bit */
// }


/*@}*/ /* end of group CMSDK Driver definitions */
