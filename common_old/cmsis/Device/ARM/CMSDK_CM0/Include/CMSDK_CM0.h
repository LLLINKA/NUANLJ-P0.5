/**************************************************************************//**
 * @file     CMSDK_CM0.h
 * @brief    CMSIS Cortex-M0 Core Peripheral Access Layer Header File for
 *           Device CMSDK
 * @version  V3.01
 * @date     06. March 2012
 *
 * @note
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef CMSDK_H
#define CMSDK_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup CMSDK_Definitions CMSDK Definitions
  This file defines all structures and symbols for CMSDK:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup CMSDK_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/

/* ToDo: use this Cortex interrupt numbers if your device is a CORTEX-M0 device                   */
  NonMaskableInt_IRQn           = -14,      /*!<  2 Cortex-M0 Non Maskable Interrupt              */
  HardFault_IRQn                = -13,      /*!<  3 Cortex-M0 Hard Fault Interrupt                */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M0 SV Call Interrupt                   */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M0 Pend SV Interrupt                   */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M0 System Tick Interrupt               */

/******  CMSDK Specific Interrupt Numbers *********************************************************/
  UARTRX0_IRQn                  = 0,       /*!< UART 0 RX Interrupt                               */
  UARTTX0_IRQn                  = 1,       /*!< UART 0 TX Interrupt                               */
  UARTOVF0_IRQn                 = 2,       /*!< UART Overflow Interrupt                           */
  NFC_IRQn                      = 3,       /*!< NFC Interrupt                                     */
  NFC_POR_IRQn                  = 4,       /*!< NFC POWER Interrupt                               */
  IMEAS_IRQn                    = 5,       /*!< IMEASInterrupt                                    */
  ZMEAS_IRQn                    = 6,       /*!< ZMEAS interrupt                                   */
  EXT_WAKE_IRQn                 = 7,       /*!< external wake interrupt                           */
  TIMER0_IRQn                   = 8,       /*!< TIMER 0 Interrupt                                 */
  TIMER1_IRQn                   = 9,       /*!< TIMER 1 Interrupt                                 */
  TIMER2_IRQn                   = 10,      /*!< TIMER 2 Interrupt                                 */
  TIMER3_IRQn                   = 11,      /*!< TIMER 3 Interrupt                                 */
  ZMEAS_ADC_IRQn                = 12,      /*!< ZMEAS ADC Interrupt                               */
  PSW_WAKE_IRQn                 = 13,      /*!< PSW WAKE Interrupt                                */
  BOD15_IRQn                    = 14,      /*!< BOD15 Interrupt                                   */
  BOD30_IRQn                    = 15,      /*!< BOD30 Interrupt                                   */
  PORT0_0_IRQn                  = 16,      /*!< All P0 I/O pins can be used as interrupt source.  */
  PORT0_1_IRQn                  = 17,      /*!< There are 16 pins in total                        */
  PORT0_2_IRQn                  = 18,
  PORT0_3_IRQn                  = 19,
  PORT0_4_IRQn                  = 20,
  PORT0_5_IRQn                  = 21,
  PORT0_6_IRQn                  = 22,
  PORT0_7_IRQn                  = 23,
  PORT0_8_IRQn                  = 24,
  PORT0_9_IRQn                  = 25,
  PORT0_10_IRQn                 = 26,
  PORT0_11_IRQn                 = 27,
  PORT0_12_IRQn                 = 28,
  PORT0_13_IRQn                 = 29,
  PORT0_14_IRQn                 = 30,
  PORT0_15_IRQn                 = 31
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                 0x0000    /*!< Core Revision r0p0                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */

/*@}*/ /* end of group CMSDK_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_CMSDK_CM0.h"               /* CMSDK System include file                          */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup CMSDK_Peripherals CMSDK Peripherals
  CMSDK Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
/** @addtogroup CMSDK_UART CMSDK Universal Asynchronous Receiver/Transmitter
  memory mapped structure for CMSDK_UART
  @{
*/
typedef struct
{
  __IO   uint32_t  DATA;          /*!< Offset: 0x000 Data Register    (R/W) */
  __IO   uint32_t  STATE;         /*!< Offset: 0x004 Status Register  (R/W) */
  __IO   uint32_t  CTRL;          /*!< Offset: 0x008 Control Register (R/W) */
  union {
    __I    uint32_t  INTSTATUS;   /*!< Offset: 0x00C Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLEAR;    /*!< Offset: 0x00C Interrupt Clear Register ( /W) */
    };
  __IO   uint32_t  BAUDDIV;       /*!< Offset: 0x010 Baudrate Divider Register (R/W) */

} CMSDK_UART_TypeDef;

/* CMSDK_UART DATA Register Definitions */

#define CMSDK_UART_DATA_Pos               0                                             /*!< CMSDK_UART_DATA_Pos: DATA Position */
#define CMSDK_UART_DATA_Msk              (0xFFul << CMSDK_UART_DATA_Pos)                /*!< CMSDK_UART DATA: DATA Mask */

#define CMSDK_UART_STATE_RXOR_Pos         3                                             /*!< CMSDK_UART STATE: RXOR Position */
#define CMSDK_UART_STATE_RXOR_Msk         (0x1ul << CMSDK_UART_STATE_RXOR_Pos)          /*!< CMSDK_UART STATE: RXOR Mask */

#define CMSDK_UART_STATE_TXOR_Pos         2                                             /*!< CMSDK_UART STATE: TXOR Position */
#define CMSDK_UART_STATE_TXOR_Msk         (0x1ul << CMSDK_UART_STATE_TXOR_Pos)          /*!< CMSDK_UART STATE: TXOR Mask */

#define CMSDK_UART_STATE_RXBF_Pos         1                                             /*!< CMSDK_UART STATE: RXBF Position */
#define CMSDK_UART_STATE_RXBF_Msk         (0x1ul << CMSDK_UART_STATE_RXBF_Pos)          /*!< CMSDK_UART STATE: RXBF Mask */

#define CMSDK_UART_STATE_TXBF_Pos         0                                             /*!< CMSDK_UART STATE: TXBF Position */
#define CMSDK_UART_STATE_TXBF_Msk         (0x1ul << CMSDK_UART_STATE_TXBF_Pos )         /*!< CMSDK_UART STATE: TXBF Mask */

#define CMSDK_UART_CTRL_HSTM_Pos          6                                             /*!< CMSDK_UART CTRL: HSTM Position */
#define CMSDK_UART_CTRL_HSTM_Msk          (0x01ul << CMSDK_UART_CTRL_HSTM_Pos)          /*!< CMSDK_UART CTRL: HSTM Mask */

#define CMSDK_UART_CTRL_RXORIRQEN_Pos     5                                             /*!< CMSDK_UART CTRL: RXORIRQEN Position */
#define CMSDK_UART_CTRL_RXORIRQEN_Msk     (0x01ul << CMSDK_UART_CTRL_RXORIRQEN_Pos)     /*!< CMSDK_UART CTRL: RXORIRQEN Mask */

#define CMSDK_UART_CTRL_TXORIRQEN_Pos     4                                             /*!< CMSDK_UART CTRL: TXORIRQEN Position */
#define CMSDK_UART_CTRL_TXORIRQEN_Msk     (0x01ul << CMSDK_UART_CTRL_TXORIRQEN_Pos)     /*!< CMSDK_UART CTRL: TXORIRQEN Mask */

#define CMSDK_UART_CTRL_RXIRQEN_Pos       3                                             /*!< CMSDK_UART CTRL: RXIRQEN Position */
#define CMSDK_UART_CTRL_RXIRQEN_Msk       (0x01ul << CMSDK_UART_CTRL_RXIRQEN_Pos)       /*!< CMSDK_UART CTRL: RXIRQEN Mask */

#define CMSDK_UART_CTRL_TXIRQEN_Pos       2                                             /*!< CMSDK_UART CTRL: TXIRQEN Position */
#define CMSDK_UART_CTRL_TXIRQEN_Msk       (0x01ul << CMSDK_UART_CTRL_TXIRQEN_Pos)       /*!< CMSDK_UART CTRL: TXIRQEN Mask */

#define CMSDK_UART_CTRL_RXEN_Pos          1                                             /*!< CMSDK_UART CTRL: RXEN Position */
#define CMSDK_UART_CTRL_RXEN_Msk          (0x01ul << CMSDK_UART_CTRL_RXEN_Pos)          /*!< CMSDK_UART CTRL: RXEN Mask */

#define CMSDK_UART_CTRL_TXEN_Pos          0                                             /*!< CMSDK_UART CTRL: TXEN Position */
#define CMSDK_UART_CTRL_TXEN_Msk          (0x01ul << CMSDK_UART_CTRL_TXEN_Pos)          /*!< CMSDK_UART CTRL: TXEN Mask */

#define CMSDK_UART_INTSTATUS_RXORIRQ_Pos  3                                             /*!< CMSDK_UART CTRL: RXORIRQ Position */
#define CMSDK_UART_CTRL_RXORIRQ_Msk       (0x01ul << CMSDK_UART_INTSTATUS_RXORIRQ_Pos)  /*!< CMSDK_UART CTRL: RXORIRQ Mask */

#define CMSDK_UART_CTRL_TXORIRQ_Pos       2                                             /*!< CMSDK_UART CTRL: TXORIRQ Position */
#define CMSDK_UART_CTRL_TXORIRQ_Msk       (0x01ul << CMSDK_UART_CTRL_TXORIRQ_Pos)       /*!< CMSDK_UART CTRL: TXORIRQ Mask */

#define CMSDK_UART_CTRL_RXIRQ_Pos         1                                             /*!< CMSDK_UART CTRL: RXIRQ Position */
#define CMSDK_UART_CTRL_RXIRQ_Msk         (0x01ul << CMSDK_UART_CTRL_RXIRQ_Pos)         /*!< CMSDK_UART CTRL: RXIRQ Mask */

#define CMSDK_UART_CTRL_TXIRQ_Pos         0                                             /*!< CMSDK_UART CTRL: TXIRQ Position */
#define CMSDK_UART_CTRL_TXIRQ_Msk         (0x01ul << CMSDK_UART_CTRL_TXIRQ_Pos)         /*!< CMSDK_UART CTRL: TXIRQ Mask */

#define CMSDK_UART_BAUDDIV_Pos            0                                             /*!< CMSDK_UART BAUDDIV: BAUDDIV Position */
#define CMSDK_UART_BAUDDIV_Msk           (0xFFFFFul << CMSDK_UART_BAUDDIV_Pos)          /*!< CMSDK_UART BAUDDIV: BAUDDIV Mask */

/*@}*/ /* end of group CMSDK_UART */


/*----------------------------- Timer (TIMER) -------------------------------*/
/** @addtogroup CMSDK_TIMER CMSDK Timer
  @{
*/
typedef struct
{
  __IO   uint32_t  CTRL;          /*!< Offset: 0x000 Control Register (R/W) */
  __IO   uint32_t  VALUE;         /*!< Offset: 0x004 Current Value Register (R/W) */
  __IO   uint32_t  RELOAD;        /*!< Offset: 0x008 Reload Value Register  (R/W) */
  union {
    __I    uint32_t  INTSTATUS;   /*!< Offset: 0x00C Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLEAR;    /*!< Offset: 0x00C Interrupt Clear Register ( /W) */
    };

} CMSDK_TIMER_TypeDef;

/* CMSDK_TIMER CTRL Register Definitions */

#define CMSDK_TIMER_CTRL_IRQEN_Pos          3                                              /*!< CMSDK_TIMER CTRL: IRQEN Position */
#define CMSDK_TIMER_CTRL_IRQEN_Msk          (0x01ul << CMSDK_TIMER_CTRL_IRQEN_Pos)         /*!< CMSDK_TIMER CTRL: IRQEN Mask */

#define CMSDK_TIMER_CTRL_WAKE_EN_Pos        2                                              /*!< CMSDK_TIMER CTRL: WAKE_EN Position */
#define CMSDK_TIMER_CTRL_WAKE_EN_Msk        (0x01ul << CMSDK_TIMER_CTRL_WAKE_EN_Pos)       /*!< CMSDK_TIMER CTRL: WAKE_EN Mask */

#define CMSDK_TIMER_CTRL_SELEXTCLK_Pos      2                                              /*!< CMSDK_TIMER CTRL: SELEXTCLK Position */
#define CMSDK_TIMER_CTRL_SELEXTCLK_Msk      (0x01ul << CMSDK_TIMER_CTRL_SELEXTCLK_Pos)     /*!< CMSDK_TIMER CTRL: SELEXTCLK Mask */

#define CMSDK_TIMER_CTRL_SELEXTEN_Pos       1                                              /*!< CMSDK_TIMER CTRL: SELEXTEN Position */
#define CMSDK_TIMER_CTRL_SELEXTEN_Msk       (0x01ul << CMSDK_TIMER_CTRL_SELEXTEN_Pos)      /*!< CMSDK_TIMER CTRL: SELEXTEN Mask */

#define CMSDK_TIMER_CTRL_RESET_Pos          1                                              /*!< CMSDK_TIMER CTRL: RESET Position */
#define CMSDK_TIMER_CTRL_RESET_Msk          (0x01ul << CMSDK_TIMER_CTRL_RESET_Pos)         /*!< CMSDK_TIMER CTRL: RESET Mask */

#define CMSDK_TIMER_CTRL_EN_Pos             0                                              /*!< CMSDK_TIMER CTRL: EN Position */
#define CMSDK_TIMER_CTRL_EN_Msk             (0x01ul << CMSDK_TIMER_CTRL_EN_Pos)            /*!< CMSDK_TIMER CTRL: EN Mask */

#define CMSDK_TIMER_VAL_CURRENT_Pos         0                                              /*!< CMSDK_TIMER VALUE: CURRENT Position */
#define CMSDK_TIMER_VAL_CURRENT_Msk         (0xFFFFFFFFul << CMSDK_TIMER_VAL_CURRENT_Pos)  /*!< CMSDK_TIMER VALUE: CURRENT Mask */

#define CMSDK_TIMER_RELOAD_VAL_Pos          0                                              /*!< CMSDK_TIMER RELOAD: RELOAD Position */
#define CMSDK_TIMER_RELOAD_VAL_Msk          (0xFFFFFFFFul << CMSDK_TIMER_RELOAD_VAL_Pos)   /*!< CMSDK_TIMER RELOAD: RELOAD Mask */

#define CMSDK_TIMER_INTSTATUS_Pos           0                                              /*!< CMSDK_TIMER INTSTATUS: INTSTATUSPosition */
#define CMSDK_TIMER_INTSTATUS_Msk           (0x01ul << CMSDK_TIMER_INTSTATUS_Pos)          /*!< CMSDK_TIMER INTSTATUS: INTSTATUSMask */

#define CMSDK_TIMER_INTCLEAR_Pos            0                                              /*!< CMSDK_TIMER INTCLEAR: INTCLEAR Position */
#define CMSDK_TIMER_INTCLEAR_Msk            (0x01ul << CMSDK_TIMER_INTCLEAR_Pos)           /*!< CMSDK_TIMER INTCLEAR: INTCLEAR Mask */

/*@}*/ /* end of group CMSDK_TIMER */



/*-------------------- General Purpose Input Output (GPIO) -------------------*/

/** @addtogroup CMSDK_GPIO CMSDK GPIO
  @{
*/
typedef struct
{
  __IO   uint32_t  INENABLESET;     /*!< Offset: 0x000 Input Enable Set register (R/W) */
  __IO   uint32_t  OUTENABLESET;    /*!< Offset: 0x004 Output Enable Set register (R/W) */
  __IO   uint32_t  DATATOPAD;       /*!< Offset: 0x008 To Pad value Register  (R/W) */
  __IO   uint32_t  OPENDRAINEN;     /*!< Offset: 0x00C Open Drain Enable Register  (R/W) */
  __IO   uint32_t  PULLUPEN;        /*!< Offset: 0x010 Pull UP Enable Register  (R/W) */
  __IO   uint32_t  PULLDOWNEN;      /*!< Offset: 0x014 Pull DOWN Enable Register  (R/W) */
  __IO   uint32_t  DRVSTRENGTH;     /*!< Offset: 0x018 Pad Driven Strength Set Register  (R/W) */
  __IO   uint32_t  ALTFUNCSET;      /*!< Offset: 0x01C Atlernate Function Enable Register  (R/W) */
  __IO   uint32_t  INTTYPESET;      /*!< Offset: 0x020 Interrupt Type Set Register  (R/W) */
  __IO   uint32_t  INTPOLSET;       /*!< Offset: 0x024 Interrupt Polarity Set Register  (R/W) */
  __IO   uint32_t  INTENSET;        /*!< Offset: 0x028 Interrupt Enable Register  (R/W) */
  union {
    __I    uint32_t  INTSTATUS;     /*!< Offset: 0x02C Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLEAR;      /*!< Offset: 0x02C Interrupt Clear Register ( /W) */
    };
  __I    uint32_t  DATAFROMPAD;     /*!< Offset: 0x030 From Pad value Register  (R/) */
  __IO   uint32_t  AE0;             /*!< Offset: 0x034 Analog Enable_0 Register  (R/W) */
  __IO   uint32_t  AE1;             /*!< Offset: 0x038 Analog Enable_1 Register  (R/W) */
} CMSDK_GPIO_TypeDef;

#define CMSDK_GPIO_DATA_Pos            0                                          /*!< CMSDK_GPIO DATA: DATA Position */
#define CMSDK_GPIO_DATA_Msk            (0xFFFFul << CMSDK_GPIO_DATA_Pos)          /*!< CMSDK_GPIO DATA: DATA Mask */

#define CMSDK_GPIO_DATAOUT_Pos         0                                          /*!< CMSDK_GPIO DATAOUT: DATAOUT Position */
#define CMSDK_GPIO_DATAOUT_Msk         (0xFFFFul << CMSDK_GPIO_DATAOUT_Pos)       /*!< CMSDK_GPIO DATAOUT: DATAOUT Mask */

#define CMSDK_GPIO_OUTENSET_Pos        0                                          /*!< CMSDK_GPIO OUTEN: OUTEN Position */
#define CMSDK_GPIO_OUTENSET_Msk        (0xFFFFul << CMSDK_GPIO_OUTEN_Pos)         /*!< CMSDK_GPIO OUTEN: OUTEN Mask */

#define CMSDK_GPIO_OUTENCLR_Pos        0                                          /*!< CMSDK_GPIO OUTEN: OUTEN Position */
#define CMSDK_GPIO_OUTENCLR_Msk        (0xFFFFul << CMSDK_GPIO_OUTEN_Pos)         /*!< CMSDK_GPIO OUTEN: OUTEN Mask */

#define CMSDK_GPIO_ALTFUNCSET_Pos      0                                          /*!< CMSDK_GPIO ALTFUNC: ALTFUNC Position */
#define CMSDK_GPIO_ALTFUNCSET_Msk      (0xFFFFul << CMSDK_GPIO_ALTFUNC_Pos)       /*!< CMSDK_GPIO ALTFUNC: ALTFUNC Mask */

#define CMSDK_GPIO_ALTFUNCCLR_Pos      0                                          /*!< CMSDK_GPIO ALTFUNC: ALTFUNC Position */
#define CMSDK_GPIO_ALTFUNCCLR_Msk      (0xFFFFul << CMSDK_GPIO_ALTFUNC_Pos)       /*!< CMSDK_GPIO ALTFUNC: ALTFUNC Mask */

#define CMSDK_GPIO_INTENSET_Pos        0                                          /*!< CMSDK_GPIO INTEN: INTEN Position */
#define CMSDK_GPIO_INTENSET_Msk        (0xFFFFul << CMSDK_GPIO_INTEN_Pos)         /*!< CMSDK_GPIO INTEN: INTEN Mask */

#define CMSDK_GPIO_INTENCLR_Pos        0                                          /*!< CMSDK_GPIO INTEN: INTEN Position */
#define CMSDK_GPIO_INTENCLR_Msk        (0xFFFFul << CMSDK_GPIO_INTEN_Pos)         /*!< CMSDK_GPIO INTEN: INTEN Mask */

#define CMSDK_GPIO_INTTYPESET_Pos      0                                          /*!< CMSDK_GPIO INTTYPE: INTTYPE Position */
#define CMSDK_GPIO_INTTYPESET_Msk      (0xFFFFul << CMSDK_GPIO_INTTYPE_Pos)       /*!< CMSDK_GPIO INTTYPE: INTTYPE Mask */

#define CMSDK_GPIO_INTTYPECLR_Pos      0                                          /*!< CMSDK_GPIO INTTYPE: INTTYPE Position */
#define CMSDK_GPIO_INTTYPECLR_Msk      (0xFFFFul << CMSDK_GPIO_INTTYPE_Pos)       /*!< CMSDK_GPIO INTTYPE: INTTYPE Mask */

#define CMSDK_GPIO_INTPOLSET_Pos       0                                          /*!< CMSDK_GPIO INTPOL: INTPOL Position */
#define CMSDK_GPIO_INTPOLSET_Msk       (0xFFFFul << CMSDK_GPIO_INTPOL_Pos)        /*!< CMSDK_GPIO INTPOL: INTPOL Mask */

#define CMSDK_GPIO_INTPOLCLR_Pos       0                                          /*!< CMSDK_GPIO INTPOL: INTPOL Position */
#define CMSDK_GPIO_INTPOLCLR_Msk       (0xFFFFul << CMSDK_GPIO_INTPOL_Pos)        /*!< CMSDK_GPIO INTPOL: INTPOL Mask */

#define CMSDK_GPIO_INTSTATUS_Pos       0                                          /*!< CMSDK_GPIO INTSTATUS: INTSTATUS Position */
#define CMSDK_GPIO_INTSTATUS_Msk       (0xFFul << CMSDK_GPIO_INTSTATUS_Pos)       /*!< CMSDK_GPIO INTSTATUS: INTSTATUS Mask */

#define CMSDK_GPIO_INTCLEAR_Pos        0                                          /*!< CMSDK_GPIO INTCLEAR: INTCLEAR Position */
#define CMSDK_GPIO_INTCLEAR_Msk        (0xFFul << CMSDK_GPIO_INTCLEAR_Pos)        /*!< CMSDK_GPIO INTCLEAR: INTCLEAR Mask */

/*@}*/ /* end of group CMSDK_GPIO */


/*------------- System Control (SYSCON) --------------------------------------*/
/** @addtogroup CMSDK_SYSCON CMSDK System Control
  @{
*/
typedef struct
{
  __IO   uint32_t  FUNCEN;          /*!< Offset: 0x000 Function Enable Register (R/W) */
  __IO   uint32_t  CLKDIV;          /*!< Offset: 0x004 Clock Divider Register (R/W) */
  __IO   uint32_t  PMUCTRL;         /*!< Offset: 0x008 PMU Control Register (R/W) */
  union {
    __I    uint32_t  PMUSTATUS;     /*!< Offset: 0x00C PMU Status Register (R/ ) */
    __O    uint32_t  PMUCLEAR;      /*!< Offset: 0x00C PMU status Clear Register ( /W) */
    };
  __IO   uint32_t  CLKSTABLE;       /*!< Offset: 0x010 Clock Stable Value Register  (R/W) */
  __IO   uint32_t  LFOSCCTRL;       /*!< Offset: 0x014 LFOSC Control Register  (R/W) */
  __IO   uint32_t  HFOSCCTRL;       /*!< Offset: 0x018 HFOSC Control Register  (R/W) */
  __IO   uint32_t  ANACTRL;         /*!< Offset: 0x01C Analog Control Register  (R/W) */
} CMSDK_SYSCON_TypeDef;

/* CMSDK_SYSCON  Register Definitions */

#define CMSDK_SYSCON_ANACTRL_BATOFF_Pos        1
#define CMSDK_SYSCON_ANACTRL_BATOFF_Msk       (0x00001ul << CMSDK_SYSCON_ANACTRL_BATOFF_Pos)       /*!< CMSDK_SYSCON FUNCEN: BAT_OFF Mask */

#define CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Pos 	2
#define CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Msk (0x00001ul << CMSDK_SYSCON_ANACTRL_MEAS_MODE_SEL_Pos) /*!< CMSDK_SYSCON FUNCEN: signal for switching the current measurement mode(0) or  impedance measurement mode(1)*/

#define CMSDK_SYSCON_REMAP0_Pos                0
#define CMSDK_SYSCON_REMAP0_Msk                (0x00001ul << CMSDK_SYSCON_REMAP0_Pos)               /*!< CMSDK_SYSCON FUNCEN: REMAP0 Mask */
#define CMSDK_SYSCON_REMAP_Msk                 (0x00003ul << CMSDK_SYSCON_REMAP0_Pos)

#define CMSDK_SYSCON_REMAP1_Pos                1
#define CMSDK_SYSCON_REMAP1_Msk                (0x00001ul << CMSDK_SYSCON_REMAP1_Pos)               /*!< CMSDK_SYSCON FUNCEN: REMAP1 Mask */

#define CMSDK_SYSCON_LOCKUPRST_RESETOP_Pos     0
#define CMSDK_SYSCON_LOCKUPRST_RESETOP_Msk     (0x00001ul << CMSDK_SYSCON_LOCKUPRST_RESETOP_Pos)   /*!< CMSDK_SYSCON SYS_CTRL: LOCKUP RESET ENABLE Mask */

#define CMSDK_SYSCON_EMICTRL_SIZE_Pos          24
#define CMSDK_SYSCON_EMICTRL_SIZE_Msk          (0x00001ul << CMSDK_SYSCON_EMICTRL_SIZE_Pos)     /*!< CMSDK_SYSCON EMICTRL: SIZE Mask */

#define CMSDK_SYSCON_EMICTRL_TACYC_Pos         16
#define CMSDK_SYSCON_EMICTRL_TACYC_Msk         (0x00007ul << CMSDK_SYSCON_EMICTRL_TACYC_Pos)    /*!< CMSDK_SYSCON EMICTRL: TURNAROUNDCYCLE Mask */

#define CMSDK_SYSCON_EMICTRL_WCYC_Pos          8
#define CMSDK_SYSCON_EMICTRL_WCYC_Msk          (0x00003ul << CMSDK_SYSCON_EMICTRL_WCYC_Pos)     /*!< CMSDK_SYSCON EMICTRL: WRITECYCLE Mask */

#define CMSDK_SYSCON_EMICTRL_RCYC_Pos          0
#define CMSDK_SYSCON_EMICTRL_RCYC_Msk          (0x00007ul << CMSDK_SYSCON_EMICTRL_RCYC_Pos)     /*!< CMSDK_SYSCON EMICTRL: READCYCLE Mask */

#define CMSDK_SYSCON_RSTINFO_SYSRESETREQ_Pos   0
#define CMSDK_SYSCON_RSTINFO_SYSRESETREQ_Msk   (0x00001ul << CMSDK_SYSCON_RSTINFO_SYSRESETREQ_Pos) /*!< CMSDK_SYSCON RSTINFO: SYSRESETREQ Mask */

#define CMSDK_SYSCON_RSTINFO_WDOGRESETREQ_Pos  1
#define CMSDK_SYSCON_RSTINFO_WDOGRESETREQ_Msk  (0x00001ul << CMSDK_SYSCON_RSTINFO_WDOGRESETREQ_Pos) /*!< CMSDK_SYSCON RSTINFO: WDOGRESETREQ Mask */

#define CMSDK_SYSCON_RSTINFO_LOCKUPRESET_Pos   2
#define CMSDK_SYSCON_RSTINFO_LOCKUPRESET_Msk   (0x00001ul << CMSDK_SYSCON_RSTINFO_LOCKUPRESET_Pos) /*!< CMSDK_SYSCON RSTINFO: LOCKUPRESET Mask */

#define CMSDK_SYSCON_PMUCTRL_WAKEUP_HF_EN_Pos  6
#define CMSDK_SYSCON_PMUCTRL_WAKEUP_HF_EN_Msk  (0x00001ul << CMSDK_SYSCON_PMUCTRL_WAKEUP_HF_EN_Pos) /*!< CMSDK_SYSCON PMUCTRL: WAKEUP_HF_EN Mask */

#define CMSDK_SYSCON_NFC_POR_MOD_Pos           9
#define CMSDK_SYSCON_NFC_POR_MOD_Msk           (0x00001ul << CMSDK_SYSCON_NFC_POR_MOD_Pos) /*!< CMSDK_SYSCON PMUCTRL : NFC_POR_MOD Mask */

#define CMSDK_SYSCON_NFC_POR_WAKE_EN_Pos       7
#define CMSDK_SYSCON_NFC_POR_WAKE_EN_Msk       (0x00001ul << CMSDK_SYSCON_NFC_POR_WAKE_EN_Pos) /*!< CMSDK_SYSCON PMUCTRL : NFC_POR_WAKE_EN Mask */

#define CMSDK_SYSCON_NFC_POR_INT_EN_Pos        2
#define CMSDK_SYSCON_NFC_POR_INT_EN_Msk        (0x00001ul << CMSDK_SYSCON_NFC_POR_INT_EN_Pos) /*!< CMSDK_SYSCON PMUCTRL : NFC_POR_INT_EN Mask */

#define CMSDK_SYSCON_NFC_POR_STS_Pos           5
#define CMSDK_SYSCON_NFC_POR_STS_Msk           (0x00001ul << CMSDK_SYSCON_NFC_POR_STS_Pos) /*!< CMSDK_SYSCON PMUSTATUS : NFC_POR_STS Mask */

#define CMSDK_SYSCON_IMEAS_ADC_CLK_DIV_Pos     4
#define CMSDK_SYSCON_IMEAS_ADC_CLK_DIV_Msk     (0x00007ul << CMSDK_SYSCON_IMEAS_ADC_CLK_DIV_Pos) /*!< CMSDK_SYSCON CLOCK DIVIDER : IMEAS_ADC_CLK_DIV Mask */

#define CMSDK_SYSCON_HCLK_DIV_Pos              2
#define CMSDK_SYSCON_HCLK_DIV_Msk              (0x00003ul << CMSDK_SYSCON_HCLK_DIV_Pos) /*!< CMSDK_SYSCON CLOCK DIVIDER : HCLK_DIV 1MHz Mask */

#define CMSDK_SYSCON_PCLK_DIV_Pos              0
#define CMSDK_SYSCON_PCLK_DIV_Msk              (0x00003ul << CMSDK_SYSCON_PCLK_DIV_Pos) /*!< CMSDK_SYSCON CLOCK DIVIDER : PCLK_DIV 1/8 HCLK Mask */

#define CMSDK_SYSCON_EXT_WAKE_INTEN_Pos        3
#define CMSDK_SYSCON_EXT_WAKE_INTEN_Msk        (0x00001ul <<  CMSDK_SYSCON_EXT_WAKE_INTEN_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_EXT_WAKE_MOD_Pos          11
#define CMSDK_SYSCON_EXT_WAKE_MOD_Msk          (0x00001ul <<  CMSDK_SYSCON_EXT_WAKE_MOD_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_EXT_WAKE_EN_Pos           8
#define CMSDK_SYSCON_EXT_WAKE_EN_Msk           (0x00001ul <<  CMSDK_SYSCON_EXT_WAKE_EN_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_EXT_WAKE_STS_Pos          6
#define CMSDK_SYSCON_EXT_WAKE_STS_Msk          (0x00001ul << CMSDK_SYSCON_EXT_WAKE_STS_Pos) /*!< CMSDK_SYSCON PMUSTATUS : EXTERNAL WAKE status bit*/

#define CMSDK_SYSCON_PSW_WAKE_INTEN_Pos        20
#define CMSDK_SYSCON_PSW_WAKE_INTEN_Msk        (0x00001ul <<  CMSDK_SYSCON_PSW_WAKE_INTEN_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_PSW_WAKE_MOD_Pos          22
#define CMSDK_SYSCON_PSW_WAKE_MOD_Msk          (0x00001ul <<  CMSDK_SYSCON_PSW_WAKE_MOD_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_PSW_WAKE_EN_Pos           21
#define CMSDK_SYSCON_PSW_WAKE_EN_Msk           (0x00001ul <<  CMSDK_SYSCON_PSW_WAKE_EN_Pos) /*!< CMSDK_SYSCON EXTERNAL WAKE INTERRUPT ENABLE Mask */

#define CMSDK_SYSCON_PSW_WAKE_STS_Pos          7
#define CMSDK_SYSCON_PSW_WAKE_STS_Msk          (0x00001ul << CMSDK_SYSCON_PSW_WAKE_STS_Pos) /*!< CMSDK_SYSCON PMUSTATUS : EXTERNAL WAKE status bit*/

/*@}*/ /* end of group CMSDK_SYSCON */

/*   cmsdk_apb_zmeas   */
/*----------------------------- AC Impedance Measurement (ZMEAS) -------------------------------*/
/** @addtogroup CMSDK_ZMEAS CMSDK Zmeas
  @{
*/
typedef struct
{
  __IO   uint32_t  REG_CTRL;                           /*!< Offset: 0x000 Control  Register (R/W) */
  __I    uint32_t  REG_STATUS ;                        /*!< Offset: 0x004 Status  Register (R/) */ 
  __I    uint32_t  REG_DATAOUT;                        /*!< Offset: 0x008 Dataout  Register (R/) */
  __I    uint32_t  ADC_ROM_REG;                        /*!< Offset: 0x00C ADC ROM data out Register (R/) */
  __I    uint32_t  SUMMATION_OFFSET;                   /*!< Offset: 0x010 Summation offset  Register (R/) */
  __I    uint32_t  SUMMATION_REAL;                     /*!< Offset: 0x014 Summation real  Register (R/) */
  __I    uint32_t  SUMMATION_IMAG;                     /*!< Offset: 0x018 Summation imag  Register (R/) */
  __I    uint32_t  DFT_CNT_REG;                        /*!< Offset: 0x01C Shifted real and dft count  Register (R/) */
  union {
    __I    uint32_t  INTSTATUS;                        /*!< Offset: 0x020 Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLEAR;                         /*!< Offset: 0x020 Interrupt Clear Register ( /W) */
    };
  union {
    __I    uint32_t  ADCINTSTATUS;                     /*!< Offset: 0x024 Interrupt Status Register (R/ ) */
    __O    uint32_t  ADCINTCLEAR;                      /*!< Offset: 0x024 Interrupt Clear Register ( /W) */
    };
} CMSDK_ZMEAS_TypeDef;

/* CMSDK_ZMEAS  Register Definitions */
                  
#define CMSDK_ZMEAS_INTCLEAR_Pos                              0                                              /*!< CMSDK_IMEAS INTCLEAR: INTCLEAR Position */
#define CMSDK_ZMEAS_INTCLEAR_Msk                              (0x00000001ul << CMSDK_ZMEAS_INTCLEAR_Pos)     /*!< CMSDK_IMEAS INTCLEAR: INTCLEAR Mask */
 
#define CMSDK_ZMEAS_ADC_DATA_Pos                              20                                              
#define CMSDK_ZMEAS_ADC_DATA_Msk                              (0x000003FFul << CMSDK_ZMEAS_ADC_DATA_Pos)

#define CMSDK_ZMEAS_COSINE_Pos                                0                                              
#define CMSDK_ZMEAS_COSINE_Msk                                (0x000003FFul << CMSDK_ZMEAS_COSINE_Pos)

#define CMSDK_ZMEAS_SINE_Pos                                  10                                              
#define CMSDK_ZMEAS_SINE_Msk                                  (0x000003FFul << CMSDK_ZMEAS_SINE_Pos)

#define CMSDK_ZMEAS_DFT_CNT_Pos                               0                                              
#define CMSDK_ZMEAS_DFT_CNT_Msk                               (0x00000FFFul << CMSDK_ZMEAS_DFT_CNT_Pos)

#define CMSDK_ZMEAS_SHIFTED_REAL_Pos                          12                                              
#define CMSDK_ZMEAS_SHIFTED_REAL_Msk                          (0x0001FFFFul << CMSDK_ZMEAS_DFT_CNT_Pos)

#define CMSDK_ZMEAS_REG_STATUS_FREQ_MEAS_DONE_Pos             2                                              
#define CMSDK_ZMEAS_REG_STATUS_FREQ_MEAS_DONE_Msk             (0x00000001ul << CMSDK_ZMEAS_REG_STATUS_FREQ_MEAS_DONE_Pos)

#define CMSDK_ZMEAS_REG_STATUS_DDS_EN_Pos                     3                                              
#define CMSDK_ZMEAS_REG_STATUS_DDS_EN_Msk                     (0x00000001ul << CMSDK_ZMEAS_REG_STATUS_DDS_EN_Pos)

#define CMSDK_ZMEAS_REG_STATUS_ADC_EN_Pos                     4                                              
#define CMSDK_ZMEAS_REG_STATUS_ADC_EN_Msk                     (0x00000001ul << CMSDK_ZMEAS_REG_STATUS_ADC_EN_Pos)

#define CMSDK_ZMEAS_REG_STATUS_PGA_GAIN_Pos                   5                                              
#define CMSDK_ZMEAS_REG_STATUS_PGA_GAIN_Msk                   (0x00000001ul << CMSDK_ZMEAS_REG_STATUS_PGA_GAIN_Pos)

#define CMSDK_ZMEAS_REG_STATUS_MEAS_CALIB_Pos                 6                                              
#define CMSDK_ZMEAS_REG_STATUS_MEAS_CALIB_Msk                 (0x00000001ul << CMSDK_ZMEAS_REG_STATUS_MEAS_CALIB_Pos)

#define CMSDK_ZMEAS_REG_STATUS_CONFIG_VOLT_Pos                7                                              
#define CMSDK_ZMEAS_REG_STATUS_CONFIG_VOLT_Msk                (0x00000003ul << CMSDK_ZMEAS_REG_STATUS_CONFIG_VOLT_Pos)

#define CMSDK_ZMEAS_REG_STATUS_FREQ_VAL_Pos                   9                                              
#define CMSDK_ZMEAS_REG_STATUS_FREQ_VAL_Msk                   (0x00000007ul << CMSDK_ZMEAS_REG_STATUS_FREQ_VAL_Pos)

#define CMSDK_ZMEAS_REG_STATUS_REPEAT_VAL_Pos                 12                                              
#define CMSDK_ZMEAS_REG_STATUS_REPEAT_VAL_Msk                 (0x0000000Ful << CMSDK_ZMEAS_REG_STATUS_REPEAT_VAL_Pos)

#define CMSDK_ZMEAS_REG_CTRL_REPEAT_VAL_Pos     	      28                                              
#define CMSDK_ZMEAS_REG_CTRL_REPEAT_VAL_Msk                   (0x00000000Ful << CMSDK_ZMEAS_REG_CTRL_REPEAT_VAL_Pos)

#define CMSDK_ZMEAS_REG_CTRL_SETTLING_TIME_VAL_Pos            20                                              
#define CMSDK_ZMEAS_REG_CTRL_SETTLING_TIME_VAL_Msk            (0x000000FFul << CMSDK_ZMEAS_REG_CTRL_SETTLING_TIME_VAL_Pos)

#define CMSDK_ZMEAS_REG_CTRL_FREQ_VAL_Pos                     16                                              
#define CMSDK_ZMEAS_REG_CTRL_FREQ_VAL_Msk                     (0x00000007ul << CMSDK_ZMEAS_REG_CTRL_FREQ_VAL_Pos)

#define CMSDK_ZMEAS_REG_CTRL_MODE_Pos            	      13                                              
#define CMSDK_ZMEAS_REG_CTRL_MODE_Msk                         (0x00000007ul << CMSDK_ZMEAS_REG_CTRL_MODE_Pos)

#define CMSDK_ZMEAS_REG_CTRL_REPEAT_CAL_EN_Pos                12                                              
#define CMSDK_ZMEAS_REG_CTRL_REPEAT_CAL_EN_Msk                (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_REPEAT_CAL_EN_Pos)

#define CMSDK_ZMEAS_REG_CTRL_ENABLE_INTR_Pos                  10                                              
#define CMSDK_ZMEAS_REG_CTRL_ENABLE_INTR_Msk                  (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_ENABLE_INTR_Pos)

#define CMSDK_ZMEAS_REG_CTRL_ENABLE_ADC_INTR_Pos              8                                              
#define CMSDK_ZMEAS_REG_CTRL_ENABLE_ADC_INTR_Msk              (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_ENABLE_ADC_INTR_Pos)

#define CMSDK_ZMEAS_REG_CTRL_MEAS_CALIB_Pos            	      6                                              
#define CMSDK_ZMEAS_REG_CTRL_MEAS_CALIB_Msk            	      (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_MEAS_CALIB_Pos)

#define CMSDK_ZMEAS_REG_CTRL_CONFIG_VOLT_Pos                  4                                              
#define CMSDK_ZMEAS_REG_CTRL_CONFIG_VOLT_Msk                  (0x00000003ul << CMSDK_ZMEAS_REG_CTRL_CONFIG_VOLT_Pos)

#define CMSDK_ZMEAS_REG_CTRL_PGA_GAIN_Pos            	      2                                              
#define CMSDK_ZMEAS_REG_CTRL_PGA_GAIN_Msk            	      (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_PGA_GAIN_Pos)

#define CMSDK_ZMEAS_REG_CTRL_RESET_MEASUREMENT_Pos            0                                              
#define CMSDK_ZMEAS_REG_CTRL_RESET_MEASUREMENT_Msk            (0x00000001ul << CMSDK_ZMEAS_REG_CTRL_RESET_MEASUREMENT_Pos)

/*@}*/ /* end of group CMSDK_ZMEAS */

/*   cmsdk_apb_imeas   */
/*----------------------------- Current Measurement (IMEAS) -------------------------------*/
/** @addtogroup CMSDK_IMEAS CMSDK Imeas
  @{
*/
typedef struct
{
  __IO   uint32_t  REG_CTRL;                         /*!< Offset: 0x000 Control  Register (R/W) */
  __IO   uint32_t  CHA_MODE;                         /*!< Offset: 0x004 Channel Mode  Register (R/W) */ 
  union {
    __I    uint32_t  INTSTATUS;                      /*!< Offset: 0x008 Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLEAR;                       /*!< Offset: 0x008 Interrupt Clear Register ( /W) */
    };
  __IO   uint32_t  SEQ_CTRL;                         /*!< Offset: 0x00C Sequence Control Register (R/W) */
  __IO   uint32_t  RST_CNT;                          /*!< Offset: 0x010 Reset Count Register (R/W) */
  __I    uint32_t  CHA0_DATA;                        /*!< Offset: 0x014 Channel 0 Data Register (R/) */
  __I    uint32_t  CHA1_DATA;                        /*!< Offset: 0x018 Channel 1 Data Register (R/) */
  __I    uint32_t  CHA2_DATA;                        /*!< Offset: 0x01C Channel 2 Data Register (R/) */
  
} CMSDK_IMEAS_TypeDef;

/* CMSDK_IMEAS  Register Definitions */

#define CMSDK_IMEAS_INTCLEAR_Pos            0                                              /*!< CMSDK_IMEAS INTCLEAR: INTCLEAR Position */
#define CMSDK_IMEAS_INTCLEAR_Msk            (0x00001ul << CMSDK_IMEAS_INTCLEAR_Pos)        /*!< CMSDK_IMEAS INTCLEAR: INTCLEAR Mask */

#define CMSDK_IMEAS_INT_EN_Pos              0                                              /*!< CMSDK_IMEAS CTRL: INT_EN Position */
#define CMSDK_IMEAS_INT_EN_Msk              (0x00001ul << CMSDK_IMEAS_INT_EN_Pos)          /*!< CMSDK_IMEAS CTRL: INT_EN Mask */

#define CMSDK_IMEAS_CAL_EN_Pos              1                                              /*!< CMSDK_IMEAS CTRL: CAL_EN Position */
#define CMSDK_IMEAS_CAL_EN_Msk              (0x00001ul << CMSDK_IMEAS_CAL_EN_Pos)          /*!< CMSDK_IMEAS CTRL: CAL_EN Mask */

#define CMSDK_IMEAS_PGA_GAIN_Pos            2                                              /*!< CMSDK_IMEAS CTRL: PGA_GAIN Position */
#define CMSDK_IMEAS_PGA_GAIN_Msk            (0x00003ul << CMSDK_IMEAS_PGA_GAIN_Pos)        /*!< CMSDK_IMEAS CTRL: PGA_GAIN Mask */

#define CMSDK_IMEAS_CIC_RATE_Pos            4                                              /*!< CMSDK_IMEAS CTRL: CIC_RATE Position */
#define CMSDK_IMEAS_CIC_RATE_Msk            (0x00007ul << CMSDK_IMEAS_CIC_RATE_Pos)        /*!< CMSDK_IMEAS CTRL: CIC_RATE Mask */

#define CMSDK_IMEAS_GUBIAS_EN_Pos           7                                              /*!< CMSDK_IMEAS CTRL: GUBIAS_EN Position */
#define CMSDK_IMEAS_GUBIAS_EN_Msk           (0x00001ul << CMSDK_IMEAS_GUBIAS_EN_Pos)       /*!< CMSDK_IMEAS CTRL: GUBIAS_EN Mask */

#define CMSDK_IMEAS_REBIAS_DAC_Pos          8                                              /*!< CMSDK_IMEAS CTRL: REBIAS_DAC Position */
#define CMSDK_IMEAS_REBIAS_DAC_Msk          (0x00003ul << CMSDK_IMEAS_REBIAS_DAC_Pos)      /*!< CMSDK_IMEAS CTRL: REBIAS_DAC Mask */

#define CMSDK_IMEAS_WEBIAS_DAC_Pos          10                                             /*!< CMSDK_IMEAS CTRL: WEBIAS_DAC Position */
#define CMSDK_IMEAS_WEBIAS_DAC_Msk          (0x0003Ful << CMSDK_IMEAS_WEBIAS_DAC_Pos)      /*!< CMSDK_IMEAS CTRL: WEBIAS_DAC Mask */

#define CMSDK_IMEAS_CHA_MODE_Pos            0                                              /*!< CMSDK_IMEAS CHANNEL: CHA_MODE Position */
#define CMSDK_IMEAS_CHA_MODE_Msk            (0x00003ul << CMSDK_IMEAS_CHA_MODE_Pos)        /*!< CMSDK_IMEAS CHANNEL: CHA_MODE Mask */

#define CMSDK_IMEAS_FORMAT_SEL_Pos          2                                              /*!< CMSDK_IMEAS CHANNEL: FORMAT_SEL Position */
#define CMSDK_IMEAS_FORMAT_SEL_Msk          (0x00001ul << CMSDK_IMEAS_FORMAT_SEL_Pos)      /*!< CMSDK_IMEAS CHANNEL: FORMAT_SEL Mask */

#define CMSDK_IMEAS_CHA_NUM_Pos             4                                              /*!< CMSDK_IMEAS CHANNEL: CHA_NUM Position */
#define CMSDK_IMEAS_CHA_NUM_Msk             (0x00007ul << CMSDK_IMEAS_CHA_NUM_Pos)         /*!< CMSDK_IMEAS CHANNEL: CHA_NUM Mask */

#define CMSDK_IMEAS_SD16OFF_Pos             0                                              /*!< CMSDK_IMEAS SEQ_CTRL: SD16OFF Position */
#define CMSDK_IMEAS_SD16OFF_Msk             (0x00001ul << CMSDK_IMEAS_SD16OFF_Pos)         /*!< CMSDK_IMEAS SEQ_CTRL: SD16OFF Mask */

#define CMSDK_IMEAS_SD16SLP_Pos             1                                              /*!< CMSDK_IMEAS SEQ_CTRL: SD16SLP Position */
#define CMSDK_IMEAS_SD16SLP_Msk             (0x00001ul << CMSDK_IMEAS_SD16SLP_Pos)         /*!< CMSDK_IMEAS SEQ_CTRL: SD16SLP Mask */

#define CMSDK_IMEAS_SD16RST_Pos             2                                              /*!< CMSDK_IMEAS SEQ_CTRL: SD16RST Position */
#define CMSDK_IMEAS_SD16RST_Msk             (0x00001ul << CMSDK_IMEAS_SD16RST_Pos)         /*!< CMSDK_IMEAS SEQ_CTRL: SD16RST Mask */
/*@}*/ /* end of group CMSDK_IMEAS */

/*   AHB_64K_FLASH   */
/*----------------------------- FLASH CONTROL (FLASH) -------------------------------*/
/** @addtogroup CMSDK_FLASH
  @{
*/
typedef struct
{
  __IO   uint32_t  FLASH_CTRL;                           /*!< Offset: 0x000 Flash Control  Register (R/W)  */
  __I    uint32_t  FLASH_STATUS;                         /*!< Offset: 0x004 Flash Status  Register (R/)    */ 
  __IO   uint32_t  FLASH_WDATA;                          /*!< Offset: 0x008 Flash Wdata Register (R/W )    */
  __IO   uint32_t  FLASH_WADDR;                          /*!< Offset: 0x00C Flash Waddress Register (R/W)  */
        uint32_t  RESERVED0[48];                         /*!< Offset: 0x010 to 0x0CC	Unused             */
  __IO   uint32_t  FLASH_BADSECTOR0;                     /*!< Offset: 0x0D0 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR1;                     /*!< Offset: 0x0D4 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR2;                     /*!< Offset: 0x0D8 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR3;                     /*!< Offset: 0x0DC Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR4;                     /*!< Offset: 0x0E0 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR5;                     /*!< Offset: 0x0E4 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR6;                     /*!< Offset: 0x0E8 Flash BADSECTOR Register (R/W) */
  __IO   uint32_t  FLASH_BADSECTOR7;                     /*!< Offset: 0x0EC Flash BADSECTOR Register (R/W) */
        uint32_t  RESERVED1[2];                          /*!< Offset: 0x0F0 to 0x0F4	Unused             */
  __IO   uint32_t  FLASH_BADSECTREGS_UNLOCKWORD;         /*!< Offset: 0x0F8 Flash BADSECTOR unlock word Register (R/W) */
  __IO   uint32_t  FLASH_MAGICW;                         /*!< Offset: 0x0FC Flash Magic word Register (R/W) */

} CMSDK_FLASH_TypeDef;                                                           


/* CMSDK_FLASH  Register Definitions */
#define CMSDK_FLASH_CTRL_SECTOR_SEL_Pos          7                                              
#define CMSDK_FLASH_CTRL_SECTOR_SEL_Msk          (0x00003F80ul)
                                     
#define CMSDK_FLASH_CTRL_NVR_SECTOR_SEL_Msk      (0x00000780ul)

#define CMSDK_FLASH_CTRL_UPPER16_Pos             5                                              
#define CMSDK_FLASH_CTRL_UPPER16_Msk             (0x00000001ul << CMSDK_FLASH_CTRL_UPPER16_Pos)

#define CMSDK_FLASH_CTRL_LOWER16_Pos             4                                       
#define CMSDK_FLASH_CTRL_LOWER16_Msk             (0x00000001ul << CMSDK_FLASH_CTRL_LOWER16_Pos)
//FUNCTION= PG/SER/CER
#define CMSDK_FLASH_CTRL_FUNCTION_Pos            0                                       
#define CMSDK_FLASH_CTRL_FUNCTION_Msk            (0x00000007ul << CMSDK_FLASH_CTRL_FUNCTION_Pos)

#define CMSDK_FLASH_CTRL_PG_EN_Pos               0                                       
#define CMSDK_FLASH_CTRL_PG_EN_Msk               (0x00000001ul << CMSDK_FLASH_CTRL_PG_EN_Pos)

#define CMSDK_FLASH_CTRL_SER_EN_Pos              1                                       
#define CMSDK_FLASH_CTRL_SER_EN_Msk              (0x00000001ul << CMSDK_FLASH_CTRL_SER_EN_Pos)

#define CMSDK_FLASH_CTRL_CER_EN_Pos              2                                       
#define CMSDK_FLASH_CTRL_CER_EN_Msk              (0x00000001ul << CMSDK_FLASH_CTRL_CER_EN_Pos)
//CMD = IDLE/INIT/START
#define CMSDK_FLASH_CTRL_CMD_Pos                 8
#define CMSDK_FLASH_CTRL_CMD_Msk                 (0x00000003ul << CMSDK_FLASH_CTRL_CMD_Pos)
                                       
#define CMSDK_FLASH_CTRL_CMD_INIT_Msk            (0x00000001ul << CMSDK_FLASH_CTRL_CMD_Pos)            

#define CMSDK_FLASH_CTRL_CMD_START_Pos           9                                       
#define CMSDK_FLASH_CTRL_CMD_START_Msk           (0x00000001ul << CMSDK_FLASH_CTRL_CMD_START_Pos)

#define CMSDK_FLASH_STATUS_PBUSY_Pos             0                                       
#define CMSDK_FLASH_STATUS_PBUSY_Msk             (0x00000001ul << CMSDK_FLASH_STATUS_PBUSY_Pos)

#define CMSDK_FLASH_STATUS_PDONE_Pos             1                                       
#define CMSDK_FLASH_STATUS_PDONE_Msk             (0x00000001ul << CMSDK_FLASH_STATUS_PDONE_Pos) 

#define CMSDK_FLASH_NVR_ACCESS_Pos               20                                       
#define CMSDK_FLASH_NVR_ACCESS_Msk               (0x00000001ul << CMSDK_FLASH_NVR_ACCESS_Pos)

#define CMSDK_FLASH_BADSECTOR_ADD_VADID_Pos      31                                       
#define CMSDK_FLASH_BADSECTOR_ADD_VADID_Msk      (0x00000001ul << CMSDK_FLASH_BADSECTOR_ADD_VADID_Pos)
/*@}*/ /* end of group CMSDK_FLASH */

/*------------------- Watchdog ----------------------------------------------*/
/** @addtogroup CMSDK_Watchdog CMSDK Watchdog
  @{
*/
typedef struct
{

  __IO    uint32_t  LOAD;          // <h> Watchdog Load Register </h>
  __I     uint32_t  VALUE;         // <h> Watchdog Value Register </h>
  __IO    uint32_t  CTRL;          // <h> Watchdog Control Register
                                   //   <o.1>    RESEN: Reset enable
                                   //   <o.0>    INTEN: Interrupt enable
                                   // </h>
  __O     uint32_t  INTCLR;        // <h> Watchdog Clear Interrupt Register </h>
  __I     uint32_t  RAWINTSTAT;    // <h> Watchdog Raw Interrupt Status Register </h>
  __I     uint32_t  MASKINTSTAT;   // <h> Watchdog Interrupt Status Register </h>
        uint32_t  RESERVED0[762];
  __IO    uint32_t  LOCK;          // <h> Watchdog Lock Register </h>
        uint32_t  RESERVED1[191];
  __IO    uint32_t  ITCR;          // <h> Watchdog Integration Test Control Register </h>
  __O     uint32_t  ITOP;          // <h> Watchdog Integration Test Output Set Register </h>

}CMSDK_WATCHDOG_TypeDef;

#define CMSDK_Watchdog_LOAD_Pos               0                                              /*!< CMSDK_Watchdog LOAD: LOAD Position */
#define CMSDK_Watchdog_LOAD_Msk              (0xFFFFFFFFul << CMSDK_Watchdog_LOAD_Pos)       /*!< CMSDK_Watchdog LOAD: LOAD Mask */

#define CMSDK_Watchdog_VALUE_Pos              0                                              /*!< CMSDK_Watchdog VALUE: VALUE Position */
#define CMSDK_Watchdog_VALUE_Msk             (0xFFFFFFFFul << CMSDK_Watchdog_VALUE_Pos)      /*!< CMSDK_Watchdog VALUE: VALUE Mask */

#define CMSDK_Watchdog_CTRL_RESEN_Pos         1                                              /*!< CMSDK_Watchdog CTRL_RESEN: Enable Reset Output Position */
#define CMSDK_Watchdog_CTRL_RESEN_Msk        (0x1ul << CMSDK_Watchdog_CTRL_RESEN_Pos)        /*!< CMSDK_Watchdog CTRL_RESEN: Enable Reset Output Mask */

#define CMSDK_Watchdog_CTRL_INTEN_Pos         0                                              /*!< CMSDK_Watchdog CTRL_INTEN: Int Enable Position */
#define CMSDK_Watchdog_CTRL_INTEN_Msk        (0x1ul << CMSDK_Watchdog_CTRL_INTEN_Pos)        /*!< CMSDK_Watchdog CTRL_INTEN: Int Enable Mask */

#define CMSDK_Watchdog_INTCLR_Pos             0                                              /*!< CMSDK_Watchdog INTCLR: Int Clear Position */
#define CMSDK_Watchdog_INTCLR_Msk            (0x1ul << CMSDK_Watchdog_INTCLR_Pos)            /*!< CMSDK_Watchdog INTCLR: Int Clear Mask */

#define CMSDK_Watchdog_RAWINTSTAT_Pos         0                                              /*!< CMSDK_Watchdog RAWINTSTAT: Raw Int Status Position */
#define CMSDK_Watchdog_RAWINTSTAT_Msk        (0x1ul << CMSDK_Watchdog_RAWINTSTAT_Pos)        /*!< CMSDK_Watchdog RAWINTSTAT: Raw Int Status Mask */

#define CMSDK_Watchdog_MASKINTSTAT_Pos        0                                              /*!< CMSDK_Watchdog MASKINTSTAT: Mask Int Status Position */
#define CMSDK_Watchdog_MASKINTSTAT_Msk       (0x1ul << CMSDK_Watchdog_MASKINTSTAT_Pos)       /*!< CMSDK_Watchdog MASKINTSTAT: Mask Int Status Mask */

#define CMSDK_Watchdog_LOCK_Pos               0                                              /*!< CMSDK_Watchdog LOCK: LOCK Position */
#define CMSDK_Watchdog_LOCK_Msk              (0x1ul << CMSDK_Watchdog_LOCK_Pos)              /*!< CMSDK_Watchdog LOCK: LOCK Mask */

#define CMSDK_Watchdog_INTEGTESTEN_Pos        0                                              /*!< CMSDK_Watchdog INTEGTESTEN: Integration Test Enable Position */
#define CMSDK_Watchdog_INTEGTESTEN_Msk       (0x1ul << CMSDK_Watchdog_INTEGTESTEN_Pos)       /*!< CMSDK_Watchdog INTEGTESTEN: Integration Test Enable Mask */

#define CMSDK_Watchdog_INTEGTESTOUTSET_Pos    1                                              /*!< CMSDK_Watchdog INTEGTESTOUTSET: Integration Test Output Set Position */
#define CMSDK_Watchdog_INTEGTESTOUTSET_Msk   (0x1ul << CMSDK_Watchdog_INTEGTESTOUTSET_Pos)   /*!< CMSDK_Watchdog INTEGTESTOUTSET: Integration Test Output Set Mask */

/*@}*/ /* end of group  CMSDK_Watchdog */



/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup CMSDK_MemoryMap CMSDK Memory Mapping
  @{
*/

/* Peripheral and SRAM base address */
#define CMSDK_FLASH_BASE        (0x00000000UL)  /*!< (FLASH     ) Base Address */
#define CMSDK_PFLASH_BASE       (0x10000000UL)  /*!< (Prgm Flash) Base Address */
#define CMSDK_SRAM_BASE         (0x20000000UL)  /*!< (SRAM      ) Base Address */
#define CMSDK_DFLASH_BASE       (0x30000000UL)  /*!< (Data Flash) Base Address */

#define CMSDK_RAM_BASE          (0x20000000UL)
#define CMSDK_AHB_BASE          (0x40010000UL)
#define CMSDK_APB_BASE          (0x40000000UL)

/* APB peripherals       */
#define CMSDK_TIMER0_BASE       (CMSDK_APB_BASE + 0x0000UL)
#define CMSDK_TIMER1_BASE       (CMSDK_APB_BASE + 0x1000UL)
#define CMSDK_TIMER2_BASE       (CMSDK_APB_BASE + 0x2000UL)
#define CMSDK_TIMER3_BASE       (CMSDK_APB_BASE + 0x3000UL)
#define CMSDK_UART0_BASE        (CMSDK_APB_BASE + 0x4000UL)
#define CMSDK_WATCHDOG_BASE     (CMSDK_APB_BASE + 0x8000UL)
#define CMSDK_TESTSLAVE_BASE    (CMSDK_APB_BASE + 0xB000UL)
#define CMSDK_ZMEAS_BASE        (CMSDK_APB_BASE + 0xC000UL)
#define CMSDK_IMEAS_BASE        (CMSDK_APB_BASE + 0xD000UL)
#define CMSDK_NFC_BASE          (CMSDK_APB_BASE + 0xE000UL)

/* AHB peripherals      */
#define CMSDK_GPIO0_BASE        (CMSDK_AHB_BASE + 0x0000UL)
#define CMSDK_SYSCTRL_BASE      (CMSDK_AHB_BASE + 0xF000UL)
#define CMSDK_DFLASH0_BASE      (CMSDK_DFLASH_BASE + 0x10000UL)   /*!< (Data Flash Register) Base Address */
#define CMSDK_PFLASH0_BASE      (CMSDK_PFLASH_BASE + 0x10000UL)   /*!< (Prgm Flash Register) Base Address */

/*@}*/ /* end of group CMSDK_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup CMSDK_PeripheralDecl CMSDK Peripheral Declaration
  @{
*/

#define CMSDK_UART0             ((CMSDK_UART_TypeDef   *) CMSDK_UART0_BASE   )
#define CMSDK_TIMER0            ((CMSDK_TIMER_TypeDef  *) CMSDK_TIMER0_BASE  )
#define CMSDK_TIMER1            ((CMSDK_TIMER_TypeDef  *) CMSDK_TIMER1_BASE  )
#define CMSDK_TIMER2            ((CMSDK_TIMER_TypeDef  *) CMSDK_TIMER2_BASE  )
#define CMSDK_TIMER3            ((CMSDK_TIMER_TypeDef  *) CMSDK_TIMER3_BASE  )
#define CMSDK_WATCHDOG          ((CMSDK_WATCHDOG_TypeDef  *) CMSDK_WATCHDOG_BASE   )
#define CMSDK_ZMEAS0            ((CMSDK_ZMEAS_TypeDef  *) CMSDK_ZMEAS_BASE   )
#define CMSDK_IMEAS0            ((CMSDK_IMEAS_TypeDef  *) CMSDK_IMEAS_BASE   )
#define CMSDK_GPIO0             ((CMSDK_GPIO_TypeDef   *) CMSDK_GPIO0_BASE   )
#define CMSDK_SYSCON0           ((CMSDK_SYSCON_TypeDef *) CMSDK_SYSCTRL_BASE )
#define CMSDK_DFLASH0           ((CMSDK_FLASH_TypeDef  *) CMSDK_DFLASH0_BASE )
#define CMSDK_PFLASH0           ((CMSDK_FLASH_TypeDef  *) CMSDK_PFLASH0_BASE )

/*@}*/ /* end of group CMSDK_PeripheralDecl */

/*@}*/ /* end of group CMSDK_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* CMSDK_H */
