/*
 *-----------------------------------------------------------------------------
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm Limited or its affiliates.
 *
 *            (C) COPYRIGHT 2010-2013 Arm Limited or its affiliates.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Arm Limited or its affiliates.
 *
 *      SVN Information
 *
 *      Checked In          : $Date: $
 *
 *      Revision            : $Revision: $
 *
 *      Release Information : Cortex-M System Design Kit-r1p1-00rel0
 *-----------------------------------------------------------------------------
 */
/* Linker script to configure memory regions.
 * Need modifying for a specific board.
 *   FLASH.ORIGIN: starting address of boot loader
 *   FLASH.LENGTH: length of flash
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 */

INCLUDE "lib-nosys.ld"

MEMORY
{
  FLASH (rx) : ORIGIN = 0x30000000, LENGTH =  0x2800 /* 8KB of Data Flash */ /** increased to 10KB for testing **/ 
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x01C00 /* 7KB (1KB reserved for NFC) /*******(remap to SRAM--> RAM (rwx)  : ORIGIN = 0x20001000, LENGTH = 0x00C00)*/
}

INCLUDE "sections.ld"
