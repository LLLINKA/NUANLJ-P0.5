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

#include "mcu_debugtester_interface.h" // For definition of interface to the debug tester

void EnableDebugTester(void)
{
  uint32_t i;


  //
  // Initialise GPIO from MCU to Debug Tester
  //

  //Initialize the GPIO0 higher 8 bits output value, set to 0
  CMSDK_GPIO->DATAOUT =  (0x00000000);    //set GPIO output to 0 for default value, disable the function strobe
  CMSDK_GPIO->IE = 0x3<<14; //DEBUG_CMD; //0x3<<14;

  //CMSDK_gpio_SetOutEnable (CMSDK_GPIO0, DEBUG_CMD);    // set specified bit in out enable register
  CMSDK_GPIO->OE = DEBUG_CMD;

  //
  // Enable the Debug Tester in the testbench
  //

  // send command to enable the connection to debug tester
  UartPutc((char) DBG_ESCAPE); //send ESCAPE code
  UartPutc((char) DBG_CONNECT_ENABLE); //send debug test enable command
  puts("\nEnabling debug tester...\n");
  // If debug tester is not present,
  if((CMSDK_GPIO->DATAIN & DEBUG_ERROR) != 0)
    {
      puts("DBGERROR bit (debug error) asserted.\n");
      puts("Debug tester not available:\n");
      puts("1: The ARM_CMSDK_INCLUDE_DEBUG_TESTER macro is not defined, or\n");
      puts("2: Cortex-M0 DesignStart is used\n");
      puts("** TEST SKIPPED **\n");
      // End simulation
      UartEndSimulation();
    }


  //
  // Initialise the Communication Region
  // (Zero the 4 words above Stack Top)
  //

  /*for(i=0; i<4 ; i++)
    {
       puts("Initialise the Communication Region Zero the 4 words above Stack Top\n");

      DEBUGTESTERDATA[i] = 0;
    }*/
}


void DisableDebugTester(void)
{
  //send command to disable the connection to debug tester
  UartPutc((char) DBG_ESCAPE); //send debug test disable command,
  UartPutc((char) DBG_CONNECT_DISABLE); //send debug test disable command,
  UartEndSimulation();  //stop simulation
}


//===========================================================================
// Start a function running on the debug tester
//===========================================================================
void StartDebugTester(uint32_t Function)
{
  puts("\nStart DebugTester...\n"); 
  // Write function onto function select pins
  //CMSDK_gpio_SetOutEnable (CMSDK_GPIO0, DEBUG_CMD);    // set specified bit in out enable register
    CMSDK_GPIO->OE = DEBUG_CMD;

  //setup function select and function strobe
  //CMSDK_gpio_MaskedWrite(CMSDK_GPIO0, Function<<DEBUG_BIT_LOC | DEBUG_STROBE, DEBUG_CMD|DEBUG_STROBE);
    //CMSDK_GPIO->ALTFL = DEBUG_CMD|DEBUG_STROBE;
    //CMSDK_GPIO->ALTFH = DEBUG_CMD|DEBUG_STROBE;
      CMSDK_GPIO->DATAOUT = (Function<<DEBUG_BIT_LOC| DEBUG_STROBE); //(Function<<DEBUG_BIT_LOC | DEBUG_STROBE, DEBUG_CMD|DEBUG_STROBE); //0x0021FF;

 //printf("%x - GPIODAT, %x-DBUGRUN\n", CMSDK_GPIO->DATAIN,DEBUG_RUNNING);
  // Wait to see Running asserted
  while((CMSDK_GPIO->DATAIN & DEBUG_RUNNING) == 0);

  // Clear strobe
  puts("\nClear DBUG STROBE...\n"); 
  //CMSDK_gpio_MaskedWrite(CMSDK_GPIO0, 0x0, DEBUG_STROBE);
     CMSDK_GPIO->DATAOUT =  Function<<8;
    //CMSDK_GPIO->ALTFH =  DEBUG_STROBE;

}


//===========================================================================
// Check that a debug tester function completed
// Return pass/fail accordingly
//===========================================================================
uint32_t CheckDebugTester(void)
{
  // Wait to see Running deasserted
  //printf("%x - GPIODAT, %x-DBUGRUN\n", CMSDK_GPIO->DATAIN,DEBUG_RUNNING);
  while((CMSDK_GPIO->DATAIN & DEBUG_RUNNING)!= 0);

  // Return status, check ERROR bit
  if((CMSDK_GPIO->DATAIN & DEBUG_ERROR) == 0)
    {
      return TEST_PASS;
    }
  else
    {
      return TEST_FAIL;
    }
}


//===========================================================================
// Execute a debug tester function
//===========================================================================
uint32_t CallDebugTester(uint32_t Function)
{
  StartDebugTester(Function);
  return CheckDebugTester();
}

