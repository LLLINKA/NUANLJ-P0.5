/**
 * @file api_isohfllhw.c
 *
 * @brief This file contains the Low Level Hardware functions
 * used to safely drive the HF subsystem (ISO14443)
 *
 * @author (INVIA)
 *
 */

#include "api_isohfllhw.h"

 /**
 * Disable the HF Fc Clock
 *
 * This LLHW switches off the Fc clock extractor
 *
 */
void _LLHW_isohf_disableFc(HFCTRL isohf)
{
   _isohf_setTestCtrl(isohf, _isohf_getTestCtrl(isohf) | HF_TEST_CLK_EXTR_DIS);
}
/**
 * Enables the HF Fc Clock
 *
 * This LLHW renables the HF Fc clock extractor
 *
 */
void _LLHW_isohf_enableFc(HFCTRL isohf)
{
   _isohf_setTestCtrl(isohf,_isohf_getTestCtrl(isohf) & ~HF_TEST_CLK_EXTR_DIS);
}

/**
 * Wait until the SW reaches the Exec period
 *  
 */
void _LLHW_isohf_waitUntilExec(HFCTRL isohf)
{
   while (_isohf_getComStatus(isohf) != HF_STATUS_COM_EXEC) {}  
}

/**
 * Wait until the SW reaches the Tx period
 *  
 */
void _LLHW_isohf_waitUntilTx(HFCTRL isohf)
{
   while (_isohf_getComStatus(isohf) != HF_STATUS_COM_TX) {}
}

/**
 * Wait until the SW reaches the Rx period
 *  
 */
void _LLHW_isohf_waitUntilRx(HFCTRL isohf)
{
   while (_isohf_getComStatus(isohf) != HF_STATUS_COM_RX) {}
}

/**
 * Wait until the SW has the hand on HF digital controller 
 *
 */
void _LLHW_isohf_waitUntilPlatformHand(HFCTRL isohf)
{
   while (_isohf_getPlatformHandStatus(isohf) != 0x0) {}
}

/**
 * Skip anticollision ISOA Layer3
 *
 * This LLHW allows to directly jump in ISOA Layer 4
 * Shall be launch before HW FSM triggering with WaitRx
 *
 */
void _LLHW_isohf_configSkipISOALayer3(HFCTRL isohf)
{
	_isohf_setTestCtrl(isohf,HF_TEST_SKIP_ANTICOL);
}

/**
 * Configures and controls the HF Subsystem for catching next Rx frame
 *
 * This LLHW configures the control register, sets the WAIT RX control
 * 
 * Caution: The IO RAM is assumed ready to be used by the HF digital controller
 * 
 * @param[in] back_to_halt   Allows to return in ISO Type A layer 3 and wait for new Rx frame
 *                           use HF_P_CTRL_BACK2HALT when calling the LLHW or set to 0
 */
void _LLHW_isohf_waitForRx(HFCTRL isohf,uint32_t back_to_halt)
{
  _isohf_setProtocolCtrl(isohf, (back_to_halt | HF_P_CTRL_WAIT_RX));

}

/**
 * Configures and controls the HF Subsystem for launching the next Tx frame
 *
 * This LLHW configures the control register, sets the LAUNCH TX control
 * 
 * Caution: The IO RAM is assumed ready to be used by the HF digital controller
 * 
 * @param[in] back_to_halt        Allows to return in ISO Type A layer 3 and wait for new Rx frame
 *                                use HF_P_CTRL_BACK2HALT when calling the LLHW or set to 0
 * @param[in] silent_time         Defines the number of slots to be waited before Tx frame sending
 *                                0 to 15 - no check on the value
 * @param[in] tx_frame_size       Defines the Tx frame size in RAM buffer
 *                                1 to 0x400 - no check on the value
 * @param[in] end_of_transaction  Removes the Tx to Rx automatic reversal
 *                                use HF_P_CTRL_ENDOFTRANSAC when calling the driver or set to 0
 */
void _LLHW_isohf_launchTx(HFCTRL isohf,uint32_t back_to_halt, uint32_t silent_time,uint32_t tx_frame_size,uint32_t end_of_transaction)
{
  _isohf_setProtocolCtrl(isohf,((back_to_halt | HF_P_CTRL_LAUNCH_TX) | (silent_time << HF_P_CTRL_SILENT_TIME_SHIFT) | (tx_frame_size << HF_P_CTRL_TX_FRAME_SIZE_SHIFT) | end_of_transaction));

}

/**
 * Get the Silent Time Ts depending on current slot timer read value
 *
 * To be done before Tx launch
 *  
 * @param[in] min_n_val     the LLHW will set the silent time such as to be reached the min_n_val in any case.
 *                          Shall be set to 8 or 9
 * 
 * @return the LLHW returns the silent time to be used
 *  
 */
uint32_t _LLHW_isohf_getSilentTime(HFCTRL isohf,  uint32_t min_n_val)
{
  uint32_t silent_time, current_slot, slot,min_slot_val;
  min_slot_val=min_n_val-2;
  
  current_slot=_isohf_getSlotCounterStatus(isohf);

  if (0<current_slot<min_slot_val) {
       // Include a waiting time until min_slot_val
       silent_time  = min_slot_val-current_slot-1;
  } else {
  if (current_slot==min_slot_val) {
       silent_time=1;
  } else { // current_slot > min_slot_val
	   slot = current_slot-min_slot_val;
	   if (slot<15) {silent_time = slot;}
	   else {silent_time = 15;}
   }
  }

  return silent_time;    
}

/**
 * Front-end digital selection for Type A - Tx
 *
 * This LLHW selects the :
 * - OOK or BPSK modulation (Tx)
 * - bit rates
 *
 * It keeps the last Tx configuration.
 * 
 * @param[in] tx_bit_rate    Bit rate from 106 to 424
 *                           use DSI
 *                             0 = 106 kbits/s
 *                             1 = 212 kbits/s
 *                             2 = 424 kbits/s
  *                            3 = 848 kbits/s
 *
 */
void _LLHW_isohf_configTxDig4TypeA(HFCTRL isohf, uint32_t tx_bit_rate)
{

    uint32_t  tx_rate;
    uint32_t  tx_code_type;


    switch (tx_bit_rate) // = dsi
    {
    case 0:
        tx_rate      = (HF_DIG_CFG_BIT_RATE_106K);
        tx_code_type = (HF_DIG_CFG_COD_TYPE_OOK);
        break;

    case 1:
        tx_rate      = (HF_DIG_CFG_BIT_RATE_212K);
        tx_code_type = (HF_DIG_CFG_COD_TYPE_BPSK);
        break;

    case 2:
        tx_rate      = (HF_DIG_CFG_BIT_RATE_424K);
        tx_code_type = (HF_DIG_CFG_COD_TYPE_BPSK);
        break;

    case 3:
        tx_rate      = (HF_DIG_CFG_BIT_RATE_848K);
        tx_code_type = (HF_DIG_CFG_COD_TYPE_BPSK);
        break;
    default:
        break;
    }

    _isohf_setDigitalCfg(isohf, (_isohf_getDigitalCfg(isohf) & ~(HF_DIG_CFG_BIT_RATE_MASK  | HF_DIG_CFG_COD_TYPE_MASK )) |
           ((tx_rate      ) |
            (tx_code_type ) )
           );

}

/**
 * Front-end digital configuration for Type A - Rx
 *
 * This LLHW configure the threshold of the $RX$ decoder.
 *
 * @param[in] rx_bit_rate    Bit rate from 106 to 424
 *                           use DRI
 *                             0 = 106 kbits/s
 *                             1 = 212 kbits/s
 *                             2 = 424 kbits/s
 *                             3 = 848 kbits/s

 *
 */
void _LLHW_isohf_configRxDig4TypeA(HFCTRL isohf,uint32_t rx_bit_rate)
{
    
    switch (rx_bit_rate) // = dsi
    {
    case 0:
        _isohf_setDigitalCntCfg(isohf, 0, 0x18);
        _isohf_setDigitalCntCfg(isohf, 1, 0x78);
        _isohf_setDigitalCntCfg(isohf, 2, 0xB8);
        _isohf_setDigitalCntCfg(isohf, 3, 0xF8);
        _isohf_setReboundFilterCfg(isohf,0xA);
        _isohf_setAnalog1Cfg(isohf, 0x00010011);
        _isohf_setAnalog2Cfg(isohf, 0x00000141);
        break;

    case 1:
        _isohf_setDigitalCntCfg(isohf, 0, 0x0C);
        _isohf_setDigitalCntCfg(isohf, 1, 0x3B);
        _isohf_setDigitalCntCfg(isohf, 2, 0x5D);
        _isohf_setDigitalCntCfg(isohf, 3, 0x77);
        _isohf_setReboundFilterCfg(isohf,0x5);
        _isohf_setAnalog1Cfg(isohf, 0x00010019);
        _isohf_setAnalog2Cfg(isohf, 0x00000142);
        break;

    case 2:
        _isohf_setDigitalCntCfg(isohf, 0, 0x07);
        _isohf_setDigitalCntCfg(isohf, 1, 0x20);
        _isohf_setDigitalCntCfg(isohf, 2, 0x30);
        _isohf_setDigitalCntCfg(isohf, 3, 0x40);
        _isohf_setReboundFilterCfg(isohf,0x3);
        _isohf_setAnalog1Cfg(isohf, 0x0001003D); 
        _isohf_setAnalog2Cfg(isohf, 0x00000142);
        break;

    case 3:
        _isohf_setDigitalCntCfg(isohf, 0, 0x01);
        _isohf_setDigitalCntCfg(isohf, 1, 0x10);
        _isohf_setDigitalCntCfg(isohf, 2, 0x17);
        _isohf_setDigitalCntCfg(isohf, 3, 0x2A);
        _isohf_setReboundFilterCfg(isohf,0x2);
        _isohf_setAnalog1Cfg(isohf, 0x00010033);
        _isohf_setAnalog2Cfg(isohf, 0x00000140);
        break;
    default:
        break;
    }

}

// --------------------------------------------------------------- //
//  IORAM config                                                   //
// --------------------------------------------------------------- //
void _LLHW_isohf_ioram_config(void) {

  uint32_t config_addr= PLATFORM_HF_BUFFER_ADDR + (HF_IO_RAM_EMPTY_OFFSET >> 2);

  ((uint8_t *)(config_addr))[0] =0x10;  // ATQA first byte
  ((uint8_t *)(config_addr))[1] =0x00;  // ATQA second byte
  ((uint8_t *)(config_addr))[2] =0x00;  // UID0
  ((uint8_t *)(config_addr))[3] =0x01;  // UID1
  ((uint8_t *)(config_addr))[4] =0x02;  // UID2
  ((uint8_t *)(config_addr))[5] =0x03;  // UID3
  ((uint8_t *)(config_addr))[6] =0x04;  // UID4
  ((uint8_t *)(config_addr))[7] =0x05;  // UID5
  ((uint8_t *)(config_addr))[8] =0x06;  // UID6
  ((uint8_t *)(config_addr))[9] =0x07;  // UID7
  ((uint8_t *)(config_addr))[10]=0x08;  // UID8
  ((uint8_t *)(config_addr))[11]=0x09;  // UID9
  ((uint8_t *)(config_addr))[12]=0x0F;  // SAK NOT COMP
  ((uint8_t *)(config_addr))[13]=0xF0;  // SAK OK  

}

// --------------------------------------------------------------- //
//  get data from IORAM                                            //
// --------------------------------------------------------------- //
uint8_t _isohf_getHFIORAMbyte_local(uint32_t offset)
{
  return ((uint8_t *)(HF_IO_RAM_START_ADD))[offset];
}

// --------------------------------------------------------------- //
//  set data to IORAM                                              //
// --------------------------------------------------------------- //
void _isohf_setHFIORAMbyte_local(uint32_t offset, uint8_t data)
{
   ((uint8_t *)(HF_IO_RAM_START_ADD))[offset]= data;
}

// --------------------------------------------------------------- //
//  compare command received in IORAM                              //
// --------------------------------------------------------------- //
uint32_t _LLHW_isohf_compareIORAM2Mem_local(HFCTRL isohf,  uint8_t *pComp, uint32_t byte_size)
{
  uint32_t i;
  
  for (i=0; i<byte_size; i++) {
    if (pComp[i] !=_isohf_getHFIORAMbyte_local(i)) {return -1;}
  }
  return 0;    
}

// --------------------------------------------------------------- //
//  copy answer to IORAM                                           //
// --------------------------------------------------------------- //
void _LLHW_isohf_copyMem2IORAM_local(HFCTRL isohf, uint8_t *pSource, uint32_t offset, uint32_t bytesLength)
{
  uint32_t i;
  
  for (i=0; i<bytesLength; i++) {
      _isohf_setHFIORAMbyte_local(offset+i, (uint8_t)pSource[i]);
  }
}

// --------------------------------------------------------------- //
//  Check RX error status and mute                                 //
// --------------------------------------------------------------- //
uint32_t _isohf_checkRxStatusFailAndMute_local(HFCTRL isohf, uint32_t back_to_halt)
{
   if (_isohf_getRxErrorStatus(isohf) != 0x0) {
       _LLHW_isohf_waitForRx(isohf,back_to_halt);
       return 0;
   } else {
       return -1;
   }
}

// -------------------------------------------------------------------------------------------------------------- //
//  Configure for TypeA Layer 3 [(Rx/Tx 106 kbits/s), Fill RAM, disable TYPEA_L4_ENA] and launchtx to HALT state  //
// -------------------------------------------------------------------------------------------------------------- //
void _isohf_configTypeALayer3AndLaunch_local(HFCTRL isohf, uint8_t *pSource, uint32_t silent_time, uint32_t tx_frame_size)
{
     // Config for Tx : 106 kbit/s
     _LLHW_isohf_configTxDig4TypeA(isohf, 0);

     // Config for Rx : 106 kbit/s
     _LLHW_isohf_configRxDig4TypeA(isohf, 0);

     // Fill the RAM for Layer 3
     _LLHW_isohf_ioram_config();

     // Lock the RAM to 64 bytes
     _isohf_resetProtocolTypeALayer4(isohf);

     // Launch
    _LLHW_isohf_copyMem2IORAM_local(isohf, pSource, 0x0, tx_frame_size);
    _LLHW_isohf_launchTx(isohf, HF_P_CTRL_BACK2HALT, silent_time, tx_frame_size, 0x0);
}

//-----------------------------------------------------------------//
// Initializes ATS fields
// --------------------------------------------------------------- //
void isohf_ATSInitialize(ATS_iso14443_Block *ATS_Block)
{
  uint8_t ATS_length_byte = 0x02;//TL+T0
  uint8_t ATS_format_byte = FSCI;
  uint8_t ATS_interface_byteA = ((D_DIRECTION << 7) | (DS_848_kbps << 6) | (DS_424_kbps << 5) | (DS_212_kbps << 4) | (DR_848_kbps << 2) | (DR_424_kbps << 1) | (DR_212_kbps << 0));
  uint8_t ATS_interface_byteB = ((FWI << 4) | SFGI);
  uint8_t ATS_interface_byteC = ((CID_COMPLIANT << 1) | NAD_COMPLIANT);

  #ifdef TA_TRANSMIT
    ATS_format_byte |= (TA_TRANSMIT << 4);
    ATS_Block->TA = ATS_interface_byteA;//set TA
    ATS_length_byte ++;
  #endif
  #ifdef TB_TRANSMIT
    ATS_format_byte |= (TB_TRANSMIT << 5);
    ATS_Block->TB = ATS_interface_byteB;//set TB
    ATS_length_byte ++;
  #endif
  #ifdef TC_TRANSMIT
    ATS_format_byte |= (TC_TRANSMIT << 6);
    ATS_Block->TC = ATS_interface_byteC;//set TC
    ATS_length_byte ++;
  #endif

  ATS_Block->TL = ATS_length_byte;//set TL
  ATS_Block->T0 = ATS_format_byte;//set T0
}
// --------------------------------------------------------------- //
//  Layer3 End Ctrl (waiting for RATS)                             //
// --------------------------------------------------------------- //
uint32_t _isohf_ctrlISOAEndOfLayer3_local(HFCTRL isohf,uint32_t silent_time, uint8_t *pFSDI_CID, ATS_iso14443_Block *ATS_Block)
{

  uint32_t frame_size, i;  
  uint8_t halt[2];
  halt[0]=0x50;
  halt[1]=0x00;  
  uint8_t rats=0xE0;
  uint8_t cid;  // 0-14
  uint8_t fsdi; // 0-12
  uint8_t byte;  
  
  if (_isohf_checkRxStatusFailAndMute_local(isohf,HF_P_CTRL_BACK2HALT) == 0) { // Error mute
   } else {
     frame_size=_isohf_getRxFrameSize(isohf); // check frame-size >2  
     if (frame_size < 4) { // CRC is included
         _LLHW_isohf_waitForRx(isohf,HF_P_CTRL_BACK2HALT); // Error mute
     } else {         
          // Check HALT - 50 00
          //--------------------------          
          if ((_LLHW_isohf_compareIORAM2Mem_local(isohf,&halt[0],2)==0) && (frame_size == 4)){
               _isohf_setProtocolIgnoreReqA(isohf);
             _LLHW_isohf_waitForRx(isohf,HF_P_CTRL_BACK2HALT); // mute
          }
              else {
                 // Check RATS (full frame given in parameter)
                 //--------------------------
                  if ((_LLHW_isohf_compareIORAM2Mem_local(isohf,&rats,1)==0) && (frame_size == 4)) {
                      // Check FDSI and CID
                      byte=_isohf_getHFIORAMbyte_local(1);
                      cid = byte & 0x0F;
                      fsdi= (byte & 0xF0) >> 4;
                       if ((cid<15) && (fsdi<13)) {
                          *pFSDI_CID=byte;
                          _isohf_setProtocolIgnoreReqA(isohf);
                          _isohf_setProtocolTypeALayer4(isohf);
                          for(int i = 0; i < ATS_Block->TL; i++) {// copy ATS to IORAM
                             ((uint8_t *)(PLATFORM_HF_BUFFER_ADDR))[i] = ((uint8_t *)(ATS_Block))[i];
                          }
                          _LLHW_isohf_launchTx(isohf, 0,silent_time,ATS_Block->TL,0x0);

                          return 1;
                       } else {// Other frame -- not compliant with Layer ISO14443-4
                          _LLHW_isohf_waitForRx(isohf,HF_P_CTRL_BACK2HALT); // mute 
                       }
                  }
                  else {// Other frame -- not compliant with Layer ISO14443-4
                         _LLHW_isohf_waitForRx(isohf,HF_P_CTRL_BACK2HALT); // mute
                  }
        }
     }
   }
   return 0;
}

// --------------------------------------------------------------- //
//  PPS request Ctrl                                               //
// --------------------------------------------------------------- //
uint32_t _isohf_ctrlISOAPPS_local(HFCTRL isohf,uint32_t cid, uint32_t silent_time)
{
   
  uint32_t frame_size;
  uint32_t dri, dsi;
  uint8_t frame;
  uint32_t ppss;

  ppss= (uint32_t)(0xD0 | cid);

  if (_isohf_checkRxStatusFailAndMute_local(isohf,0x0) == 0) {// Error mute
   } else {
     frame_size=_isohf_getRxFrameSize(isohf); // check frame-size >2  
     if (frame_size < 4) {
         _LLHW_isohf_waitForRx(isohf,0x0); // Error mute
     } else {

      // Check PPSS and CID
      //-------------------------
      frame = (uint8_t)_isohf_getHFIORAMbyte_local(0);
      // Check PPSS
      if ((frame & 0xF0) == 0xD0 ) {
         // check CID
          if ((frame & 0x0F) == cid ) {
             // check PPS0
             //-------------------------------------
             if ((_isohf_getHFIORAMbyte_local(1) == 0x01) && (frame_size == 4)){     // no PPS1
                frame=(uint8_t)ppss;
                _LLHW_isohf_copyMem2IORAM_local(isohf, &frame, 0x0, 1);    // launch PPSS with no frequency change
                _LLHW_isohf_launchTx(isohf, 0,silent_time,1,0x0);
                 return 1;
             } else {
                 if ((_isohf_getHFIORAMbyte_local(1) == 0x11) && (frame_size == 5)){   // PPS1 expected
                   frame=_isohf_getHFIORAMbyte_local(2);
                   if ((frame & 0xF0) == 0x0) {
                         
                       // get DRI and DSI from inputs
                       dri=(uint32_t)(frame & 0x3); // not used, kept to 0, else, change the function using _LLHW_isohf_configRxDig4TypeA
                       dsi=(uint32_t)((frame & 0xC) >> 2);
                       if ((dri >3) || (dsi >3)) {
                          _LLHW_isohf_waitForRx(isohf,0x0); // Error mute if dri/dsi selection not compliant with HW capability
                       } else {
                          // Change frequency depending DSI
                          _LLHW_isohf_configRxDig4TypeA(isohf, dri);
                          _LLHW_isohf_configTxDig4TypeA(isohf, dsi);
                          // Answer PPS -- frequency change at the end of the Tx end of frame
                          frame=(uint8_t)ppss;       
                          _LLHW_isohf_copyMem2IORAM_local(isohf, &frame, 0x0, 1);
                          _LLHW_isohf_launchTx(isohf, 0,silent_time,1,0x0);
                       return 1;   
                       }
                     }
                     else {// Error on PPS1 mute - ignore the frame
                       _LLHW_isohf_waitForRx(isohf,0x0);
                     } 
                 }
                 else {//  Error on PPS0 mute - ignore the frame
                       _LLHW_isohf_waitForRx(isohf,0x0);
                 }
             }
        }
        else {//  Error on CID mute - ignore the frame
              _LLHW_isohf_waitForRx(isohf,0x0);
        }
      }
      else {return -1;} // Other frame
    }
  }
  return 0;   
}

uint32_t _isohf_ctrlISOADESELECT_local(HFCTRL isohf, iso14443_Block *isohf_NFC, uint8_t cid, uint32_t frame_size, uint32_t silent_time)
{
  // Improvement/check to be done ouside this function:
  //-----------------------------------------------
  // 1) CID : can be sent by the PCD even if not supported by the PICC - no check versus TC(1)
  // (refer to Interface byte TC(1) sent by the PICC when ATS)
  
  // note : power level indication is set to (00)b

    uint8_t deselect_cid,deselect_nocid;
    uint8_t tx_frame[2];

    deselect_cid  = 0xCA;  // frame size 2 (out of CRC)
    deselect_nocid= 0xC2;  // frame size 1 (out of CRC)
         
       // Check deselect - PCB (S-block)
       //-------------------------
       if (isohf_NFC->PCB == deselect_cid) { // CID expected
          // check CID
             //-------------------------------------
          if ((isohf_NFC->CID == cid) && (frame_size==4)){ // CID with power level indication 00

                // Answer DESELECT ACK -- frequency change at the end of the Tx end of frame -- return in Layer 3 -- 106/106
                 // build the Tx frame
                  tx_frame[0]=(uint8_t)deselect_cid;  //PCB
                  tx_frame[1]=(uint8_t)cid;           //CID
                  _isohf_configTypeALayer3AndLaunch_local(isohf, &tx_frame[0], silent_time, 2);                  
                  return 1;
          } else {
                _LLHW_isohf_waitForRx(isohf,0x0); // Error mute - ignore the frame
          }
       } else { // NO CID expected
                if (frame_size==3) {                                                        
                   // Answer DESELECT ACK -- frequency change at the end of the Tx end of frame -- return in Layer 3
                   // build the Tx frame
                   tx_frame[0]=(uint8_t)deselect_nocid;//PCB
                   _isohf_configTypeALayer3AndLaunch_local(isohf, &tx_frame[0], silent_time, 1);                   
                   return 1;
                } else {
                       _LLHW_isohf_waitForRx(isohf,0x0); // Error mute - ignore the frame                   
                }
      }
  
  return 0;
}            

//-----------------------------------------------------------------//
//Get I_Block_PCB
// --------------------------------------------------------------- //
uint32_t isohf_GetIblock(iso14443_Block *isohf_NFC, uint8_t *i_block, uint8_t *chaining_bit_en, uint8_t *cid_exist, uint8_t *NAD_exist, uint8_t *block_num, uint8_t *error)
{
   *i_block    	     =  (isohf_NFC-> PCB & IRS_BLOCK_MASK) >> IRS_BLOCK_SHIFT;
   *chaining_bit_en   =  (isohf_NFC-> PCB  & I_CHANINIG_MASK) >>  I_CHANINIG_SHIFT;
   *cid_exist         =  (isohf_NFC-> PCB  & CID_MASK) >> CID_SHIFT;
   *NAD_exist         =  (isohf_NFC-> PCB  & I_NAD_MASK) >> I_NAD_SHIFT;
   *block_num         =  (isohf_NFC-> PCB  & BLOCK_NUMBER_MASK) >> BLOCK_NUMBER_SHIFT;
   if((((isohf_NFC-> PCB) & (1 << 1)) == 0x02) && (((isohf_NFC-> PCB) & (1 << 5)) == 0x0)) {
      *error = 0x0;
   }
   else {
      *error = 0x1;
   }

  return 0;
}

//-----------------------------------------------------------------//
//Get R_Block_PCB
// --------------------------------------------------------------- //
uint32_t isohf_GetRblock(iso14443_Block *isohf_NFC, uint8_t *r_block, uint8_t *ACK, uint8_t *cid_exist, uint8_t *block_num, uint8_t *error)
{
   *r_block    	     =  (isohf_NFC-> PCB & IRS_BLOCK_MASK) >> IRS_BLOCK_SHIFT;
   *ACK               =  (isohf_NFC-> PCB & R_ACK_OR_NAK_MASK) >> R_ACK_OR_NAK_SHIFT;
   *cid_exist         =  (isohf_NFC-> PCB  & CID_MASK) >> CID_SHIFT;
   *block_num         =  (isohf_NFC-> PCB  & BLOCK_NUMBER_MASK) >> BLOCK_NUMBER_SHIFT;
   if((((isohf_NFC-> PCB) & (1 << 1)) == 0x02) && (((isohf_NFC-> PCB) & (1 << 2)) == 0x0) && (((isohf_NFC-> PCB) & (1 << 5)) == 0x20)) {
      *error = 0x0;
   }
   else {
      *error = 0x1;
   }

  return 0;
}

//-----------------------------------------------------------------//
//Get S_Block_PCB
// --------------------------------------------------------------- //
uint32_t isohf_GetSblock(iso14443_Block *isohf_NFC, uint8_t *s_block, uint8_t *FUNC_SEL, uint8_t *FUNC, uint8_t *cid_exist, uint8_t *error)
{
   *s_block    	      =  (isohf_NFC-> PCB & IRS_BLOCK_MASK) >> IRS_BLOCK_SHIFT;
   *FUNC_SEL          =  (isohf_NFC-> PCB & S_FUNC_SEL_MASK) >> S_FUNC_SEL_SHIFT;
   *FUNC              =  (isohf_NFC-> PCB & S_FUNC_MASK) >> S_FUNC_SHIFT;
   *cid_exist         =  (isohf_NFC-> PCB  & CID_MASK) >> CID_SHIFT;
   if((((isohf_NFC-> PCB) & (0x01)) == 0x0) && (((isohf_NFC-> PCB) & (1 << 2)) == 0x0)) {
      *error = 0x0;
   }
   else {
      *error = 0x1;
   }

  return 0;
}

//-----------------------------------------------------------------//
//Set I_Block_PCB                                                  //
// --------------------------------------------------------------- //
uint32_t isohf_SetIblock(iso14443_Block *isohf_NFC, uint8_t chaining_bit_en, uint8_t cid_exist, uint8_t NAD_exist, uint8_t cid, uint8_t block_num)
{
                            isohf_NFC-> PCB = 0;
   if(block_num == 1 )      isohf_NFC-> PCB |= BLOCK_NUMBER_MASK; 
                            isohf_NFC-> PCB |= 1 << 1;
   if(NAD_exist == 1 )      isohf_NFC-> PCB |= I_NAD_MASK;
   if(cid_exist == 1 )      isohf_NFC-> PCB |= CID_MASK;
   if(chaining_bit_en == 1 )isohf_NFC-> PCB |= I_CHANINIG_MASK;
                            isohf_NFC-> PCB |= I_BLOCK_MODE << IRS_BLOCK_SHIFT ;
  // Copy I block(PCB/CID) to IORAM BUFFER
   ((uint8_t *)(PLATFORM_HF_BUFFER_ADDR))[0]   = IRS_BLOCK->PCB;  //PCB
   if(cid_exist) {
        ((uint8_t *)(PLATFORM_HF_BUFFER_ADDR))[1]   = cid;  //CID
   }

  return 0;  
}

//-----------------------------------------------------------------//
//Set R_Block_PCB                                                  //
// --------------------------------------------------------------- //
uint32_t isohf_SetRblock(iso14443_Block *isohf_NFC, uint8_t ACK, uint8_t cid_exist, uint8_t cid, uint8_t block_num)
{
                       isohf_NFC-> PCB = 0;	  	 
  if(block_num == 1 )  isohf_NFC-> PCB |= BLOCK_NUMBER_MASK;      	         
                       isohf_NFC-> PCB |= 1 << 1;                               	        
  if(cid_exist == 1 )  isohf_NFC-> PCB |= CID_MASK;                             	        
  if(ACK == 1 )        isohf_NFC-> PCB |= R_ACK_OR_NAK_MASK ;  	  // send pos/neg Acknowledgment            
                       isohf_NFC-> PCB |= 1 << 5;
                       isohf_NFC-> PCB |= R_BLOCK_MODE << IRS_BLOCK_SHIFT ;

 // Copy R block to IORAM BUFFER
  ((uint8_t *)(PLATFORM_HF_BUFFER_ADDR))[0]   = IRS_BLOCK->PCB;  //PCB
  if(cid_exist) {
       ((uint8_t *)(PLATFORM_HF_BUFFER_ADDR))[1]   = cid;  //CID
  }
  
  return 0;  
}

//-----------------------------------------------------------------//
//Get FSD/FSC Value depending upon FSDI/FSCI
// --------------------------------------------------------------- //
uint16_t isohf_GetFS_Value(uint8_t FSI)
{
   uint16_t   FS;  
 	switch (FSI) {
            case 0:
                FS= 0x10;
                break;
            case 1:
                FS= 0x18;
                break;
            case 2:
                FS= 0x20;
                break;
            case 3:
                FS= 0x28;
                break;
            case 4:
                FS= 0x30;
                break;
            case 5:
                FS= 0x40;
                break;
             case 6:
                FS= 0x60;
                break;
             case 7:
                FS= 0x80;
                break;
             case 8:
                FS= 0x100;
                break;
             case 9:
                FS= 0x200;
                break;
             case 10:
                FS= 0x400;
                break;
             case 11:
                FS= 0x800;
                break;
             case 12:
                FS= 0x1000;
                break;
            default:
                FS= 0x1000;
                break;
        }
        return FS;	
}
