#include "parameters.h"
/***********************************************************************************
***********************************************************************************/
//Timers
#define  TIMER1_1sec_RLD_VAL                    0x0001
#define  TIMER1_1sec_CUR_VAL                    0x0001
#define  TIMER1_30sec_CUR_VAL               	0x001E            
#define  TIMER1_30sec_RLD_VAL               	0x001E           
#define  TIMER2_1min_CUR_VAL                	0x003C             
#define  TIMER2_1min_RLD_VAL                	0x003C             
#define  TIMER3_30min_CUR_VAL               	0x0708             
#define  TIMER3_30min_RLD_VAL               	0x0708  

/***********************************************************************************
***********************************************************************************/
//Number of days measure Glucose and Impedance
#define IMEAS_14DAYS_1MIN_COUNT                 0x4EC0  //20160   // 14Days * 1440times/day 
#define IMEAS_13DAYS_5MIN_COUNT                 0xEA0   //=3744   // 13Days * 288times/day
#define ZMEAS_14DAYS_30MIN_COUNT                0x2A0   //=672    // 14Days * 48times/day
/***********************************************************************************
***********************************************************************************/
//Clocks           
#define  HCLK_DIV_VAL               		HCLK_DIV_VAL_8MHZ
//#define  HCLK_DIV_VAL				HCLK_DIV_VAL_4MHZ
//#define  HCLK_DIV_VAL				HCLK_DIV_VAL_2MHZ
//#define  HCLK_DIV_VAL				HCLK_DIV_VAL_1MHZ

#define  PCLK_DIV_VAL               		PCLK_DIV_VAL_HCLK
//#define PCLK_DIV_VAL				PCLK_DIV_VAL_HCLK_BY_2
//#define PCLK_DIV_VAL				PCLK_DIV_VAL_HCLK_BY_4
//#define PCLK_DIV_VAL				PCLK_DIV_VAL_HCLK_BY_8

#define  ICLK_DIV_VAL 			        ICLK_DIV_VAL_128KHZ
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_64KHZ 
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_32KHZ 
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_16KHZ 
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_8KHZ  
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_4KHZ  
//#define  ICLK_DIV_VAL 			ICLK_DIV_VAL_2KHZ  
//#define  ICLK_DIV_VAL               		ICLK_DIV_VAL_1KHZ
/***********************************************************************************
***********************************************************************************/
//IMEAS
#define  IMEAS_CHA_NUM            		IMEAS_CHA_NUM_0 //(W1)
//#define  IMEAS_CHA_NUM            		IMEAS_CHA_NUM_1 //(W2)
//#define  IMEAS_CHA_NUM            		IMEAS_CHA_NUM_2 //(CE)

#define  IMEAS_CHA_MODE             		IMEAS_CHA_MODE_SINGLE
//#define  IMEAS_CHA_MODE			IMEAS_CHA_MODE_SINGLE_CONT 	
//#define  IMEAS_CHA_MODE                       IMEAS_CHA_MODE_GROUP      	

#define  IMEAS_CHA_FORMAT_SEL       		IMEAS_CHA_FORMAT_SIGNED
//#define  IMEAS_CHA_FORMAT_SEL                 IMEAS_CHA_FORMAT_UNSIGNED

#define  IMEAS_WEBIAS_DAC           		0x3F // range [0x0 ~ 0x3F]; i.e.[0mv ~ 800mv]

#define  IMEAS_REBIAS_DAC           		IMEAS_REBIAS_DAC_1250mv
//#define  IMEAS_REBIAS_DAC           		IMEAS_REBIAS_DAC_1400mv
//#define  IMEAS_REBIAS_DAC           		IMEAS_REBIAS_DAC_1550mv
//#define  IMEAS_REBIAS_DAC           		IMEAS_REBIAS_DAC_1700mv

#define  IMEAS_GUBIAS_EN            		IMEAS_GUBIAS_EN_HIGH
//#define  IMEAS_GUBIAS_EN 			IMEAS_GUBIAS_EN_LOW

#define  IMEAS_CIC_RATE           		IMEAS_CIC_RATE_32
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_64
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_128
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_256
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_512
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_1024	
//#define  IMEAS_CIC_RATE			IMEAS_CIC_RATE_2048

#define  IMEAS_PGA_GAIN             		IMEAS_PGA_GAIN_1X
//#define  IMEAS_PGA_GAIN			IMEAS_PGA_GAIN_2X
//#define  IMEAS_PGA_GAIN			IMEAS_PGA_GAIN_4X
//#define  IMEAS_PGA_GAIN			IMEAS_PGA_GAIN_8X

/***********************************************************************************
***********************************************************************************/
//Set up address location	
#define  DFLASH_BASE_ADDR  			CMSDK_DFLASH_BASE
#define  IMEAS_1MIN_START_ADDR  		0x1000
#define  IMEAS_5MIN_START_ADDR  		0x2000
#define  ZMEAS_30MIN_START_ADDR  		0x3000
#define  IMEAS_1MIN_END_ADDR  			0x18F0 // ((1440times/day *6 bytes)/4)+IMEAS_1MIN_START_ADDR+0x80 = 0x18F0
#define  IMEAS_1MIN_1DAY_SECTOR_COUNT           0x12   // 18 sectors for 1 day 1min storage
/***********************************************************************************
***********************************************************************************/
//ZMEAS 
#define REG_NUMBER_OF_REPEAT_CYCLE_VAL        ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_2  
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_4  
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_8  
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_16  
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_32 
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_64 
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_128 
//#define REG_NUMBER_OF_REPEAT_CYCLE_VAL      ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_256
	 
#define REG_SETTLING_TIME_VAL                 0x3F // range [0x0 ~ 0xFF]

#define REG_FREQ_VAL   			      ZMEAS_REG_FREQ_VAL_1KHZ	 	 	 	  
//#define REG_FREQ_VAL			      ZMEAS_REG_FREQ_VAL_2KHZ        	 
//#define REG_FREQ_VAL			      ZMEAS_REG_FREQ_VAL_4KHZ        	 
//#define REG_FREQ_VAL			      ZMEAS_REG_FREQ_VAL_8KHZ        	 
//#define REG_FREQ_VAL			      ZMEAS_REG_FREQ_VAL_16KHZ       	 
//#define REG_FREQ_VAL			      ZMEAS_REG_FREQ_VAL_32KHZ       	 
//#define REG_FREQ_VAL		              ZMEAS_REG_NO_FREQVAL  

//#define ZMEAS_REG_MODE                      ZMEAS_NOOP
//#define ZMEAS_REG_MODE                      ZMEAS_INIT
//#define ZMEAS_REG_MODE                      ZMEAS_CALC 
//#define ZMEAS_REG_MODE                      ZMEAS_POWER_DOWN
//#define ZMEAS_REG_MODE                      ZMEAS_STANDBY
		 	
#define REPEAT_CALCULATION		      ZMEAS_REPEAT_CALCULATION_HIGH		
//#define REPEAT_CALCULATION     	      ZMEAS_REPEAT_CALCULATION_LOW

#define ENABLE_INTR			      ZMEAS_ENABLE_INTR_HIGH
//#define ENABLE_INTR		 	      ZMEAS_ENABLE_INTR_LOW

#define ENABLE_ADC_INTR		 	      ZMEAS_ENABLE_ADC_INTR_HIGH
//#define ENABLE_ADC_INTR	 	      ZMEAS_ENABLE_ADC_INTR_LOW

//#define MEASURE_CALIBRATE	              ZMEAS_CALIBRATE
#define MEASURE_CALIBRATE	              ZMEAS_MEASURE
		 	 
//#define CONFIG_VOLTAGE_RANGE		      ZMEAS_CONFIG_VOLTAGE_RANGE_0
//#define CONFIG_VOLTAGE_RANGE		      ZMEAS_CONFIG_VOLTAGE_RANGE_2
#define CONFIG_VOLTAGE_RANGE		      ZMEAS_CONFIG_VOLTAGE_RANGE_3

#define ZMEAS_PGA_GAIN			      ZMEAS_PGA_GAIN_1x			 
//#define ZMEAS_PGA_GAIN		      ZMEAS_PGA_GAIN_5x
			 
#define RESET_MEASUREMENT		      ZMEAS_RESET_MEASUREMENT_OFF		 	 
//#define RESET_MEASUREMENT		      ZMEAS_RESET_MEASUREMENT_ON

#define FDT0_VALUE                 	      0x4E
#define FDT1_VALUE                 	      0x4E

#define REBOUND_FILTER_VALUE                  0xC

#define ZMEAS_30MIN_CMD                       0xDA30
#define IMEAS_05MIN_CMD                       0xDA05
#define IMEAS_01MIN_CMD                       0xDA01
/***********************************************************************************
***********************************************************************************
***********************************************************************************
***********************************************************************************/
