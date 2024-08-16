//CLKDIV hfosc=8MHz, lfosc=256KHz
#define  HCLK_DIV_VAL_8MHZ         	             0
#define  HCLK_DIV_VAL_4MHZ        	             1
#define  HCLK_DIV_VAL_2MHZ         	             2
#define  HCLK_DIV_VAL_1MHZ         	             3
#define  PCLK_DIV_VAL_HCLK         	             0
#define  PCLK_DIV_VAL_HCLK_BY_2     	             1
#define  PCLK_DIV_VAL_HCLK_BY_4     	             2
#define  PCLK_DIV_VAL_HCLK_BY_8     	             3
#define  ICLK_DIV_VAL_128KHZ        	             0
#define  ICLK_DIV_VAL_64KHZ       	             1
#define  ICLK_DIV_VAL_32KHZ        	             2  
#define  ICLK_DIV_VAL_16KHZ        	             3
#define  ICLK_DIV_VAL_8KHZ        	             4
#define  ICLK_DIV_VAL_4KHZ         	             5
#define  ICLK_DIV_VAL_2KHZ         	             6
#define  ICLK_DIV_VAL_1KHZ          	             7
//IMEAS
#define  IMEAS_CHA_NUM_0		             0
#define  IMEAS_CHA_NUM_1                             1
#define  IMEAS_CHA_NUM_2		             2
#define  IMEAS_CHA_MODE_SINGLE      	             0
#define  IMEAS_CHA_MODE_SINGLE_CONT 	             1
#define  IMEAS_CHA_MODE_GROUP      	             2
#define  IMEAS_CHA_FORMAT_SIGNED    	             0
#define  IMEAS_CHA_FORMAT_UNSIGNED  	             1
#define  IMEAS_REBIAS_DAC_1250mv      	             0
#define  IMEAS_REBIAS_DAC_1400mv       	             1
#define  IMEAS_REBIAS_DAC_1550mv       	             2
#define  IMEAS_REBIAS_DAC_1700mv       	             3
#define  IMEAS_GUBIAS_EN_HIGH                        1
#define  IMEAS_GUBIAS_EN_LOW                         0   
#define  IMEAS_CIC_RATE_32          	             0
#define  IMEAS_CIC_RATE_64         	             1
#define  IMEAS_CIC_RATE_128        	             2
#define  IMEAS_CIC_RATE_256         	             3
#define  IMEAS_CIC_RATE_512        	             4
#define  IMEAS_CIC_RATE_1024       	             5
#define  IMEAS_CIC_RATE_2048        	             6
#define  IMEAS_PGA_GAIN_1X         	             0
#define  IMEAS_PGA_GAIN_2X          	             1
#define  IMEAS_PGA_GAIN_4X         	             2
#define  IMEAS_PGA_GAIN_8X         	             3
//ZMEAS
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_2      1
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_4      2
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_8      3
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_16     4
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_32     5
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_64     6
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_128    7
#define  ZMEAS_REG_NUMBER_OF_REPEAT_CYCLE_VAL_256    8
#define  ZMEAS_REG_FREQ_VAL_1KHZ	 	     1	 	  
#define  ZMEAS_REG_FREQ_VAL_2KHZ        	     2
#define  ZMEAS_REG_FREQ_VAL_4KHZ        	     3
#define  ZMEAS_REG_FREQ_VAL_8KHZ        	     4
#define  ZMEAS_REG_FREQ_VAL_16KHZ       	     5
#define  ZMEAS_REG_FREQ_VAL_32KHZ       	     6
#define  ZMEAS_REG_NO_FREQVAL        		     0	
#define  ZMEAS_NOOP			    	     0
#define  ZMEAS_INIT			             1
#define  ZMEAS_CALC  		                     3
#define  ZMEAS_POWER_DOWN		             2
#define  ZMEAS_STANDBY		                     6
#define  ZMEAS_REPEAT_CALCULATION_HIGH		     1
#define  ZMEAS_REPEAT_CALCULATION_LOW                0
#define	 ZMEAS_ENABLE_INTR_HIGH		             1
#define	 ZMEAS_ENABLE_ADC_INTR_HIGH		     1
#define	 ZMEAS_ENABLE_INTR_LOW			     0
#define	 ZMEAS_ENABLE_ADC_INTR_LOW		     0
#define  ZMEAS_CALIBRATE 		             1
#define  ZMEAS_MEASURE 		                     0 
#define  ZMEAS_CONFIG_VOLTAGE_RANGE_0		     0
#define  ZMEAS_CONFIG_VOLTAGE_RANGE_2		     2
#define  ZMEAS_CONFIG_VOLTAGE_RANGE_3		     3
#define  ZMEAS_PGA_GAIN_1x			     0
#define  ZMEAS_PGA_GAIN_5x			     1
#define  ZMEAS_RESET_MEASUREMENT_OFF		     0     	//normal operation
#define  ZMEAS_RESET_MEASUREMENT_ON	     	     1		//zmeas will reset itself to wait for next user controlled MODE and INIT values
