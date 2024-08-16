/*
���ܣ�IFT���Ų�ʾ��
˵����2ͨ������������ֱ������ͬƵ�ʲ���
����˳��
    �������Ҳ����ɵ����ò�����в���������ift����ʹ��2�����Ҳ���õ�
		����ADDR_WG_DRV_CONFIG_REG�Ĵ���ʱӦ���ö�缫ѡ�
		ÿ�������������Լ���Ƶ�ʣ���˻��������ЧӦ����ˣ�ÿ��driver����Ӧ����Ϊ���в�ͬ�������㲨�ε��������Ƶ��
        ���ڷ���f_overlay����T1-T0��/��T1xT0��
        ����T0=������A�е����Ҳ�����[0]
            T1=������A�е����Ҳ�����[1]
ע����������ʾ�Ĳ���ʹ�ö�缫ѡ����ʹ��������A[0]��������A[1]

*/  
#include <stdio.h>
#include "uart_stdout.h"
#include "CMSDK_CM0.h"
#include "sine.h"
#include "boost_select.h"
int wavegen_driverA_ift_sine_test(CMSDK_WAVE_GEN_TypeDef *CMSDK_WAVEGEN_DRVA);
volatile int count;
// --------------------------------------------------------------- //
//  UART1 exception handler                                        //
// --------------------------------------------------------------- //
void UART1_Handler(void) {
	
			uint8_t rev_data = 0;
			NVIC_ClearPendingIRQ(UART1_IRQn);//Clear Pending NVIC interrupt

    if(((CMSDK_UART1->IIR & CMSDK_UART_IIR_INT_TYPE_Msk) >> CMSDK_UART_IIR_INT_TYPE_Pos) == 0x3) {
//Disable Line status interrupt
    	CMSDK_UART1->IER &= ~CMSDK_UART_IER_RLSI_EN_Msk;
    }
    if((((CMSDK_UART1->IIR & CMSDK_UART_IIR_INT_TYPE_Msk) >> CMSDK_UART_IIR_INT_TYPE_Pos) == 0x2) || (((CMSDK_UART1->IIR & CMSDK_UART_IIR_INT_TYPE_Msk) >> CMSDK_UART_IIR_INT_TYPE_Pos) == 0x6)) {
//Disable Data Ready interrupt
    	CMSDK_UART1->IER &= ~CMSDK_UART_IER_RDAI_EN_Msk;
			rev_data = CMSDK_UART1->RBR; 
			CMSDK_UART1->IER |= CMSDK_UART_IER_RDAI_EN_Msk;
    }

 
    return;

}
/*******main()*******/
int main()
{   
    int result=0;
    count=0;
     
    //Set MTP Wait cycles
    CMSDK_MTPREG->MTP_CR = 0x00000002;
    // set hsi frequency as 32MHz
    CMSDK_SYSCON->HSI_CTRL = (CMSDK_SYSCON->HSI_CTRL & ~CMSDK_SYSCON_HSI_FREQ_Msk) | (0x3 << CMSDK_SYSCON_HSI_FREQ_Pos);
    // enable uart & WAVE_GEN pclk
    CMSDK_SYSCON->APB_CLKEN = 0x1003|0x4000;; 

	
	
	
	 		//boost��ѹѡ��
//		boost_voltage_select_11V();
//		boost_voltage_select_15V();
//		boost_voltage_select_26V();
		boost_voltage_select_45V();
//		boost_voltage_select_55V();

	
	
	
	
	
	
    // UART init
    UartStdOutInit();
	    //config gpio for UART1 (select ALT : 1) to use UART1(txd,rxd,rts,cts) to test
    CMSDK_GPIO->ALTFL |= (1<<24) | (1<<26) ;  

    NVIC_DisableIRQ(UART1_IRQn);//Disable NVIC interrupt
    NVIC_ClearPendingIRQ(UART1_IRQn);//Clear Pending NVIC interrupt
    NVIC_EnableIRQ(UART1_IRQn);//Enable NVIC interrupt
		CMSDK_UART1->IER |= CMSDK_UART_IER_RDAI_EN_Msk;
    puts("ENS1 - WAVE_GENERATOR_DRIVER_A_Test - $Revision: R001\n");
 
    //WAVEGEN driver_A IFT_SINE test
		
		
                        /*ͨ��һift*/
    result += wavegen_driverA_ift_sine_test(WAVE_GEN_DRVA_BLK0);
    result += wavegen_driverA_ift_sine_test(WAVE_GEN_DRVA_BLK1);
		
		
//                        /*ͨ����ift*/
//    result += wavegen_driverA_ift_sine_test(WAVE_GEN_DRVA_BLK2);
//    result += wavegen_driverA_ift_sine_test(WAVE_GEN_DRVA_BLK3);												

    //enable drivers
    puts("Enable Driver 0 & 1\n");
		
		
		
                        /*ͨ��һiftʹ��*/
    WAVE_GEN_DRVA_BLK0->WAVE_GEN_DRV_CTRL_REG = 0x00000001;
    WAVE_GEN_DRVA_BLK1->WAVE_GEN_DRV_CTRL_REG = 0x00000001;
		
		
//		
//                        /*ͨ����iftʹ��*/
//    WAVE_GEN_DRVA_BLK2->WAVE_GEN_DRV_CTRL_REG = 0x00000001;
//    WAVE_GEN_DRVA_BLK3->WAVE_GEN_DRV_CTRL_REG = 0x00000001;  


    while(count < 0x2000) {
    	count++;
    };
    count=0;

    if (result==0) {
				puts ("** TEST PASSED **\n");
    } else {
				puts ("** TEST FAILED **\n");
    }

    UartEndSimulation();
    return 0;
}

/* --------------------------------------------------------------- */
/*  WAVE GEN Driver_A IFT_SINE test                                */
/* --------------------------------------------------------------- */

int wavegen_driverA_ift_sine_test(CMSDK_WAVE_GEN_TypeDef *CMSDK_WAVEGEN_DRVA){
    int return_val=0;
    int err_code=0; 
    
    puts("\nDRIVER A IFT_SINE TEST\n");

    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_REST_T_REG = 0x00000000;
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_SILENT_T_REG = 0x00000000; 
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_CLK_FREQ_REG = 0x00000020;     //32MHZ ==PCLK
	
	
	
	
		CMSDK_WAVEGEN_DRVA ->WAVE_GEN_DRV_ISEL_REG = 1;  // �ܵ��� = ��Ԫ����ISEL * WAVE_GEN_DRV_IN_WAVE_REG ��0-7��
	
	
															/*ͨ��һ����*/
    if(CMSDK_WAVEGEN_DRVA == WAVE_GEN_DRVA_BLK0) {//5Khz
				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_HLF_WAVE_PRD_REG = 50000; //100us
				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_HLF_WAVE_PRD_REG = 50000; //100us
    }
															/*ͨ��������*/
    else if(CMSDK_WAVEGEN_DRVA == WAVE_GEN_DRVA_BLK1) {
				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_HLF_WAVE_PRD_REG = 50020; //104us
				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_HLF_WAVE_PRD_REG = 50020; //104us
    }
		
		
		
//															/*ͨ��������*/
//	 else if(CMSDK_WAVEGEN_DRVA == WAVE_GEN_DRVA_BLK2) {
//				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_HLF_WAVE_PRD_REG = 50020; //104us
//				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_HLF_WAVE_PRD_REG = 50020; //104us
//    }
//															/*ͨ���Ŀ���*/
//    else if(CMSDK_WAVEGEN_DRVA == WAVE_GEN_DRVA_BLK3) {
//				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_HLF_WAVE_PRD_REG = 50020; //104us
//				CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_HLF_WAVE_PRD_REG = 50020; //104us
//    }
			

		
		
		
		
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_CONFIG_REG = 0x0000004A;//bit 0:rest enable, 1:negative enable, 2: silent enable, 3: source B enable, 5: continue mode, 6: multi-electrode


    for(int i=0; i<64; i++){
       	CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_IN_WAVE_ADDR_REG = i; 
       	CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_IN_WAVE_REG = sine_data[i];
    }

    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_ALT_LIM_REG = 0x00000000;// no alternating patterns 
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_ALT_SILENT_LIM_REG = 0x00000000;// number of clocks to be silent after alternating.In this case, driver b is continuously alternating
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_DELAY_LIM_REG = 0x00000000;// number of delayed clockes
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_SCALE_REG = 0x00000001; //scale (multiply) the negative side by this register value
    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_NEG_OFFSET_REG = 0x00000000;//offset (add) the negative side by this register value

    CMSDK_WAVEGEN_DRVA->WAVE_GEN_DRV_INT_REG = 0x0;//interrupt register set to 0 

    /* Generate return value */
    if (err_code != 0) {
       puts("\nERROR : Driver A Test Failed\n");
       return_val=1;
       err_code = 0;
    }

    return(return_val);
}
