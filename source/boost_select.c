#include <stdio.h>
#include "CMSDK_CM0.h"

//32����Ƶ�£�����boost��ѹΪ11V
void boost_voltage_select_11V(void)
{
	//		//10us 20%ռ�ձ�64��Ƶ  0.5A   11V  �رչ���   �ڲ�
		CMSDK_ANAC->BOOST_CTRL =0x71013;//�ڲ�boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32����Ƶ�£�����boost��ѹΪ15V
void boost_voltage_select_15V(void)
{
		//10us 20%ռ�ձ�64��Ƶ  0.5A   15V  �رչ���   �ڲ�
		CMSDK_ANAC->BOOST_CTRL =0x71113;//�ڲ�boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32����Ƶ�£�����boost��ѹΪ26V
void boost_voltage_select_26V(void)
{
			//10us 20%ռ�ձ�64��Ƶ  0.5A   26V  �رչ���   �ڲ�
		CMSDK_ANAC->BOOST_CTRL =0x71213;//�ڲ�boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32����Ƶ�£�����boost��ѹΪ45V
void boost_voltage_select_45V(void)
{
		//10%ռ�ձ�64��Ƶ  0.5A   45V  �رչ���   �ڲ�
		CMSDK_ANAC->BOOST_CTRL =0xc1413;//
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32����Ƶ�£�����boost��ѹΪ55V
void boost_voltage_select_55V(void)
{
		//10%ռ�ձ�64��Ƶ  0.5A   55V  �رչ���   �ڲ�
		CMSDK_ANAC->BOOST_CTRL =0xc1713;//�ڲ�boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}



















