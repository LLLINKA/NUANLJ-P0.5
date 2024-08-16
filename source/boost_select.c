#include <stdio.h>
#include "CMSDK_CM0.h"

//32兆主频下，设置boost电压为11V
void boost_voltage_select_11V(void)
{
	//		//10us 20%占空比64分频  0.5A   11V  关闭过温   内部
		CMSDK_ANAC->BOOST_CTRL =0x71013;//内部boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32兆主频下，设置boost电压为15V
void boost_voltage_select_15V(void)
{
		//10us 20%占空比64分频  0.5A   15V  关闭过温   内部
		CMSDK_ANAC->BOOST_CTRL =0x71113;//内部boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32兆主频下，设置boost电压为26V
void boost_voltage_select_26V(void)
{
			//10us 20%占空比64分频  0.5A   26V  关闭过温   内部
		CMSDK_ANAC->BOOST_CTRL =0x71213;//内部boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32兆主频下，设置boost电压为45V
void boost_voltage_select_45V(void)
{
		//10%占空比64分频  0.5A   45V  关闭过温   内部
		CMSDK_ANAC->BOOST_CTRL =0xc1413;//
		CMSDK_ANAC->PMU_CTRL = 0x10;

}
//32兆主频下，设置boost电压为55V
void boost_voltage_select_55V(void)
{
		//10%占空比64分频  0.5A   55V  关闭过温   内部
		CMSDK_ANAC->BOOST_CTRL =0xc1713;//内部boost	
		CMSDK_ANAC->PMU_CTRL = 0x10;

}



















