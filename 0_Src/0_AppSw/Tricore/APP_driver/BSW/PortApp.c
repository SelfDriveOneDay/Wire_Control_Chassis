/*
 * PortApp.c
 */

#include "Port/Io/IfxPort_Io.h"
#include "SysSe/Bsp/Bsp.h"
#include "PortApp.h"
#define PORT_DIVIDER_1 1000


/*************************************************************************
*  函数名称：module_port_init
*  功能说明：LED引脚初始化函数
*  使用说明：放在主函数中进行初始化
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月5日
*  备    注：
*************************************************************************/
void module_port_init(void){

	// configure P33.8~11 as general output
	IfxPort_setPinMode(&MODULE_P33, 8, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 9, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 10, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 11, IfxPort_Mode_outputPushPullGeneral); 	// 4个LED引脚
	IfxPort_setPinMode(&MODULE_P33, 0, IfxPort_Mode_outputPushPullGeneral);		// 蜂鸣器引脚， P33.0

	// 初始化4个LED引脚的电平为高电平，LED为熄灭状态
	IfxPort_setPinHigh(&MODULE_P33, 8);
	IfxPort_setPinHigh(&MODULE_P33, 9);
	IfxPort_setPinHigh(&MODULE_P33, 10);
	IfxPort_setPinHigh(&MODULE_P33, 11);
	IfxPort_setPinHigh(&MODULE_P33, 0);

}



/*************************************************************************
*  函数名称：module_port_run
*  功能说明：LED闪烁函数
*  使用说明：放在主函数的循环部分在
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月5日
*  备    注：
*************************************************************************/
void module_port_run(void)
{
	IfxPort_togglePin(&MODULE_P33, 8);			//LED1
	IfxPort_togglePin(&MODULE_P33, 9);			//LED2
	IfxPort_togglePin(&MODULE_P33, 10);			//LED3
	IfxPort_togglePin(&MODULE_P33, 11);			//LED4
	IfxStm_waitTicks(&MODULE_STM0, 10000000);	//100ms, 死延时

}

void led_flow(int count)
{
	switch(count)
	{
		case 0: IfxPort_setPinLow(&MODULE_P33, 8);		//LED1亮
				IfxPort_setPinHigh(&MODULE_P33, 11);	//LED4灭
				break;

		case 1: IfxPort_setPinLow(&MODULE_P33, 9);		//LED2亮
				IfxPort_setPinHigh(&MODULE_P33, 8);		//LED1灭
				break;

		case 2: IfxPort_setPinLow(&MODULE_P33, 10);		//LED3亮
				IfxPort_setPinHigh(&MODULE_P33, 9);		//LED2灭
				break;

		case 3: IfxPort_setPinLow(&MODULE_P33, 11);		//LED4亮
				IfxPort_setPinHigh(&MODULE_P33, 10);	//LED3灭
				break;

		default: break;
	}
	//IfxStm_waitTicks(&MODULE_STM0, 100000000);	//硬延时
}
