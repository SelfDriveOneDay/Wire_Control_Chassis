/*
 * PortApp.c
 */

#include "Port/Io/IfxPort_Io.h"
#include "SysSe/Bsp/Bsp.h"
#include "PortApp.h"
#define PORT_DIVIDER_1 1000


/*************************************************************************
*  �������ƣ�module_port_init
*  ����˵����LED���ų�ʼ������
*  ʹ��˵���������������н��г�ʼ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��5��
*  ��    ע��
*************************************************************************/
void module_port_init(void){

	// configure P33.8~11 as general output
	IfxPort_setPinMode(&MODULE_P33, 8, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 9, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 10, IfxPort_Mode_outputPushPullGeneral);
	IfxPort_setPinMode(&MODULE_P33, 11, IfxPort_Mode_outputPushPullGeneral); 	// 4��LED����
	IfxPort_setPinMode(&MODULE_P33, 0, IfxPort_Mode_outputPushPullGeneral);		// ���������ţ� P33.0

	// ��ʼ��4��LED���ŵĵ�ƽΪ�ߵ�ƽ��LEDΪϨ��״̬
	IfxPort_setPinHigh(&MODULE_P33, 8);
	IfxPort_setPinHigh(&MODULE_P33, 9);
	IfxPort_setPinHigh(&MODULE_P33, 10);
	IfxPort_setPinHigh(&MODULE_P33, 11);
	IfxPort_setPinHigh(&MODULE_P33, 0);

}



/*************************************************************************
*  �������ƣ�module_port_run
*  ����˵����LED��˸����
*  ʹ��˵����������������ѭ��������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��5��
*  ��    ע��
*************************************************************************/
void module_port_run(void)
{
	IfxPort_togglePin(&MODULE_P33, 8);			//LED1
	IfxPort_togglePin(&MODULE_P33, 9);			//LED2
	IfxPort_togglePin(&MODULE_P33, 10);			//LED3
	IfxPort_togglePin(&MODULE_P33, 11);			//LED4
	IfxStm_waitTicks(&MODULE_STM0, 10000000);	//100ms, ����ʱ

}

void led_flow(int count)
{
	switch(count)
	{
		case 0: IfxPort_setPinLow(&MODULE_P33, 8);		//LED1��
				IfxPort_setPinHigh(&MODULE_P33, 11);	//LED4��
				break;

		case 1: IfxPort_setPinLow(&MODULE_P33, 9);		//LED2��
				IfxPort_setPinHigh(&MODULE_P33, 8);		//LED1��
				break;

		case 2: IfxPort_setPinLow(&MODULE_P33, 10);		//LED3��
				IfxPort_setPinHigh(&MODULE_P33, 9);		//LED2��
				break;

		case 3: IfxPort_setPinLow(&MODULE_P33, 11);		//LED4��
				IfxPort_setPinHigh(&MODULE_P33, 10);	//LED3��
				break;

		default: break;
	}
	//IfxStm_waitTicks(&MODULE_STM0, 100000000);	//Ӳ��ʱ
}
