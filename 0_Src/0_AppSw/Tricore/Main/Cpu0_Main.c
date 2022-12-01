#include "Cpu0_Main.h"
#include "ASW/CanbusTask.h"

App_Cpu0 g_AppCpu0; /**< \brief CPU 0 global data */

extern motor Motor[3];

/*************************************************************************
*  �������ƣ�core0_main
*  ����˵����CPU0��������
*  ʵ��Ч����    ���г������Կ���P33.8~P33.11�ϵ�LED������˸
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��5��
*  ��    ע��
*************************************************************************/
int core0_main(void)
{
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* Initialise the application state */
    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

    module_port_init();		// led�������ų�ʼ������
    STM0_Init();			// STM��ʼ��
    CanApp_Init();			// CAN��ʼ��
    AsclinAsc_init();		// UART��ʼ��
    motor_params_init();	// ���������ʼ��

    // �ж�ʹ���ڵ��������ʼ��֮��
    /* Enable the global interrupts of this CPU */
    IfxCpu_enableInterrupts();

    /* background endless loop */
    while (TRUE)
    {
    	// λ�ÿ�����Ҫʱ�� ��Ҫ1s
//    	set_aim_vel(SM, 2000, 0, 0);
//
//    	IfxStm_waitTicks(&MODULE_STM0,5000000);
//
//    	set_aim_vel(SM, 3000, 0, 0);
//
//    	IfxStm_waitTicks(&MODULE_STM0,5000000);
//
//    	set_aim_vel(SM, 2000, 0, 0);
//
//    	IfxStm_waitTicks(&MODULE_STM0,5000000);
//
//    	set_aim_vel(SM, 1000, 0, 0);
//
//    	IfxStm_waitTicks(&MODULE_STM0,5000000);

    }

}



