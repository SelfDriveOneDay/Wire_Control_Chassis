#include "Cpu0_Main.h"
#include "ASW/CanbusTask.h"

App_Cpu0 g_AppCpu0; /**< \brief CPU 0 global data */

extern motor Motor[3];

/*************************************************************************
*  函数名称：core0_main
*  功能说明：CPU0的主函数
*  实验效果：    运行程序后可以看到P33.8~P33.11上的LED持续闪烁
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月5日
*  备    注：
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

    module_port_init();		// led所在引脚初始化程序
    STM0_Init();			// STM初始化
    CanApp_Init();			// CAN初始化
    AsclinAsc_init();		// UART初始化
    motor_params_init();	// 电机参数初始化

    // 中断使能在点击参数初始化之后
    /* Enable the global interrupts of this CPU */
    IfxCpu_enableInterrupts();

    /* background endless loop */
    while (TRUE)
    {
    	// 位置控制需要时间 需要1s
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



