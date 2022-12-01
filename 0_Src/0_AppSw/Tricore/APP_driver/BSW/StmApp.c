
#include "IfxStm_Timer.h"
#include "Cpu/Irq/IfxCpu_Irq.h"
#include "StmApp.h"
#include "ASW/VCU_Task.h"


Ifx_STM *stmSfr0;
IfxStm_CompareConfig stmConfig0;
#define STM0PRIO 20

uint8 flag_20ms = 0, flag_50ms = 0, flag_100ms = 0, flag_200ms = 0, flag_500ms = 0;
uint16 count_20ms = 0, count_50ms = 0, count_100ms = 0, count_200ms = 0, count_500ms = 0;

 /*************************************************************************
 *  函数名称：IFX_INTERRUPT
 *  功能说明：中断函数
 *  使用说明    IFX_INTERRUPT_INTERNAL(isr, vectabNum, prio)  //isr:stm0Sr0ISR(中断函数名)
 *  vectabNum：代表CPU0，参数范围[0,2] prio：优先级，参数值和STM0_Init里定义的优先级一致
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年6月27日
 *  备    注：#define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)   所以实际函数名为stm0Sr0ISR
 *************************************************************************/
uint8 count = 1;
sint32 pos = 0;
IFX_INTERRUPT(stm0Sr0ISR, 0, STM0PRIO)
{
	// 底盘控制任务1ms
	Chassis_Control_Task();

	count_20ms++;
	if(count_20ms == 20)
	{
		count_20ms = 0;
		flag_20ms = 1;
	}

	count_50ms++;
	if(count_50ms == 50)
	{
		count_50ms = 0;
		flag_50ms = 1;
		// send rtrmsg to get motors and odrives states, different states have different response frequency
		//CAN0Send_RtrMsgProcess();
	}

	count_100ms++;
	if(count_100ms == 100)
	{
		count_100ms = 0;
		flag_100ms = 1;
	}

	count_200ms++;
	if(count_200ms == 200)
	{
		count_200ms = 0;
		flag_200ms = 1;
	}

	//IfxPort_togglePin(&MODULE_P33, 9);
    IfxStm_increaseCompare(stmSfr0, stmConfig0.comparator, stmConfig0.ticks);
	IfxStm_clearCompareFlag(stmSfr0, stmConfig0.comparator);
}


/*************************************************************************
*  函数名称：STM0_Init
*  功能说明：中断初始化函数
*  使用说明    放在主函数的初始化界面中
*  参数说明：无
*  函数返回：无
*  修改时间：2020年6月27日
*  备    注：使用此函数时应注意修改对应的定时器模块，假如使用多个定时器时，建议复制加一个子函数类似于STM1_Init（）再加以修改
*************************************************************************/

void STM0_Init(void)
{   //MODULE_STM0可以选择STM1,STM2
	stmSfr0 = &MODULE_STM0;
	//初始化STM的配置
	IfxStm_initCompareConfig(&stmConfig0);
	/*配置比较寄存器的偏移长度和位置
	 *compareSize代表偏移长度，取值范围[0,31]；
	 *compareOffset代表偏移位置，取值范围[0,31]；
	 */
	stmConfig0.compareSize=IfxStm_ComparatorSize_32Bits;
	stmConfig0.compareOffset=IfxStm_ComparatorOffset_0;
    /*配置STM的计时的时间，
     * 单位：微秒 ...如果是1，  就是定时1微秒     参数范围[0，42949672]
              也可以用函数IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 1000)实现1s定时
                                        单位是毫秒                             参数范围[0，42949]
    */
	uint32 ticks = IfxStm_getTicksFromMicroseconds(&MODULE_STM0,1000);	//定时1ms, 即1000us
	/*配置ticks的值*/
	stmConfig0.ticks = ticks;
	/*配置比较寄存器
	 *取值范围0,1
	 */
	stmConfig0.comparator=1;
	/*配置STM的中断输出为
	 *如果要让中断输出为STMIR0，则替换为IfxStm_ComparatorInterrupt_ir0
	 */
	stmConfig0.comparatorInterrupt  = IfxStm_ComparatorInterrupt_ir1;
	/*配置stmConfig0的优先级
	 *0代表禁止中断*/
	stmConfig0.triggerPriority = STM0PRIO;
	/*配置STM触发输出的中断服务请求为CPU0
	 *如果要输出的中断服务请求为CPU1，应写为： stmConfig0.typeOfService = IfxSrc_Tos_cpu1;
	 *如果要输出的中断服务请求为CPU2，应写为： stmConfig0.typeOfService = IfxSrc_Tos_cpu2;
	 */
	stmConfig0.typeOfService = IfxSrc_Tos_cpu0;
	IfxStm_initCompare(stmSfr0, &stmConfig0);
	/*安装中断服务函数IfxCpu_Irq_installInterruptHandler(&stm0Sr0ISR, STM0PRIO);
	  stm0Sr0ISR为中断函数名，STM0PRIO为优先级
	 */
	IfxCpu_Irq_installInterruptHandler(&stm0Sr0ISR, STM0PRIO);
	IfxCpu_enableInterrupts();
}


/*************************************************************************
*  函数名称：IO_Init
*  功能说明：翻转引脚电平
*  使用说明    放在主函数的初始化界面中
*  参数说明：无
*  函数返回：无
*  修改时间：2020年6月27日
*  备    注：无
*************************************************************************/

void IO_Init(void)
{
IfxPort_setPinMode(&MODULE_P33, 8, IfxPort_Mode_outputPushPullGeneral);
}
