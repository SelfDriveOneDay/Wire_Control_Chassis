
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
 *  �������ƣ�IFX_INTERRUPT
 *  ����˵�����жϺ���
 *  ʹ��˵��    IFX_INTERRUPT_INTERNAL(isr, vectabNum, prio)  //isr:stm0Sr0ISR(�жϺ�����)
 *  vectabNum������CPU0��������Χ[0,2] prio�����ȼ�������ֵ��STM0_Init�ﶨ������ȼ�һ��
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��6��27��
 *  ��    ע��#define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)   ����ʵ�ʺ�����Ϊstm0Sr0ISR
 *************************************************************************/
uint8 count = 1;
sint32 pos = 0;
IFX_INTERRUPT(stm0Sr0ISR, 0, STM0PRIO)
{
	// ���̿�������1ms
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
*  �������ƣ�STM0_Init
*  ����˵�����жϳ�ʼ������
*  ʹ��˵��    �����������ĳ�ʼ��������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��6��27��
*  ��    ע��ʹ�ô˺���ʱӦע���޸Ķ�Ӧ�Ķ�ʱ��ģ�飬����ʹ�ö����ʱ��ʱ�����鸴�Ƽ�һ���Ӻ���������STM1_Init�����ټ����޸�
*************************************************************************/

void STM0_Init(void)
{   //MODULE_STM0����ѡ��STM1,STM2
	stmSfr0 = &MODULE_STM0;
	//��ʼ��STM������
	IfxStm_initCompareConfig(&stmConfig0);
	/*���ñȽϼĴ�����ƫ�Ƴ��Ⱥ�λ��
	 *compareSize����ƫ�Ƴ��ȣ�ȡֵ��Χ[0,31]��
	 *compareOffset����ƫ��λ�ã�ȡֵ��Χ[0,31]��
	 */
	stmConfig0.compareSize=IfxStm_ComparatorSize_32Bits;
	stmConfig0.compareOffset=IfxStm_ComparatorOffset_0;
    /*����STM�ļ�ʱ��ʱ�䣬
     * ��λ��΢�� ...�����1��  ���Ƕ�ʱ1΢��     ������Χ[0��42949672]
              Ҳ�����ú���IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 1000)ʵ��1s��ʱ
                                        ��λ�Ǻ���                             ������Χ[0��42949]
    */
	uint32 ticks = IfxStm_getTicksFromMicroseconds(&MODULE_STM0,1000);	//��ʱ1ms, ��1000us
	/*����ticks��ֵ*/
	stmConfig0.ticks = ticks;
	/*���ñȽϼĴ���
	 *ȡֵ��Χ0,1
	 */
	stmConfig0.comparator=1;
	/*����STM���ж����Ϊ
	 *���Ҫ���ж����ΪSTMIR0�����滻ΪIfxStm_ComparatorInterrupt_ir0
	 */
	stmConfig0.comparatorInterrupt  = IfxStm_ComparatorInterrupt_ir1;
	/*����stmConfig0�����ȼ�
	 *0�����ֹ�ж�*/
	stmConfig0.triggerPriority = STM0PRIO;
	/*����STM����������жϷ�������ΪCPU0
	 *���Ҫ������жϷ�������ΪCPU1��ӦдΪ�� stmConfig0.typeOfService = IfxSrc_Tos_cpu1;
	 *���Ҫ������жϷ�������ΪCPU2��ӦдΪ�� stmConfig0.typeOfService = IfxSrc_Tos_cpu2;
	 */
	stmConfig0.typeOfService = IfxSrc_Tos_cpu0;
	IfxStm_initCompare(stmSfr0, &stmConfig0);
	/*��װ�жϷ�����IfxCpu_Irq_installInterruptHandler(&stm0Sr0ISR, STM0PRIO);
	  stm0Sr0ISRΪ�жϺ�������STM0PRIOΪ���ȼ�
	 */
	IfxCpu_Irq_installInterruptHandler(&stm0Sr0ISR, STM0PRIO);
	IfxCpu_enableInterrupts();
}


/*************************************************************************
*  �������ƣ�IO_Init
*  ����˵������ת���ŵ�ƽ
*  ʹ��˵��    �����������ĳ�ʼ��������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��6��27��
*  ��    ע����
*************************************************************************/

void IO_Init(void)
{
IfxPort_setPinMode(&MODULE_P33, 8, IfxPort_Mode_outputPushPullGeneral);
}
