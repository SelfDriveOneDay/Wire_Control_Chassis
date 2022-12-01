#include "AscApp.h"
#include <stdio.h>
#include <IfxAsclin_reg.h>
#include "IfxCpu.h"

#define ASC_TX_BUFFER_SIZE 128	// 256 ����35���ֽڵ�SBUS������
static uint8 ascTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
#define ASC_RX_BUFFER_SIZE 128
static uint8 ascRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

// used globally
IfxAsclin_Asc asc;

uint8 Rx_Count = 0, date = 0;				// �������ݼ���
uint8 SBUS_Buffer[35] = {0};	// SBUS֡���ݽ��ջ�����

/******************************************************************************/

/*************************************************************************
*  �������ƣ�IFX_INTERRUPT
*  ����˵�����жϺ���
*  ʹ��˵��    IFX_INTERRUPT_INTERNAL(isr, vectabNum, prio)  //isr:stm0Sr0ISR(�жϺ�����) vectabNum������CPU0��������Χ[0,2] prio�����ȼ�������ֵ��STM0_Init�ﶨ������ȼ�һ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��10��
*  ��    ע��#define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)   ����ʵ�ʺ�����Ϊasclin3TxISR��asclin3RxISR��asclin3ErISR
*************************************************************************/
IFX_INTERRUPT(asclin3TxISR, 0, IFX_INTPRIO_ASCLIN3_TX)
{
    IfxAsclin_Asc_isrTransmit(&asc);
    IfxPort_togglePin(&MODULE_P33, 8);			//LED1

}
IFX_INTERRUPT(asclin3RxISR, 0, IFX_INTPRIO_ASCLIN3_RX)
{
    IfxAsclin_Asc_isrReceive(&asc);

    IfxPort_togglePin(&MODULE_P33, 9);			//LED2

    Rx_Callback();
    Sbus_Decode(SBUS_Buffer);

}
IFX_INTERRUPT(asclin3ErISR, 0, IFX_INTPRIO_ASCLIN3_ER)
{
    IfxAsclin_Asc_isrError(&asc);
   // IfxPort_togglePin(&MODULE_P33, 10);			//LED3
}

/*************************************************************************
*  �������ƣ�AsclinAsc_init
*  ����˵����UART��ʼ������
*  ʹ��˵��    �����������ĳ�ʼ��������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��10��
*  ��    ע��RX��TX���Ų��ܲ巴�ˣ���Ȼ�ղ�������
*************************************************************************/
void AsclinAsc_init(void)
{
	IfxCpu_disableInterrupts();	// disable interrupts

    /* create module config */
    IfxAsclin_Asc_Config ascConfig;   //����һ��ascConfig�Ľṹ�壬���ڱ�ʾasc����

    IfxAsclin_Asc_initModuleConfig(&ascConfig, &MODULE_ASCLIN3);

    ascConfig.clockSource = IfxAsclin_ClockSource_ascFastClock;

    /* set the desired baudrate */
   // ascConfig.baudrate.prescaler    = 1;
    ascConfig.baudrate.baudrate     = 115200; /* FDR values will be calculated in initModule */
    ascConfig.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;

    /* Configure the sampling mode */
	ascConfig.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;             /* Set the number of samples per bit*/
	ascConfig.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;    /* Set the first sample position    */


    ascConfig.frame.frameMode = IfxAsclin_FrameMode_asc;
    ascConfig.frame.dataLength = IfxAsclin_DataLength_8;
    ascConfig.frame.stopBit = IfxAsclin_StopBit_1;
    ascConfig.frame.shiftDir = IfxAsclin_ShiftDirection_lsbFirst;
    ascConfig.frame.parityBit = FALSE;
    //ascConfig.frame.idleDelay = IfxAsclin_IdleDelay_0;		// �ֽ����ֽڼ䲻������ͣ

    // ISR priorities and interrupt target
    ascConfig.interrupt.txPriority = IFX_INTPRIO_ASCLIN3_TX;
    ascConfig.interrupt.rxPriority = IFX_INTPRIO_ASCLIN3_RX;
    ascConfig.interrupt.erPriority = IFX_INTPRIO_ASCLIN3_ER;
    ascConfig.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    ascConfig.txBuffer = &ascTxBuffer;
    ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConfig.rxBuffer = &ascRxBuffer;
    ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;


    /* pin configuration */ //��������
	const IfxAsclin_Asc_Pins pins =
	{
		NULL,                     IfxPort_InputMode_pullUp,
		&IfxAsclin3_RXE_P00_1_IN, IfxPort_InputMode_pullUp,        // Rx pin
		NULL,                     IfxPort_OutputMode_pushPull,
		&IfxAsclin3_TX_P00_0_OUT, IfxPort_OutputMode_pushPull,		// Tx pin
		IfxPort_PadDriver_cmosAutomotiveSpeed4
		//IfxPort_PadDriver_lvdsSpeed4
		//IfxPort_PadDriver_ttlSpeed4
	};
	ascConfig.pins = &pins;

	IfxCpu_Irq_installInterruptHandler(&asclin3TxISR, IFX_INTPRIO_ASCLIN3_TX);
	IfxCpu_Irq_installInterruptHandler(&asclin3RxISR, IFX_INTPRIO_ASCLIN3_RX);
	IfxCpu_Irq_installInterruptHandler(&asclin3ErISR, IFX_INTPRIO_ASCLIN3_ER);

    /* initialize module */
    IfxAsclin_Asc_initModule(&asc, &ascConfig);

    IfxCpu_enableInterrupts();
}




/*************************************************************************
*  �������ƣ�Asclin_Send_Data
*  ����˵����UART���ͺ���
*  ʹ��˵��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��10��
*  ��    ע�����ݸ�ʽΪ0x00��0xFF����Ҫ�����ַ���������ת��ΪascII��
*************************************************************************/
void Asclin_Send_Data(uint8 Asclin_Send_Data)
{
	IfxAsclin_Asc_blockingWrite(&asc,Asclin_Send_Data);

}

/*************************************************************************
*  �������ƣ�Asclin_Read_Data
*  ����˵����UART���պ���
*  ʹ��˵��    �����������ĳ�ʼ��������
*  ����˵������
*  �������أ����ض�ȡ����ֵ
*  �޸�ʱ�䣺2020��7��10��
*  ��    ע�����ݸ�ʽΪ0x00��0xFF����Ҫ�����ַ���������ת��ΪascII��
*************************************************************************/
uint8 Asclin_Read_Data(void)
{
	if(IfxAsclin_Asc_getReadCount(&asc) >0)
	{
		return IfxAsclin_Asc_blockingRead(&asc);
	}

	return 0;

}

/* uint8 *value------------------Ҫ���͵�����ָ��
 * uint8 count-------------------������Ԫ�ظ���
 * */

/*************************************************************************
*  �������ƣ�Asclin_Demo_WriteData
*  ����˵����UART�жϷ��ͺ���
*  ʹ��˵��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��7��10��
*  ��    ע��
*************************************************************************/
boolean Asclin_Demo_WriteData(uint8 *value,uint8 count)
{

	return IfxAsclin_Asc_write(&asc,value,&count,TIME_INFINITE);
}


// �����ַ�
void UART_Send_Char(uint8 c)
{
	IfxAsclin_Asc_blockingWrite(&asc, c);
}

// �����ַ���
void UART_Send_Str(uint8 *str)
{
	while(*str != '\0') // or *str
	{
		UART_Send_Char(*str++);
	}
}

// �����ַ�����, �ɱ䳤��
void UART_Send_Data(uint8 *data, uint8 len)
{
	while(len--)
	{
		UART_Send_Char(*data++);
	}
}

uart_data UART_Read_Char(void)
{
	uart_data u_data = {0};

	if(IfxAsclin_Asc_getReadCount(&asc) > 0)	// �жϽ��ջ������Ƿ�������
	{
		u_data.data = IfxAsclin_Asc_blockingRead(&asc);
		u_data.flag = TRUE;
		return u_data;
	}

	return u_data;
}

//// ������
//uint8 UART_Read_Data(uint8 *data, uint8 len)
//{
//	uint8 count = len;
//
//	if(IfxAsclin_Asc_getReadCount(&asc) < len)	// �жϽ��ջ������Ƿ�������
//	{
//		return 0;			// ���Ȳ����˳�����
//	}
//
//	// ���ݳ��ȴﵽĿ�곤��Ҫ�󣬽�������  bug ����
//	IfxAsclin_Asc_read(&asc, data, &count, TIME_NULL);
//	return 1;
//}

// ����SBUS����֡��֡ͷ0x0F-ͨ��ֵ-FLAG-XOR
void Rx_Callback(void)
{
	uart_data u_data;
	uint8 i;

	u_data = UART_Read_Char();
	if(u_data.flag == TRUE)
	{
		SBUS_Buffer[Rx_Count] = u_data.data;
	}

	// �ж�֡ͷ�������˳�����
	if(Rx_Count == 0 && SBUS_Buffer[Rx_Count] != 0x0F) return ;

	Rx_Count++;

	if(SBUS_Buffer[0] == 0x0F && Rx_Count == 35)
	{
		//UART_Send_Data(SBUS_Buffer, 35);

//		for(i = 0; i < 35; i++)		// ��Ϊÿ������һ֡�󣬾Ͷ�SBUS_Bufferȫ���㣬 ���Լ���Asclin_Read_Data()����0x00������SBUS_BufferҲ�ܴ�����
//		{
//			SBUS_Buffer[i] = 0;
//		}
		IfxPort_togglePin(&MODULE_P33, 8);
		Rx_Count = 0;
	}

}



