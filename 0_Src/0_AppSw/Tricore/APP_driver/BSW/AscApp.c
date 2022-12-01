#include "AscApp.h"
#include <stdio.h>
#include <IfxAsclin_reg.h>
#include "IfxCpu.h"

#define ASC_TX_BUFFER_SIZE 128	// 256 接收35个字节的SBUS够用了
static uint8 ascTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
#define ASC_RX_BUFFER_SIZE 128
static uint8 ascRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

// used globally
IfxAsclin_Asc asc;

uint8 Rx_Count = 0, date = 0;				// 接收数据计数
uint8 SBUS_Buffer[35] = {0};	// SBUS帧数据接收缓冲区

/******************************************************************************/

/*************************************************************************
*  函数名称：IFX_INTERRUPT
*  功能说明：中断函数
*  使用说明    IFX_INTERRUPT_INTERNAL(isr, vectabNum, prio)  //isr:stm0Sr0ISR(中断函数名) vectabNum：代表CPU0，参数范围[0,2] prio：优先级，参数值和STM0_Init里定义的优先级一致
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月10日
*  备    注：#define IFX_INTERRUPT(isr, vectabNum, prio) void isr(void)   所以实际函数名为asclin3TxISR，asclin3RxISR和asclin3ErISR
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
*  函数名称：AsclinAsc_init
*  功能说明：UART初始化函数
*  使用说明    放在主函数的初始化界面中
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月10日
*  备    注：RX和TX引脚不能插反了，不然收不到数据
*************************************************************************/
void AsclinAsc_init(void)
{
	IfxCpu_disableInterrupts();	// disable interrupts

    /* create module config */
    IfxAsclin_Asc_Config ascConfig;   //定义一个ascConfig的结构体，用于表示asc设置

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
    //ascConfig.frame.idleDelay = IfxAsclin_IdleDelay_0;		// 字节与字节间不插入暂停

    // ISR priorities and interrupt target
    ascConfig.interrupt.txPriority = IFX_INTPRIO_ASCLIN3_TX;
    ascConfig.interrupt.rxPriority = IFX_INTPRIO_ASCLIN3_RX;
    ascConfig.interrupt.erPriority = IFX_INTPRIO_ASCLIN3_ER;
    ascConfig.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    ascConfig.txBuffer = &ascTxBuffer;
    ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConfig.rxBuffer = &ascRxBuffer;
    ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;


    /* pin configuration */ //设置引脚
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
*  函数名称：Asclin_Send_Data
*  功能说明：UART发送函数
*  使用说明
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月10日
*  备    注：数据格式为0x00到0xFF，若要发送字符，请自行转化为ascII码
*************************************************************************/
void Asclin_Send_Data(uint8 Asclin_Send_Data)
{
	IfxAsclin_Asc_blockingWrite(&asc,Asclin_Send_Data);

}

/*************************************************************************
*  函数名称：Asclin_Read_Data
*  功能说明：UART接收函数
*  使用说明    放在主函数的初始化界面中
*  参数说明：无
*  函数返回：返回读取到的值
*  修改时间：2020年7月10日
*  备    注：数据格式为0x00到0xFF，若要发送字符，请自行转化为ascII码
*************************************************************************/
uint8 Asclin_Read_Data(void)
{
	if(IfxAsclin_Asc_getReadCount(&asc) >0)
	{
		return IfxAsclin_Asc_blockingRead(&asc);
	}

	return 0;

}

/* uint8 *value------------------要发送的数组指针
 * uint8 count-------------------数组内元素个数
 * */

/*************************************************************************
*  函数名称：Asclin_Demo_WriteData
*  功能说明：UART中断发送函数
*  使用说明
*  参数说明：无
*  函数返回：无
*  修改时间：2020年7月10日
*  备    注：
*************************************************************************/
boolean Asclin_Demo_WriteData(uint8 *value,uint8 count)
{

	return IfxAsclin_Asc_write(&asc,value,&count,TIME_INFINITE);
}


// 发送字符
void UART_Send_Char(uint8 c)
{
	IfxAsclin_Asc_blockingWrite(&asc, c);
}

// 发送字符串
void UART_Send_Str(uint8 *str)
{
	while(*str != '\0') // or *str
	{
		UART_Send_Char(*str++);
	}
}

// 发送字符数组, 可变长度
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

	if(IfxAsclin_Asc_getReadCount(&asc) > 0)	// 判断接收缓冲区是否有数据
	{
		u_data.data = IfxAsclin_Asc_blockingRead(&asc);
		u_data.flag = TRUE;
		return u_data;
	}

	return u_data;
}

//// 有问题
//uint8 UART_Read_Data(uint8 *data, uint8 len)
//{
//	uint8 count = len;
//
//	if(IfxAsclin_Asc_getReadCount(&asc) < len)	// 判断接收缓冲区是否有数据
//	{
//		return 0;			// 长度不够退出函数
//	}
//
//	// 数据长度达到目标长度要求，接收数据  bug 函数
//	IfxAsclin_Asc_read(&asc, data, &count, TIME_NULL);
//	return 1;
//}

// 接收SBUS数据帧：帧头0x0F-通道值-FLAG-XOR
void Rx_Callback(void)
{
	uart_data u_data;
	uint8 i;

	u_data = UART_Read_Char();
	if(u_data.flag == TRUE)
	{
		SBUS_Buffer[Rx_Count] = u_data.data;
	}

	// 判断帧头，不对退出接收
	if(Rx_Count == 0 && SBUS_Buffer[Rx_Count] != 0x0F) return ;

	Rx_Count++;

	if(SBUS_Buffer[0] == 0x0F && Rx_Count == 35)
	{
		//UART_Send_Data(SBUS_Buffer, 35);

//		for(i = 0; i < 35; i++)		// 因为每接受完一帧后，就对SBUS_Buffer全置零， 所以即便Asclin_Read_Data()读到0x00的数据SBUS_Buffer也能存下来
//		{
//			SBUS_Buffer[i] = 0;
//		}
		IfxPort_togglePin(&MODULE_P33, 8);
		Rx_Count = 0;
	}

}



