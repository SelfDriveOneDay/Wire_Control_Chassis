/*
 * Asclin_app.h
 *
 *  Created on: 2018Äê1ÔÂ2ÈÕ
 *      Author: TEC
 */

#ifndef _ASC_APP_H
#define _ASC_APP_H

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Asclin/Asc/IfxAsclin_Asc.h>
#include <Ifx_Types.h>
#include "Configuration.h"
#include "Bsp/Bsp.h"
#include "ConfigurationIsr.h"
#include "Cpu/Irq/IfxCpu_Irq.h"
#include "Port/Std/IfxPort.h"
#include <Stm/Std/IfxStm.h>
#include <Src/Std/IfxSrc.h>
#include "Cpu0_Main.h"
#include "Cpu/Irq/IfxCpu_Irq.h"

#define IFX_INTPRIO_ASCLIN3_TX  19
#define IFX_INTPRIO_ASCLIN3_RX  15
#define IFX_INTPRIO_ASCLIN3_ER  23

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
typedef struct
{
	uint8 data;
	boolean flag;
} uart_data;



/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
IFX_EXTERN void AsclinAsc_init(void);
IFX_EXTERN void Asclin_Send_Data(uint8 Asclin_Send_Data);
IFX_EXTERN uint8 Asclin_Read_Data(void);
boolean Asclin_Demo_WriteData(uint8 *value,uint8 count);

void UART_Send_Char(uint8 c);
void UART_Send_Str(uint8 *str);
void UART_Send_Data(uint8 *data, uint8 len);
uart_data UART_Read_Char(void);
//uint8 UART_Read_Data(uint8 *data, uint8 len);

void Rx_Callback(void);
void asclin3TxISR(void);
void asclin3RxISR(void);
void asclin3ErISR(void);

#endif /* 0_APPSW_TRICORE_MAIN_ASCLIN_APP_H_ */
