/*
 * CanApp.h
 *
 *  Created on: 2021Äê9ÔÂ13ÈÕ
 *      Author: Administrator
 */

#ifndef _CANAPP_H_
#define _CANAPP_H_

#include "Multican/Can/IfxMultican_Can.h"

#define INTPRIO_MultiCAN_RxISR		2
#define INTPRIO_MultiCAN_TxISR  	3

typedef union
{
	uint32 Integer;
	float  Decimal;
} IEEE754;

//typedef union
//{
//	sint16 S;
//	uint16 U;
//} data16;
//
//typedef union
//{
//	sint32 	S;
//	uint32 	U;
//	float32 D;
//} data32;

typedef union
{
  uint64 M;
  uint8  B[8];
  uint16 H[4];
  uint32 W[2];
} CAN_Buffer_format;

typedef struct
{
	uint32							id;
	IfxMultican_DataLengthCode 		lengthCode;
	CAN_Buffer_format             	can_buffer;
	boolean							fastBitRate;
} CAN_Message;

void CanApp_Init(void);
void CAN_SendSingle(CAN_Buffer_format *message,IfxMultican_Can_MsgObj *msgObj,uint32 IDnumber);
void CAN_ReceiveSingle(uint32 IDnumber, IfxMultican_Can_MsgObj *msgObj,CAN_Buffer_format *message);

void CAN0_Receive_Msg(IfxMultican_Can_MsgObj *msgObj, CAN_Message *message);
void CAN0_Send_Msg(IfxMultican_Can_MsgObj *msgObj, CAN_Message *message);
void Can_Msg_Copy(CAN_Message *Self_msg, IfxMultican_Message *Ifx_msg);

void MultiCAN_TxISR(void);
void MultiCAN_RxISR(void);

#endif

