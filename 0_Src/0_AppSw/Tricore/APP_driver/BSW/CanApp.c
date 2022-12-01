/*
 * CanApp.c
 *
 *  Created on: 2021��9��13��
 *      Author: Administrator
 */

#include "CanApp.h"
#include "ASW/CanbusTask.h"
#include"IfxCan_reg.h"
#include"IfxStm.h"

// CAN handle
IfxMultican_Can can;
// Nodes handles
IfxMultican_Can_Node canSrcNode;
// Message Object handles

// Can Message Type
// std_1~std-12 used for control 3 motors
IfxMultican_Can_MsgObj canSrcMsgObj_Std_1, canSrcMsgObj_Std_2, canSrcMsgObj_Std_3, canSrcMsgObj_Std_4, canSrcMsgObj_Std_5, canSrcMsgObj_Std_6, canSrcMsgObj_Std_7, canSrcMsgObj_Std_8, canSrcMsgObj_Std_9;
IfxMultican_Can_MsgObj canSrcMsgObj_Std_10, canSrcMsgObj_Std_11, canSrcMsgObj_Std_12;
// std_13-std_16 used for send message for PC
IfxMultican_Can_MsgObj canSrcMsgObj_Std_13, canSrcMsgObj_Std_14, canSrcMsgObj_Std_15, canSrcMsgObj_Std_16;

// Զ��֡Ŀǰû���õ�
IfxMultican_Can_MsgObj canSrcMsgObj_Rtr_1, canSrcMsgObj_Rtr_2, canSrcMsgObj_Rtr_3, canSrcMsgObj_Rtr_4;

// ���ձ��Ľṹ�壬��������������ID�ı���
IfxMultican_Can_MsgObj canSrcMsgObj_Rec, canSrcMsgObj_Ext,canSrcMsgObj_Err;


//�ڵ�0�����ж�
IFX_INTERRUPT(MultiCAN_TxISR, 0, INTPRIO_MultiCAN_TxISR)
{


}
//�ڵ�0�����ж�
IFX_INTERRUPT(MultiCAN_RxISR, 0, INTPRIO_MultiCAN_RxISR)
{

	CAN_Message Can_Rx_Msg;

	// save msg each interrupt
	CAN0_Receive_Msg(&canSrcMsgObj_Rec, &Can_Rx_Msg);

	// ���յ�����������͵�CAN����
	CAN0Rx_Respond_Process(&Can_Rx_Msg);
	// ����PC�˷��͵�CAN����
	CAN0Rx_Process_For_PC(&Can_Rx_Msg);

	IfxPort_togglePin(&MODULE_P33, 10);			//LED1
}
 /******************************************************************************
 �������ƣ� CAN_SendSingle()
 �������ܣ� CAN����   �������κ�һ�����ͱ���
 ��ڲ����� message�����͵�����
           msgObj�����͵ı���
           IDnumber��id
 ���ڲ�������
 *******************************************************************************/
void CanApp_Init(void)
{
/***************************CANģ���ʼ��************************************/
	IfxMultican_Can_Config canConfig;                           //����can config
	IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);  //��ʼ��ģ������
	canConfig.nodePointer[0].priority = INTPRIO_MultiCAN_RxISR;    //�ж����ȼ�
	canConfig.nodePointer[0].typeOfService = IfxSrc_Tos_cpu0;	//�жϷ����ṩ��

	canConfig.nodePointer[1].priority = INTPRIO_MultiCAN_TxISR;    //�ж����ȼ�
	canConfig.nodePointer[1].typeOfService = IfxSrc_Tos_cpu0;	//�жϷ����ṩ��

	IfxMultican_Can_initModule(&can, &canConfig);				//ģ���ʼ��

	/***************************CAN�ڵ�0��ʼ��********************************/

	IfxMultican_Can_NodeConfig canNodeConfig;					//�����ڵ�����
	IfxMultican_Can_Node_initConfig(&canNodeConfig, &can);		//�ڵ��ʼ������           ֱ�Ӹ��Ƽ���
	canNodeConfig.baudrate = 500000;
	canNodeConfig.nodeId = IfxMultican_NodeId_0;
	canNodeConfig.rxPin = &IfxMultican_RXD0B_P20_7_IN;
	canNodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
	canNodeConfig.txPin = &IfxMultican_TXD0_P20_8_OUT;
	canNodeConfig.txPinMode = IfxPort_OutputMode_pushPull;
	canNodeConfig.samplePoint = 8000;
	canNodeConfig.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed4;
	IfxMultican_Can_Node_init(&canSrcNode, &canNodeConfig);

	//�ڵ�0���ͱ�׼֡��ʼ��
	{
		IfxMultican_Can_MsgObjConfig   canMsgObjConfig;
		IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &canSrcNode);
//		canMsgObjConfig.messageId = 0x100; // 'id' is defined globally
		canMsgObjConfig.acceptanceMask = 0x7FFFFFFFUL;
		canMsgObjConfig.frame = IfxMultican_Frame_transmit;		// data frame
		canMsgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
		canMsgObjConfig.control.extendedFrame = FALSE;			// std frame
		canMsgObjConfig.control.matchingId = TRUE;
		canMsgObjConfig.txInterrupt.enabled = FALSE;
		canMsgObjConfig.txInterrupt.srcId=IfxMultican_SrcId_1;

		canMsgObjConfig.msgObjId = 0;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_1, &canMsgObjConfig);	 	//��׼CAN���ĳ�ʼ��
		canMsgObjConfig.msgObjId = 1;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_2, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 2;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_3, &canMsgObjConfig);

		canMsgObjConfig.msgObjId = 3;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_4, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 9;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_5, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 10;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_6, &canMsgObjConfig);

		canMsgObjConfig.msgObjId = 11;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_7, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 12;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_8, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 13;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_9, &canMsgObjConfig);

		canMsgObjConfig.msgObjId = 14;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_10, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 15;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_11, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 16;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_12, &canMsgObjConfig);

		// 4*canMsgObj For PC
		canMsgObjConfig.msgObjId = 17;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_13, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 18;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_14, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 19;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_15, &canMsgObjConfig);
		canMsgObjConfig.msgObjId = 20;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Std_16, &canMsgObjConfig);

		IfxCpu_Irq_installInterruptHandler(&MultiCAN_TxISR, INTPRIO_MultiCAN_TxISR);
	}

	//�ڵ�0����Զ��֡��ʼ��
	{
		IfxMultican_Can_MsgObjConfig canMsgObjConfig;
		IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &canSrcNode);
//		canMsgObjConfig.messageId = 0x100; // 'id' is defined globally
		canMsgObjConfig.acceptanceMask = 0x7FFFFFFFUL;
		canMsgObjConfig.frame = IfxMultican_Frame_remoteRequest;		// rtr frame
		canMsgObjConfig.control.messageLen = IfxMultican_DataLengthCode_0;
		canMsgObjConfig.control.extendedFrame = FALSE;					// std frame
		canMsgObjConfig.control.matchingId = TRUE;
		canMsgObjConfig.txInterrupt.enabled = FALSE;
		canMsgObjConfig.txInterrupt.srcId = IfxMultican_SrcId_2;

		canMsgObjConfig.msgObjId = 4;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Rtr_1, &canMsgObjConfig);	 	//Rtr CAN���ĳ�ʼ��
		canMsgObjConfig.msgObjId = 5;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Rtr_2, &canMsgObjConfig);	 	//Rtr CAN���ĳ�ʼ��
		canMsgObjConfig.msgObjId = 6;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Rtr_3, &canMsgObjConfig);	 	//Rtr CAN���ĳ�ʼ��
		canMsgObjConfig.msgObjId = 7;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Rtr_4, &canMsgObjConfig);	 	//Rtr CAN���ĳ�ʼ��

		IfxCpu_Irq_installInterruptHandler(&MultiCAN_TxISR, INTPRIO_MultiCAN_TxISR);
	}

	//�ڵ�0���ձ��ĳ�ʼ��
	{
		IfxMultican_Can_MsgObjConfig canMsgObjConfig;
		IfxMultican_Can_MsgObj_initConfig(&canMsgObjConfig, &canSrcNode);
		canMsgObjConfig.msgObjId = 8;
//		canMsgObjConfig.messageId = 0x202; // 'id' is defined globally
		canMsgObjConfig.acceptanceMask = 0;		// �������б���
		canMsgObjConfig.frame = IfxMultican_Frame_receive;
		canMsgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
		canMsgObjConfig.control.extendedFrame = FALSE;
		canMsgObjConfig.control.matchingId = TRUE;
		canMsgObjConfig.rxInterrupt.enabled = TRUE;
		canMsgObjConfig.rxInterrupt.srcId = IfxMultican_SrcId_0;
		IfxMultican_Can_MsgObj_init(&canSrcMsgObj_Rec, &canMsgObjConfig);	 //���ĳ�ʼ��
		IfxCpu_Irq_installInterruptHandler(&MultiCAN_RxISR, INTPRIO_MultiCAN_RxISR);
	}

	IfxCpu_enableInterrupts();
}

/******************************************************************************
�������ƣ� CAN_SendSingle()
�������ܣ� CAN����   �������κ�һ�����ͱ���
��ڲ����� message�����͵�����
          msgObj�����͵ı���
          IDnumber��id
���ڲ�������
*******************************************************************************/
void CAN_SendSingle(CAN_Buffer_format *message,IfxMultican_Can_MsgObj *msgObj, uint32 IDnumber)
{
 IfxMultican_Message txMsg;
 txMsg.id        = IDnumber;
 txMsg.data[0]     = message->W[0];
 txMsg.data[1]     = message->W[1];
 txMsg.lengthCode  = IfxMultican_DataLengthCode_8;
 // Transmit Data
 while( IfxMultican_Can_MsgObj_sendMessage(msgObj, &txMsg) == IfxMultican_Status_notSentBusy )  //���ķ���
 {
    break;
 }

}
/******************************************************************************
�������ƣ� CAN_ReceiveSingle()
�������ܣ� CAN����   �������κ�һ�����ձ���
��ڲ�����IDnumber��id
          msgObj�����յı���
          message�������յ�������
���ڲ�������
*******************************************************************************/
void CAN_ReceiveSingle(uint32 IDnumber, IfxMultican_Can_MsgObj *msgObj, CAN_Buffer_format *message)
{
	uint32 wait_count = 1000;
  IfxMultican_Message rxMsg;;
  IfxMultican_Message_init(&rxMsg, 0xdead, 0xdeadbeef, 0xdeadbeef, IfxMultican_DataLengthCode_8); // start with invalid values
  // wait until Multican received a new message
  while( !IfxMultican_Can_MsgObj_isRxPending(msgObj) && wait_count-->0);
  if(wait_count == 0)  return;
  // read message
 IfxMultican_Status readStatus = IfxMultican_Can_MsgObj_readMessage(msgObj, &rxMsg);
 // if new data is been received report an error
	if((readStatus & IfxMultican_Status_newData) && (rxMsg.id == IDnumber))
  {
   message->W[0] = rxMsg.data[0];
   message->W[1] = rxMsg.data[1];
  }
}

// receive all can msg
//IfxMultican_Message rxMsg;
void CAN0_Receive_Msg(IfxMultican_Can_MsgObj *msgObj, CAN_Message *message)
{
	uint32 wait_count = 1000;
	IfxMultican_Message rxMsg;

	// init rxmsg at will
	IfxMultican_Message_init(&rxMsg, 0xdead, 0xdeadbeef, 0xdeadbeef, IfxMultican_DataLengthCode_8);

	// wait for new msg
	while(!IfxMultican_Can_MsgObj_isRxPending(msgObj) && wait_count-->0);
	if(wait_count == 0)	return ;  // exit

	// read message
	IfxMultican_Status readStatus = IfxMultican_Can_MsgObj_readMessage(msgObj, &rxMsg);

	// if new data receive all id msg
	if((readStatus == IfxMultican_Status_newData))
	{
		message->id = rxMsg.id;
		message->can_buffer.W[0] = rxMsg.data[0];
		message->can_buffer.W[1] = rxMsg.data[1];

		message->fastBitRate = FALSE;							// can ignore
		message->lengthCode = IfxMultican_DataLengthCode_8;		// can ignore
	}
}

// send you CAN_Message
void CAN0_Send_Msg(IfxMultican_Can_MsgObj *msgObj, CAN_Message *message)
{
	IfxMultican_Message txMsg;

	txMsg.id = message->id;
	txMsg.data[0] = message->can_buffer.W[0];
	txMsg.data[1] = message->can_buffer.W[1];
	txMsg.lengthCode = message->lengthCode;

	// Tx data
	while(IfxMultican_Can_MsgObj_sendMessage(msgObj, &txMsg) == IfxMultican_Status_notSentBusy)
	{
		break;
	}
}






