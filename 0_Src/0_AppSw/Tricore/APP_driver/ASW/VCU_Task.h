/*
 * VCU_Task.h
 *
 *  Created on: 2021Äê9ÔÂ23ÈÕ
 *      Author: Administrator
 */

#ifndef _VCU_TASK_H_
#define _VCU_TASK_H_

#include "ASW/CanbusTask.h"
#include "ASW/RemoteTask.h"
#include "ASW/ComputerTask.h"
#include "ASW/ControlTask.h"
#include "BSW/CanApp.h"
#include "BSW/AscApp.h"
#include "ASW/Chassis_Cfg.h"


extern IfxMultican_Can_MsgObj canSrcMsgObj_Std_1, canSrcMsgObj_Std_2, canSrcMsgObj_Std_3, canSrcMsgObj_Std_4, canSrcMsgObj_Std_5, canSrcMsgObj_Std_6, canSrcMsgObj_Std_7, canSrcMsgObj_Std_8, canSrcMsgObj_Std_9, canSrcMsgObj_Std_10, canSrcMsgObj_Std_11, canSrcMsgObj_Std_12;
extern IfxMultican_Can_MsgObj canSrcMsgObj_Std_13, canSrcMsgObj_Std_14, canSrcMsgObj_Std_15, canSrcMsgObj_Std_16;
extern IfxMultican_Can_MsgObj canSrcMsgObj_Rtr_1, canSrcMsgObj_Rtr_2, canSrcMsgObj_Rtr_3, canSrcMsgObj_Rtr_4;
extern IfxMultican_Can_MsgObj canSrcMsgObj_Rec;
extern IfxAsclin_Asc asc;
extern t8s T8S;
extern chassis Chassis;
extern motor Motor[3];
extern chassis_control_from_pc ChassisCtrlFromPC;
extern chassis_states_send_pc ChassisStatesSendPC;


#endif

