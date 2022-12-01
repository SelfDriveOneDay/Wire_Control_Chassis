/*
 * CanbusTask.c
 *
 *  Created on: 2021年9月13日
 *      Author: cv
 *      Describe: Tx/Rx CAN报文
 */
#include "CanbusTask.h"
#include "VCU_Task.h"

// 未考虑初始化
motor Motor[3];

// 接收电机驱动器发送的应答报文处理，数据解析
void CAN0Rx_Respond_Process(CAN_Message *msg)
{
	uint16 rxID = msg->id & 0x7FF0;			// 获取CAN报文的接收ID,取高位
	uint8  nodeID = msg->id - CAN_RX_ID;	// 获取CAN报文的节点ID
	uint8  data0 = msg->can_buffer.B[0];	// 获取字节0的状态码
	uint16 param_index = msg->can_buffer.B[1] + (msg->can_buffer.B[2] << 8);	// 获取字典索引号

	if(rxID != CAN_RX_ID) 	return;		// 如果rxID不是电机驱动器的应答ID，则退出函数
	// 接收电机驱动器的应答报文，应答报文的第零个字节表示6种状态
	// 如果MCU发送CAN请求数据的报文则第零个字节为请求数据的长度或者传送中止的代码
	// 如果MCU发送CAN控制的报文则第零个字节为传送成功或传送中止的代码
	//	if(data0 == TX_SUCCESS) return;		// TC275每发一条控制报文,发送成功并成功执行，驱动器回复成功，其数据域无效
	if(data0 == TX_SUSPEND) return;		// 传送中止，说明发送的报文出问题，可以报警。

	switch(nodeID)
	{
		case STEERING_MOTOR_CAN_ID:
			if(data0 == TX_SUCCESS)		// 判断MCU发送的CAN控制报文电机控制器是否正确接收到
			{
				motor_control_param_callback(SM, param_index);	//
			}
			else						// 接收电机控制器反馈的数据：数据长度为1-4字节
			{
				get_motor_state_param(SM, param_index, msg);	// 获取电机的实时的参数，转速，位置
			}
			break;
		case LEFT_DRIVE_MOTOR_CAN_ID:
			if(data0 == TX_SUCCESS)
			{
				motor_control_param_callback(LM, param_index);
			}
			else
			{
				get_motor_state_param(LM, param_index, msg);
			}
			break;
		case RIGHT_DRIVE_MOTOR_CAN_ID:
			if(data0 == TX_SUCCESS)
			{
				motor_control_param_callback(RM, param_index);
			}
			else
			{
				get_motor_state_param(RM, param_index, msg);
			}
			break;
	}
}

// 如果设置成功，相应标志位置位, 如果设置成功驱动器可以接受下一条报文, 由此标志位判断，可以不用进行死延时！！！！
void motor_control_param_callback(motor_index mIndex, uint16 param_index)
{
	// 判断电机常用的几个控制量是否设置成功
	switch(param_index)
	{
		case MOTOR_CONTROL_MODE:
			Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = TRUE;	// 驱动器应答报文表明-电机控制模式设置成功
			Motor[mIndex].MCtrl_Mode_Real = Motor[mIndex].MCtrl_Mode;
			break;
		case MOTOR_CONTROL_QUANTITY:	// 这里的控制量指的是目标速度
			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;	// 驱动器应答报文表明-电机控制量设置成功
			Motor[mIndex].Aim_Vel_Real = Motor[mIndex].Aim_Vel;
			break;
		case AIM_POS:
			Motor[mIndex].Ctrl_State.Aim_Pos_Fg = TRUE;
			Motor[mIndex].Aim_Pos_Real = Motor[mIndex].Aim_Pos;
			break;
		case CLOSE_LOOP_ACCEL:
			Motor[mIndex].Ctrl_State.CL_Accel_Fg = TRUE;
			Motor[mIndex].Aim_Accel_Real = Motor[mIndex].Aim_Accel;
			break;
		case CLOSE_LOOP_DECEL:
			Motor[mIndex].Ctrl_State.CL_Decel_Fg = TRUE;
			Motor[mIndex].Aim_Decel_Real = Motor[mIndex].Aim_Decel;
			break;
	}
}

// 从接受的CAN报文中分离出数据
void get_motor_state_param(motor_index mIndex, uint16 param_index, CAN_Message *msg)
{
	switch(param_index)
	{
		case POS_COUNT:			// 电机的绝对位置 改成相对位置
			Motor[mIndex].Pos_Est.U = msg->can_buffer.W[1];		// S32
			break;
		case POS_CONTROL_FINISH_STATE:
			Motor[mIndex].Pos_State = msg->can_buffer.B[4];		// U8
			break;
		case VEL_ESTIMATE:
			Motor[mIndex].Vel_Est = msg->can_buffer.W[1];		// U32
			break;
		case BUS_CURRENT:
			Motor[mIndex].Bus_Cur = msg->can_buffer.H[2];		// U16
			break;
		case BUS_VOLTAGE:
			Motor[mIndex].Bus_Volt = msg->can_buffer.H[2];		// U16
			break;
		case POWER_TUBE_TEMPERATURE:
			Motor[mIndex].Tube_Temp = msg->can_buffer.H[2];		// U16
			break;
		case MOTOR_ERROR_STATE:
			Motor[mIndex].Error_State = msg->can_buffer.B[4];	// U8
			break;
	}
}

// 发送请求数据的报文， 获取电机的状态，采用任务调度表的形式，对于实时性要求高的可以多发
volatile uint16 cntCanReqSch = 1;
void CAN0Tx_Request_Process(void)
{
	CAN_Message txmsg;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	switch(cntCanReqSch)
	{
		case 1: // 改成相对位置
			fill_can_msg_buff(&txmsg, RX_DATA, POS_COUNT, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 2:
			fill_can_msg_buff(&txmsg, RX_DATA, POS_CONTROL_FINISH_STATE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 3:
			fill_can_msg_buff(&txmsg, RX_DATA, VEL_ESTIMATE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 4:
			fill_can_msg_buff(&txmsg, RX_DATA, BUS_CURRENT, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 5:
			fill_can_msg_buff(&txmsg, RX_DATA, BUS_VOLTAGE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 6:
			fill_can_msg_buff(&txmsg, RX_DATA, POS_CONTROL_FINISH_STATE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 7:
			fill_can_msg_buff(&txmsg, RX_DATA, POS_COUNT, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 8:
			fill_can_msg_buff(&txmsg, RX_DATA, MOTOR_ERROR_STATE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 9:
			fill_can_msg_buff(&txmsg, RX_DATA, VEL_ESTIMATE, SINDEX_0, 0);
			cntCanReqSch++;
			break;
		case 10:
			fill_can_msg_buff(&txmsg, RX_DATA, POWER_TUBE_TEMPERATURE, SINDEX_0, 0);
			cntCanReqSch = 1;
			break;
		// 在加一个case出问题
	}
	send_request_msg(&txmsg);
}

// 对于同一电机状态地址,给3个电机驱动器发数据请求报文
void send_request_msg(CAN_Message *txmsg)
{
	// sw = 3,对应的报文结构体是专门给向驱动器请求数据用的不能再别处使用
	if(Motor[SM].Ctrl_State.Aim_Pos_Fg == TRUE)
	{
		switch_canid_send(SM, txmsg, 3);	// 注意没有避免向同一个驱动器发送两条报文
	}
	if(Motor[LM].Ctrl_State.Ctrl_Qty_Fg == TRUE)
	{
		switch_canid_send(LM, txmsg, 3);
	}
	if(Motor[RM].Ctrl_State.Ctrl_Qty_Fg == TRUE)
	{
		switch_canid_send(RM, txmsg, 3);
	}

}

// 给同一个电机驱动器发送CAN命令不能，连着发，需要间隔一定时间，延时在100-10ms之间
// 初始化电机的参数：速度、加速度单位、最大速度，位置类型
void motor_params_init(void)
{
	// 设置三个电机速度、加速度单位
	set_motor_uint(RPM, RAD_S2);

	// 设置三个电机的最大转速
	set_max_vel(STEER_MOTOR_MAX_VEL, DRIVE_MOTOR_MAX_VEL, DRIVE_MOTOR_MAX_VEL);

	// 初始化电机的控制模式
	set_control_mode(SM, CLOSE_LOOP_POS);
	set_control_mode(LM, CLOSE_LOOP_VEL);
	set_control_mode(RM, CLOSE_LOOP_VEL);

	// 单片机执行速度很快，所以为了保证给同以控制器发送报文间隔一定时间
	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, 死延时

	// 设置三个电机的位置模式
	set_pos_mode(SM, ABSOLUTE_POS_MODE);	// 绝对位置是以初始位置，即0位置为基准的
	set_pos_mode(LM, RELATIVE_POS_MODE);	// 相对位置示意当前位置，即当前的位置计数值开始的
	set_pos_mode(RM, RELATIVE_POS_MODE);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, 死延时

	// 位置计数清零
	clear_pos_count(SM);
	clear_pos_count(LM);
	clear_pos_count(RM);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, 死延时

	// 设置加速度单位rad/s2, 注意不用set_aim_accel()
	accel_init(SM, STEER_MOTOR_MAX_ACCEL, STEER_MOTOR_MAX_ACCEL);
	accel_init(LM, DRIVE_MOTOR_MAX_ACCEL, DRIVE_MOTOR_MAX_ACCEL);
	accel_init(RM, DRIVE_MOTOR_MAX_ACCEL, DRIVE_MOTOR_MAX_ACCEL);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, 死延时

}

// 设置电机的控制模式
void set_control_mode(motor_index mIndex, mctrl_mode mcMode)
{
	CAN_Message txmsg;

	// 保存配置(应该在接收到驱动器的发送的设置成功的反馈报文后保存配置)
	Motor[mIndex].MCtrl_Mode = mcMode;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 发送的报文的字节数一定要和驱动器的CAN协议匹配，否则不会生效
	fill_can_msg_buff(&txmsg, TX_ONE_BYTE, MOTOR_CONTROL_MODE, SINDEX_0, mcMode);

	if(Motor[mIndex].MCtrl_Mode_Real != mcMode)
	{
		switch_canid_send(mIndex, &txmsg, 0);
		Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = FALSE;	// 清除标志
	}
	else
		Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = TRUE;
}

// 注意此函数会连续发两条报文，电机驱动器处理不过来需要一定延时
// 电机转到给定的位置需要一定时间，时间与电机的转速，负载有关，想调快需要把速度和加速度设高点, 同时要设定最大转速，参数还可以吧加速度加上
void set_aim_pos(motor_index mIndex, sint32 aim_pos, sint16 aim_vel)
{
	CAN_Message txmsg1, txmsg2;

	// 保存参数，aim_pos可以设置为减速器输出轴的转角，速度有正负
	Motor[mIndex].Aim_Pos.S = aim_pos;
	Motor[mIndex].Aim_Vel.S = aim_vel;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 填充待发送报文8个字节的数据域
	fill_can_msg_buff(&txmsg1, TX_FOUR_BYTE, AIM_POS, SINDEX_0, Motor[mIndex].Aim_Pos.U);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Vel.U);

	// 如果电机实际的目标速度不等于设定值，则发送控制目标速度的CAN报文，若等于则对应标志位置一
	if(Motor[mIndex].Aim_Vel_Real.S != Motor[mIndex].Aim_Vel.S)
	{
		switch_canid_send(mIndex, &txmsg2, 0);			// 位置控制时先发目标速度
		// 标志位清零便于下次使用
		Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = FALSE;
	}
	else
		Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;	// 如果的电机实际目标位置为设定值，可以发下一个CAN报文

//	IfxStm_waitTicks(&MODULE_STM1,50000);	// 0.5ms

	// 当目标速度设置成功(得到驱动器发送的传送成功的数据)在发目标位置
	if(Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg == TRUE)
	{
		// 当前电机的实际的目标位置不等于设定的目标位置，则发送目标位置
		if(Motor[mIndex].Aim_Pos_Real.S != Motor[mIndex].Aim_Pos.S)
		{
			switch_canid_send(mIndex, &txmsg1, 1);
			Motor[mIndex].Ctrl_State.Aim_Pos_Fg = FALSE;	// 没有接着用这个标志自己清零
		}
		else
			Motor[mIndex].Ctrl_State.Aim_Pos_Fg = TRUE;		// 位置的标志位怎么清零
	}

}

// 注意此函数会连续发两条报文，电机驱动器处理不过来需要一定延时
void set_aim_vel(motor_index mIndex, sint16 aim_vel, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg;

	Motor[mIndex].Aim_Vel.S = aim_vel;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 填充can报文，电机控制量可以选择发送S16两个字节和S32四个字节，这里用S16两个字节，足够表示转速(-3万-3万之间)
	fill_can_msg_buff(&txmsg, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Vel.U);

	// 设置目标加减速度
	set_aim_accel(mIndex, aim_accel, aim_decel);

	// 设置目标速度
	if(Motor[mIndex].Ctrl_State.CL_Decel_Fg == TRUE)
	{
		switch_canid_send(mIndex, &txmsg, 0);
//		if(Motor[mIndex].Aim_Vel_Real.S != Motor[mIndex].Aim_Vel.S)
//		{
//			switch_canid_send(mIndex, &txmsg, 0);
//			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = FALSE;	// 没有接着用这个标志自己清零
//		}
//		else
//			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;
	}
}

// 艾思控的驱动器没有扭矩闭环控制的功能，其力矩控制只是设定最大电流值，主要用于堵转保持力矩，如果负载消失，电机就会飞转
void set_aim_torque(motor_index mIndex, float cur)
{
	CAN_Message txmsg;

	// 保存目标电流
	Motor[mIndex].Aim_Cur.S = cur * CURRENT_K;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 填充发送设定目标扭矩的CAN报文
	fill_can_msg_buff(&txmsg, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Cur.U);

	// 选择canID发送
	switch_canid_send(mIndex, &txmsg, 0);
}

// 注意此函数会连续发两条报文，电机驱动器处理不过来需要一定延时
void set_aim_accel(motor_index mIndex, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg1, txmsg2;

	// 设置目标加减速度
	Motor[mIndex].Aim_Accel = aim_accel * REVERSE_K;
	Motor[mIndex].Aim_Decel = aim_decel * REVERSE_K;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 填充发送设定目标加减速度的CAN报文
	fill_can_msg_buff(&txmsg1, TX_TWO_BYTE, CLOSE_LOOP_ACCEL, SINDEX_0, Motor[mIndex].Aim_Accel);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, CLOSE_LOOP_DECEL, SINDEX_0, Motor[mIndex].Aim_Decel);

	if(Motor[mIndex].Aim_Accel_Real != Motor[mIndex].Aim_Accel)
	{
		switch_canid_send(mIndex, &txmsg1, 1);
		Motor[mIndex].Ctrl_State.CL_Accel_Fg = FALSE;
	}
	else
		Motor[mIndex].Ctrl_State.CL_Accel_Fg = TRUE;

//	IfxStm_waitTicks(&MODULE_STM1, 100000);	//1ms, 死延时

	if(Motor[mIndex].Ctrl_State.CL_Accel_Fg == TRUE)
	{
		if(Motor[mIndex].Aim_Decel_Real != Motor[mIndex].Aim_Decel)
		{
			switch_canid_send(mIndex, &txmsg2, 2);
			Motor[mIndex].Ctrl_State.CL_Decel_Fg = FALSE;
		}
		else
			Motor[mIndex].Ctrl_State.CL_Decel_Fg = TRUE;
	}
}

// 设置加速度初始化,只调用一次
void accel_init(motor_index mIndex, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg1, txmsg2;

	// 设置目标加减速度
	Motor[mIndex].Aim_Accel = aim_accel * REVERSE_K;
	Motor[mIndex].Aim_Decel = aim_decel * REVERSE_K;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 填充发送设定目标加减速度的CAN报文
	fill_can_msg_buff(&txmsg1, TX_TWO_BYTE, CLOSE_LOOP_ACCEL, SINDEX_0, Motor[mIndex].Aim_Accel);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, CLOSE_LOOP_DECEL, SINDEX_0, Motor[mIndex].Aim_Decel);

	switch_canid_send(mIndex, &txmsg1, 1);	// 连续向同一控制器发两条报文会处理不过来，必须要有延时

	IfxStm_waitTicks(&MODULE_STM1, 1000000);	// 10ms, 死延时

	switch_canid_send(mIndex, &txmsg2, 2);

	IfxStm_waitTicks(&MODULE_STM1, 1000000);	// 10ms, 死延时
}

// 注意此函数会连续发两条报文，电机驱动器处理不过来需要一定延时
// 通过can总线设置电机的速度、加速度单位为转每分，rad/s2, 只用一次
void set_motor_uint(vel_uint vel_u, accel_uint accel_u)
{
	CAN_Message txmsg1, txmsg2;
	uint8 index;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 事先填充好两条CAN报文的数据域
	fill_can_msg_buff(&txmsg1, TX_ONE_BYTE, VEL_UINT, SINDEX_0, vel_u);
	fill_can_msg_buff(&txmsg2, TX_ONE_BYTE, ACCEL_UINT, SINDEX_0, accel_u);

	// 将设定的速度单位和加速度单位保存到Motor, 将数据通过CAN总线发出
	for(index = 0; index < 3; index++)
	{
		Motor[index].Vel_Uint = vel_u;
		Motor[index].Accel_Uint = accel_u;

		switch(index)
		{
			case SM:
				txmsg1.id = CAN_TX_ID + STEERING_MOTOR_CAN_ID;
				txmsg2.id = txmsg1.id;
				break;
			case LM:
				txmsg1.id = CAN_TX_ID + LEFT_DRIVE_MOTOR_CAN_ID;
				txmsg2.id = txmsg1.id;
				break;
			case RM:
				txmsg1.id = CAN_TX_ID + RIGHT_DRIVE_MOTOR_CAN_ID;
				txmsg2.id = txmsg1.id;
				break;
		}
		// 驱动器最高能支持1ms一帧can报文，1ms五帧数据，驱动器处理不了
		CAN0_Send_Msg(&canSrcMsgObj_Std_1, &txmsg1);
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, 死延时
		CAN0_Send_Msg(&canSrcMsgObj_Std_2, &txmsg2);
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, 死延时
	}
}

// 通过CAN总线设置最大速度单位转每分,只用一次
void set_max_vel(uint16 SM_max_vel, uint16 LM_max_vel, uint16 RM_max_vel)
{
	CAN_Message txmsg;
	uint8 index;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	for(index = 0; index < 3; index++)
	{
		switch(index)
		{
			case SM:
				Motor[SM].Max_Vel = SM_max_vel;
				fill_can_msg_buff(&txmsg, TX_FOUR_BYTE, MAX_VEL, SINDEX_0, SM_max_vel);
				txmsg.id = CAN_TX_ID + STEERING_MOTOR_CAN_ID;
				break;
			case LM:
				Motor[LM].Max_Vel = LM_max_vel;
				fill_can_msg_buff(&txmsg, TX_FOUR_BYTE, MAX_VEL, SINDEX_0, LM_max_vel);
				txmsg.id = CAN_TX_ID + LEFT_DRIVE_MOTOR_CAN_ID;
				break;
			case RM:
				Motor[RM].Max_Vel = RM_max_vel;
				fill_can_msg_buff(&txmsg, TX_FOUR_BYTE, MAX_VEL, SINDEX_0, RM_max_vel);
				txmsg.id = CAN_TX_ID + RIGHT_DRIVE_MOTOR_CAN_ID;
				break;
		}
		CAN0_Send_Msg(&canSrcMsgObj_Std_3, &txmsg);
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, 死延时
	}
}

// 设置电机的位置模式 只用一次
void set_pos_mode(motor_index mIndex, pos_mode pMode)
{
	CAN_Message txmsg;

	// 保存配置
	Motor[mIndex].Pos_Mode = pMode;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	fill_can_msg_buff(&txmsg, TX_ONE_BYTE, POS_MODE, SINDEX_0, pMode);

	switch_canid_send(mIndex, &txmsg, 1);
}

// 清除位置计数值（绝对位置） 只用一次
void clear_pos_count(motor_index mIndex)
{
	CAN_Message txmsg;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 数据区对数据写0，位置计数值则为零
	fill_can_msg_buff(&txmsg, TX_FOUR_BYTE, RESET_MOTOR_POS_COUNT, SINDEX_0, 0);

	switch_canid_send(mIndex, &txmsg, 2);
}


// 选择电机对应的ID发送CAN报文
void switch_canid_send(motor_index mIndex, CAN_Message *msg, uint8 sw)
{
	switch(mIndex)
	{
		case SM:
			msg->id = CAN_TX_ID + STEERING_MOTOR_CAN_ID;
			if(sw == 0)
				CAN0_Send_Msg(&canSrcMsgObj_Std_1, msg);
			else if(sw == 1)
				CAN0_Send_Msg(&canSrcMsgObj_Std_2, msg);
			else if(sw == 2)
				CAN0_Send_Msg(&canSrcMsgObj_Std_3, msg);
			else if(sw == 3)
				CAN0_Send_Msg(&canSrcMsgObj_Std_4, msg);
			break;
		case LM:
			msg->id = CAN_TX_ID + LEFT_DRIVE_MOTOR_CAN_ID;
			if(sw == 0)
				CAN0_Send_Msg(&canSrcMsgObj_Std_5, msg);
			else if(sw == 1)
				CAN0_Send_Msg(&canSrcMsgObj_Std_6, msg);
			else if(sw == 2)
				CAN0_Send_Msg(&canSrcMsgObj_Std_7, msg);
			else if(sw == 3)
				CAN0_Send_Msg(&canSrcMsgObj_Std_8, msg);
			break;
		case RM:
			msg->id = CAN_TX_ID + RIGHT_DRIVE_MOTOR_CAN_ID;
			if(sw == 0)
				CAN0_Send_Msg(&canSrcMsgObj_Std_9, msg);
			else if(sw == 1)
				CAN0_Send_Msg(&canSrcMsgObj_Std_10, msg);
			else if(sw == 2)
				CAN0_Send_Msg(&canSrcMsgObj_Std_11, msg);
			else if(sw == 3)
				CAN0_Send_Msg(&canSrcMsgObj_Std_12, msg);
			break;
	}
}

// 接收PC端的控制报文:(测试成功)
void CAN0Rx_Process_For_PC(CAN_Message *msg)
{
	uint16 rxID = msg->id;

	if(rxID != CAN_RXPC_ID)	return;

	// 接收来自PC端CAN报文数据
	// 有符号数据接收(此处做修改, 数据的长度发生变化，增加了一个变量 aim_SM_pose)
	//	ChassisCtrlFromPC.aim_vel.U = msg->can_buffer.H[0];
	//	ChassisCtrlFromPC.aim_accel.U = msg->can_buffer.H[1];
	//	ChassisCtrlFromPC.aim_deltaf.U= msg->can_buffer.H[2];

	// 要特别注意跨字节有符号的数据解析
	// aim_vel占据12位(跨字节，有符号)：H[0]的低12位或者B[1]的低四位和B[0]的所有位  B[1] << 8 + B[0]
	ChassisCtrlFromPC.aim_vel.U = msg->can_buffer.H[0] & 0x0FFF;
	if(ChassisCtrlFromPC.aim_vel.U >= 2048)
	{
		ChassisCtrlFromPC.aim_vel.S = ChassisCtrlFromPC.aim_vel.U - 4096;
	}

	// aim_accel占据10个位(跨字节，有符号)：B[2]的低6位和B[1]的高四位
	ChassisCtrlFromPC.aim_accel.U = ((msg->can_buffer.B[2] & 0x3F) << 4) + ((msg->can_buffer.B[1] & 0xF0) >> 4);
	if(ChassisCtrlFromPC.aim_accel.U >= 512)
	{
		ChassisCtrlFromPC.aim_accel.S = ChassisCtrlFromPC.aim_accel.U - 1024;
	}

	// aim_SM_pose(跨字节，有符号)占据10个位：B[3]所有位和B[2]的高两位
	ChassisCtrlFromPC.aim_SM_pose.U = (msg->can_buffer.B[3] << 2) + ((msg->can_buffer.B[2] & 0xC0) >> 6);
	if(ChassisCtrlFromPC.aim_SM_pose.U >= 512)
	{
		ChassisCtrlFromPC.aim_SM_pose.S = ChassisCtrlFromPC.aim_SM_pose.U - 1024;
	}

	ChassisCtrlFromPC.aim_deltaf.U= msg->can_buffer.H[2];

	// 无符号数据接收
	ChassisCtrlFromPC.aim_steer_vel = msg->can_buffer.H[3] & 0x0FFF;
	ChassisCtrlFromPC.stop_flag = msg->can_buffer.B[7] & 0x30;
	ChassisCtrlFromPC.control_mode = msg->can_buffer.B[7] & 0xC0;
}

// 发送底盘的状态信息给PC
// 发送请求数据的报文， 获取电机的状态，采用任务调度表的形式，对于实时性要求高的可以多发
volatile uint16 cntCanSendPCSch = 1;
void send_chassis_states_2pc(void)
{
	CAN_Message txmsg1, txmsg2, txmsg3, txmsg4;

	// 初始化报文
	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg3, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg4, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// 获取底盘的实时状态数据
	PC_Get_Chassis_States();

	// 填充报文数据
	txmsg1.can_buffer.H[0] = ChassisStatesSendPC.cur_vel.U;
//	txmsg1.can_buffer.H[1] = ChassisStatesSendPC.cur_deltaf.U;
	txmsg1.can_buffer.H[1] = ChassisStatesSendPC.SM_pose_count.U;
	txmsg1.can_buffer.W[1] = ChassisStatesSendPC.odometer.U;

	txmsg2.can_buffer.W[0] = ChassisStatesSendPC.LM_pose_count.U;
	txmsg2.can_buffer.W[1] = ChassisStatesSendPC.RM_pose_count.U;

	txmsg3.can_buffer.H[0] = ChassisStatesSendPC.LM_vel.U;
	txmsg3.can_buffer.H[1] = ChassisStatesSendPC.RM_vel.U;
	txmsg3.can_buffer.H[2] = ChassisStatesSendPC.SM_vel.U;
	txmsg3.can_buffer.B[6] = ChassisStatesSendPC.control_mode;

	txmsg4.can_buffer.H[0] = ChassisStatesSendPC.LM_cur;
	txmsg4.can_buffer.H[1] = ChassisStatesSendPC.RM_cur;
	txmsg4.can_buffer.H[2] = ChassisStatesSendPC.total_cur;
	txmsg4.can_buffer.H[3] = ChassisStatesSendPC.vbus;

	if(cntCanSendPCSch % 2 == 0)	switch_canmsgobj_send_2pc(&txmsg1, 13);
	if(cntCanSendPCSch % 7 == 0)	switch_canmsgobj_send_2pc(&txmsg2, 14);
	if(cntCanSendPCSch % 5 == 0)	switch_canmsgobj_send_2pc(&txmsg3, 15);
	if(cntCanSendPCSch % 15 == 0)	switch_canmsgobj_send_2pc(&txmsg4, 16);

	(++cntCanSendPCSch) > 50 ? (cntCanSendPCSch = 1) : NULL;
}

// 发送CAN报文给PC
void switch_canmsgobj_send_2pc(CAN_Message *msg, uint8 sw)
{
	switch(sw)
	{
		case 13:
			msg->id = CAN_TXPC_ID1;
			CAN0_Send_Msg(&canSrcMsgObj_Std_13, msg);
			break;
		case 14:
			msg->id = CAN_TXPC_ID2;
			CAN0_Send_Msg(&canSrcMsgObj_Std_14, msg);
			break;
		case 15:
			msg->id = CAN_TXPC_ID3;
			CAN0_Send_Msg(&canSrcMsgObj_Std_15, msg);
			break;
		case 16:
			msg->id = CAN_TXPC_ID4;
			CAN0_Send_Msg(&canSrcMsgObj_Std_16, msg);
			break;
	}
}

// 按照艾思控的CAN协议填充CAN报文
void fill_can_msg_buff(CAN_Message *msg, Data0_Cmd data0_cmd, Motor_Parm_Index mp_Index, Motor_Sub_Index ms_Index, uint32 data32)
{
	// 数据字节0-命令
	msg->can_buffer.B[0] = data0_cmd;

	// 数据字节1、2-索引号
	msg->can_buffer.B[1] = mp_Index;
	msg->can_buffer.B[2] = mp_Index >> 8;

	// 数据字节3-子索引号
	msg->can_buffer.B[3] = ms_Index;

	// 数据字节4-7-数据
	msg->can_buffer.W[1] = data32;
}

void Can_Message_Init(CAN_Message *msg, uint32 id, uint32 dataLow, uint32 dataHigh, IfxMultican_DataLengthCode lengthCode)
{
    msg->id = id;
    msg->can_buffer.W[0] = dataLow;
    msg->can_buffer.W[1] = dataHigh;
    msg->lengthCode  = lengthCode;

    msg->fastBitRate = FALSE;
}
