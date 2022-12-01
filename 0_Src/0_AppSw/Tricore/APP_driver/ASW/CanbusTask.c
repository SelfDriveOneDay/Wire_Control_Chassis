/*
 * CanbusTask.c
 *
 *  Created on: 2021��9��13��
 *      Author: cv
 *      Describe: Tx/Rx CAN����
 */
#include "CanbusTask.h"
#include "VCU_Task.h"

// δ���ǳ�ʼ��
motor Motor[3];

// ���յ�����������͵�Ӧ���Ĵ������ݽ���
void CAN0Rx_Respond_Process(CAN_Message *msg)
{
	uint16 rxID = msg->id & 0x7FF0;			// ��ȡCAN���ĵĽ���ID,ȡ��λ
	uint8  nodeID = msg->id - CAN_RX_ID;	// ��ȡCAN���ĵĽڵ�ID
	uint8  data0 = msg->can_buffer.B[0];	// ��ȡ�ֽ�0��״̬��
	uint16 param_index = msg->can_buffer.B[1] + (msg->can_buffer.B[2] << 8);	// ��ȡ�ֵ�������

	if(rxID != CAN_RX_ID) 	return;		// ���rxID���ǵ����������Ӧ��ID�����˳�����
	// ���յ����������Ӧ���ģ�Ӧ���ĵĵ�����ֽڱ�ʾ6��״̬
	// ���MCU����CAN�������ݵı����������ֽ�Ϊ�������ݵĳ��Ȼ��ߴ�����ֹ�Ĵ���
	// ���MCU����CAN���Ƶı����������ֽ�Ϊ���ͳɹ�������ֹ�Ĵ���
	//	if(data0 == TX_SUCCESS) return;		// TC275ÿ��һ�����Ʊ���,���ͳɹ����ɹ�ִ�У��������ظ��ɹ�������������Ч
	if(data0 == TX_SUSPEND) return;		// ������ֹ��˵�����͵ı��ĳ����⣬���Ա�����

	switch(nodeID)
	{
		case STEERING_MOTOR_CAN_ID:
			if(data0 == TX_SUCCESS)		// �ж�MCU���͵�CAN���Ʊ��ĵ���������Ƿ���ȷ���յ�
			{
				motor_control_param_callback(SM, param_index);	//
			}
			else						// ���յ�����������������ݣ����ݳ���Ϊ1-4�ֽ�
			{
				get_motor_state_param(SM, param_index, msg);	// ��ȡ�����ʵʱ�Ĳ�����ת�٣�λ��
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

// ������óɹ�����Ӧ��־λ��λ, ������óɹ����������Խ�����һ������, �ɴ˱�־λ�жϣ����Բ��ý�������ʱ��������
void motor_control_param_callback(motor_index mIndex, uint16 param_index)
{
	// �жϵ�����õļ����������Ƿ����óɹ�
	switch(param_index)
	{
		case MOTOR_CONTROL_MODE:
			Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = TRUE;	// ������Ӧ���ı���-�������ģʽ���óɹ�
			Motor[mIndex].MCtrl_Mode_Real = Motor[mIndex].MCtrl_Mode;
			break;
		case MOTOR_CONTROL_QUANTITY:	// ����Ŀ�����ָ����Ŀ���ٶ�
			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;	// ������Ӧ���ı���-������������óɹ�
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

// �ӽ��ܵ�CAN�����з��������
void get_motor_state_param(motor_index mIndex, uint16 param_index, CAN_Message *msg)
{
	switch(param_index)
	{
		case POS_COUNT:			// ����ľ���λ�� �ĳ����λ��
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

// �����������ݵı��ģ� ��ȡ�����״̬������������ȱ����ʽ������ʵʱ��Ҫ��ߵĿ��Զ෢
volatile uint16 cntCanReqSch = 1;
void CAN0Tx_Request_Process(void)
{
	CAN_Message txmsg;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	switch(cntCanReqSch)
	{
		case 1: // �ĳ����λ��
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
		// �ڼ�һ��case������
	}
	send_request_msg(&txmsg);
}

// ����ͬһ���״̬��ַ,��3�����������������������
void send_request_msg(CAN_Message *txmsg)
{
	// sw = 3,��Ӧ�ı��Ľṹ����ר�Ÿ������������������õĲ����ٱ�ʹ��
	if(Motor[SM].Ctrl_State.Aim_Pos_Fg == TRUE)
	{
		switch_canid_send(SM, txmsg, 3);	// ע��û�б�����ͬһ��������������������
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

// ��ͬһ���������������CAN����ܣ����ŷ�����Ҫ���һ��ʱ�䣬��ʱ��100-10ms֮��
// ��ʼ������Ĳ������ٶȡ����ٶȵ�λ������ٶȣ�λ������
void motor_params_init(void)
{
	// ������������ٶȡ����ٶȵ�λ
	set_motor_uint(RPM, RAD_S2);

	// ����������������ת��
	set_max_vel(STEER_MOTOR_MAX_VEL, DRIVE_MOTOR_MAX_VEL, DRIVE_MOTOR_MAX_VEL);

	// ��ʼ������Ŀ���ģʽ
	set_control_mode(SM, CLOSE_LOOP_POS);
	set_control_mode(LM, CLOSE_LOOP_VEL);
	set_control_mode(RM, CLOSE_LOOP_VEL);

	// ��Ƭ��ִ���ٶȺܿ죬����Ϊ�˱�֤��ͬ�Կ��������ͱ��ļ��һ��ʱ��
	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, ����ʱ

	// �������������λ��ģʽ
	set_pos_mode(SM, ABSOLUTE_POS_MODE);	// ����λ�����Գ�ʼλ�ã���0λ��Ϊ��׼��
	set_pos_mode(LM, RELATIVE_POS_MODE);	// ���λ��ʾ�⵱ǰλ�ã�����ǰ��λ�ü���ֵ��ʼ��
	set_pos_mode(RM, RELATIVE_POS_MODE);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, ����ʱ

	// λ�ü�������
	clear_pos_count(SM);
	clear_pos_count(LM);
	clear_pos_count(RM);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, ����ʱ

	// ���ü��ٶȵ�λrad/s2, ע�ⲻ��set_aim_accel()
	accel_init(SM, STEER_MOTOR_MAX_ACCEL, STEER_MOTOR_MAX_ACCEL);
	accel_init(LM, DRIVE_MOTOR_MAX_ACCEL, DRIVE_MOTOR_MAX_ACCEL);
	accel_init(RM, DRIVE_MOTOR_MAX_ACCEL, DRIVE_MOTOR_MAX_ACCEL);

	IfxStm_waitTicks(&MODULE_STM1, 3000000);	// 30ms, ����ʱ

}

// ���õ���Ŀ���ģʽ
void set_control_mode(motor_index mIndex, mctrl_mode mcMode)
{
	CAN_Message txmsg;

	// ��������(Ӧ���ڽ��յ��������ķ��͵����óɹ��ķ������ĺ󱣴�����)
	Motor[mIndex].MCtrl_Mode = mcMode;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ���͵ı��ĵ��ֽ���һ��Ҫ����������CANЭ��ƥ�䣬���򲻻���Ч
	fill_can_msg_buff(&txmsg, TX_ONE_BYTE, MOTOR_CONTROL_MODE, SINDEX_0, mcMode);

	if(Motor[mIndex].MCtrl_Mode_Real != mcMode)
	{
		switch_canid_send(mIndex, &txmsg, 0);
		Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = FALSE;	// �����־
	}
	else
		Motor[mIndex].Ctrl_State.Ctrl_Mode_Fg = TRUE;
}

// ע��˺������������������ģ��������������������Ҫһ����ʱ
// ���ת��������λ����Ҫһ��ʱ�䣬ʱ��������ת�٣������йأ��������Ҫ���ٶȺͼ��ٶ���ߵ�, ͬʱҪ�趨���ת�٣����������԰ɼ��ٶȼ���
void set_aim_pos(motor_index mIndex, sint32 aim_pos, sint16 aim_vel)
{
	CAN_Message txmsg1, txmsg2;

	// ���������aim_pos��������Ϊ������������ת�ǣ��ٶ�������
	Motor[mIndex].Aim_Pos.S = aim_pos;
	Motor[mIndex].Aim_Vel.S = aim_vel;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// �������ͱ���8���ֽڵ�������
	fill_can_msg_buff(&txmsg1, TX_FOUR_BYTE, AIM_POS, SINDEX_0, Motor[mIndex].Aim_Pos.U);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Vel.U);

	// ������ʵ�ʵ�Ŀ���ٶȲ������趨ֵ�����Ϳ���Ŀ���ٶȵ�CAN���ģ����������Ӧ��־λ��һ
	if(Motor[mIndex].Aim_Vel_Real.S != Motor[mIndex].Aim_Vel.S)
	{
		switch_canid_send(mIndex, &txmsg2, 0);			// λ�ÿ���ʱ�ȷ�Ŀ���ٶ�
		// ��־λ��������´�ʹ��
		Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = FALSE;
	}
	else
		Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;	// ����ĵ��ʵ��Ŀ��λ��Ϊ�趨ֵ�����Է���һ��CAN����

//	IfxStm_waitTicks(&MODULE_STM1,50000);	// 0.5ms

	// ��Ŀ���ٶ����óɹ�(�õ����������͵Ĵ��ͳɹ�������)�ڷ�Ŀ��λ��
	if(Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg == TRUE)
	{
		// ��ǰ�����ʵ�ʵ�Ŀ��λ�ò������趨��Ŀ��λ�ã�����Ŀ��λ��
		if(Motor[mIndex].Aim_Pos_Real.S != Motor[mIndex].Aim_Pos.S)
		{
			switch_canid_send(mIndex, &txmsg1, 1);
			Motor[mIndex].Ctrl_State.Aim_Pos_Fg = FALSE;	// û�н����������־�Լ�����
		}
		else
			Motor[mIndex].Ctrl_State.Aim_Pos_Fg = TRUE;		// λ�õı�־λ��ô����
	}

}

// ע��˺������������������ģ��������������������Ҫһ����ʱ
void set_aim_vel(motor_index mIndex, sint16 aim_vel, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg;

	Motor[mIndex].Aim_Vel.S = aim_vel;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ���can���ģ��������������ѡ����S16�����ֽں�S32�ĸ��ֽڣ�������S16�����ֽڣ��㹻��ʾת��(-3��-3��֮��)
	fill_can_msg_buff(&txmsg, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Vel.U);

	// ����Ŀ��Ӽ��ٶ�
	set_aim_accel(mIndex, aim_accel, aim_decel);

	// ����Ŀ���ٶ�
	if(Motor[mIndex].Ctrl_State.CL_Decel_Fg == TRUE)
	{
		switch_canid_send(mIndex, &txmsg, 0);
//		if(Motor[mIndex].Aim_Vel_Real.S != Motor[mIndex].Aim_Vel.S)
//		{
//			switch_canid_send(mIndex, &txmsg, 0);
//			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = FALSE;	// û�н����������־�Լ�����
//		}
//		else
//			Motor[mIndex].Ctrl_State.Ctrl_Qty_Fg = TRUE;
	}
}

// ��˼�ص�������û��Ť�رջ����ƵĹ��ܣ������ؿ���ֻ���趨������ֵ����Ҫ���ڶ�ת�������أ����������ʧ������ͻ��ת
void set_aim_torque(motor_index mIndex, float cur)
{
	CAN_Message txmsg;

	// ����Ŀ�����
	Motor[mIndex].Aim_Cur.S = cur * CURRENT_K;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ��䷢���趨Ŀ��Ť�ص�CAN����
	fill_can_msg_buff(&txmsg, TX_TWO_BYTE, MOTOR_CONTROL_QUANTITY, SINDEX_0, Motor[mIndex].Aim_Cur.U);

	// ѡ��canID����
	switch_canid_send(mIndex, &txmsg, 0);
}

// ע��˺������������������ģ��������������������Ҫһ����ʱ
void set_aim_accel(motor_index mIndex, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg1, txmsg2;

	// ����Ŀ��Ӽ��ٶ�
	Motor[mIndex].Aim_Accel = aim_accel * REVERSE_K;
	Motor[mIndex].Aim_Decel = aim_decel * REVERSE_K;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ��䷢���趨Ŀ��Ӽ��ٶȵ�CAN����
	fill_can_msg_buff(&txmsg1, TX_TWO_BYTE, CLOSE_LOOP_ACCEL, SINDEX_0, Motor[mIndex].Aim_Accel);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, CLOSE_LOOP_DECEL, SINDEX_0, Motor[mIndex].Aim_Decel);

	if(Motor[mIndex].Aim_Accel_Real != Motor[mIndex].Aim_Accel)
	{
		switch_canid_send(mIndex, &txmsg1, 1);
		Motor[mIndex].Ctrl_State.CL_Accel_Fg = FALSE;
	}
	else
		Motor[mIndex].Ctrl_State.CL_Accel_Fg = TRUE;

//	IfxStm_waitTicks(&MODULE_STM1, 100000);	//1ms, ����ʱ

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

// ���ü��ٶȳ�ʼ��,ֻ����һ��
void accel_init(motor_index mIndex, uint16 aim_accel, uint16 aim_decel)
{
	CAN_Message txmsg1, txmsg2;

	// ����Ŀ��Ӽ��ٶ�
	Motor[mIndex].Aim_Accel = aim_accel * REVERSE_K;
	Motor[mIndex].Aim_Decel = aim_decel * REVERSE_K;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ��䷢���趨Ŀ��Ӽ��ٶȵ�CAN����
	fill_can_msg_buff(&txmsg1, TX_TWO_BYTE, CLOSE_LOOP_ACCEL, SINDEX_0, Motor[mIndex].Aim_Accel);
	fill_can_msg_buff(&txmsg2, TX_TWO_BYTE, CLOSE_LOOP_DECEL, SINDEX_0, Motor[mIndex].Aim_Decel);

	switch_canid_send(mIndex, &txmsg1, 1);	// ������ͬһ���������������Ļᴦ������������Ҫ����ʱ

	IfxStm_waitTicks(&MODULE_STM1, 1000000);	// 10ms, ����ʱ

	switch_canid_send(mIndex, &txmsg2, 2);

	IfxStm_waitTicks(&MODULE_STM1, 1000000);	// 10ms, ����ʱ
}

// ע��˺������������������ģ��������������������Ҫһ����ʱ
// ͨ��can�������õ�����ٶȡ����ٶȵ�λΪתÿ�֣�rad/s2, ֻ��һ��
void set_motor_uint(vel_uint vel_u, accel_uint accel_u)
{
	CAN_Message txmsg1, txmsg2;
	uint8 index;

	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ������������CAN���ĵ�������
	fill_can_msg_buff(&txmsg1, TX_ONE_BYTE, VEL_UINT, SINDEX_0, vel_u);
	fill_can_msg_buff(&txmsg2, TX_ONE_BYTE, ACCEL_UINT, SINDEX_0, accel_u);

	// ���趨���ٶȵ�λ�ͼ��ٶȵ�λ���浽Motor, ������ͨ��CAN���߷���
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
		// �����������֧��1msһ֡can���ģ�1ms��֡���ݣ�������������
		CAN0_Send_Msg(&canSrcMsgObj_Std_1, &txmsg1);
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, ����ʱ
		CAN0_Send_Msg(&canSrcMsgObj_Std_2, &txmsg2);
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, ����ʱ
	}
}

// ͨ��CAN������������ٶȵ�λתÿ��,ֻ��һ��
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
		IfxStm_waitTicks(&MODULE_STM1, 2000000);	// 20ms, ����ʱ
	}
}

// ���õ����λ��ģʽ ֻ��һ��
void set_pos_mode(motor_index mIndex, pos_mode pMode)
{
	CAN_Message txmsg;

	// ��������
	Motor[mIndex].Pos_Mode = pMode;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	fill_can_msg_buff(&txmsg, TX_ONE_BYTE, POS_MODE, SINDEX_0, pMode);

	switch_canid_send(mIndex, &txmsg, 1);
}

// ���λ�ü���ֵ������λ�ã� ֻ��һ��
void clear_pos_count(motor_index mIndex)
{
	CAN_Message txmsg;

	Can_Message_Init(&txmsg, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ������������д0��λ�ü���ֵ��Ϊ��
	fill_can_msg_buff(&txmsg, TX_FOUR_BYTE, RESET_MOTOR_POS_COUNT, SINDEX_0, 0);

	switch_canid_send(mIndex, &txmsg, 2);
}


// ѡ������Ӧ��ID����CAN����
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

// ����PC�˵Ŀ��Ʊ���:(���Գɹ�)
void CAN0Rx_Process_For_PC(CAN_Message *msg)
{
	uint16 rxID = msg->id;

	if(rxID != CAN_RXPC_ID)	return;

	// ��������PC��CAN��������
	// �з������ݽ���(�˴����޸�, ���ݵĳ��ȷ����仯��������һ������ aim_SM_pose)
	//	ChassisCtrlFromPC.aim_vel.U = msg->can_buffer.H[0];
	//	ChassisCtrlFromPC.aim_accel.U = msg->can_buffer.H[1];
	//	ChassisCtrlFromPC.aim_deltaf.U= msg->can_buffer.H[2];

	// Ҫ�ر�ע����ֽ��з��ŵ����ݽ���
	// aim_velռ��12λ(���ֽڣ��з���)��H[0]�ĵ�12λ����B[1]�ĵ���λ��B[0]������λ  B[1] << 8 + B[0]
	ChassisCtrlFromPC.aim_vel.U = msg->can_buffer.H[0] & 0x0FFF;
	if(ChassisCtrlFromPC.aim_vel.U >= 2048)
	{
		ChassisCtrlFromPC.aim_vel.S = ChassisCtrlFromPC.aim_vel.U - 4096;
	}

	// aim_accelռ��10��λ(���ֽڣ��з���)��B[2]�ĵ�6λ��B[1]�ĸ���λ
	ChassisCtrlFromPC.aim_accel.U = ((msg->can_buffer.B[2] & 0x3F) << 4) + ((msg->can_buffer.B[1] & 0xF0) >> 4);
	if(ChassisCtrlFromPC.aim_accel.U >= 512)
	{
		ChassisCtrlFromPC.aim_accel.S = ChassisCtrlFromPC.aim_accel.U - 1024;
	}

	// aim_SM_pose(���ֽڣ��з���)ռ��10��λ��B[3]����λ��B[2]�ĸ���λ
	ChassisCtrlFromPC.aim_SM_pose.U = (msg->can_buffer.B[3] << 2) + ((msg->can_buffer.B[2] & 0xC0) >> 6);
	if(ChassisCtrlFromPC.aim_SM_pose.U >= 512)
	{
		ChassisCtrlFromPC.aim_SM_pose.S = ChassisCtrlFromPC.aim_SM_pose.U - 1024;
	}

	ChassisCtrlFromPC.aim_deltaf.U= msg->can_buffer.H[2];

	// �޷������ݽ���
	ChassisCtrlFromPC.aim_steer_vel = msg->can_buffer.H[3] & 0x0FFF;
	ChassisCtrlFromPC.stop_flag = msg->can_buffer.B[7] & 0x30;
	ChassisCtrlFromPC.control_mode = msg->can_buffer.B[7] & 0xC0;
}

// ���͵��̵�״̬��Ϣ��PC
// �����������ݵı��ģ� ��ȡ�����״̬������������ȱ����ʽ������ʵʱ��Ҫ��ߵĿ��Զ෢
volatile uint16 cntCanSendPCSch = 1;
void send_chassis_states_2pc(void)
{
	CAN_Message txmsg1, txmsg2, txmsg3, txmsg4;

	// ��ʼ������
	Can_Message_Init(&txmsg1, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg2, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg3, 0x100, 0, 0, IfxMultican_DataLengthCode_8);
	Can_Message_Init(&txmsg4, 0x100, 0, 0, IfxMultican_DataLengthCode_8);

	// ��ȡ���̵�ʵʱ״̬����
	PC_Get_Chassis_States();

	// ��䱨������
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

// ����CAN���ĸ�PC
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

// ���հ�˼�ص�CANЭ�����CAN����
void fill_can_msg_buff(CAN_Message *msg, Data0_Cmd data0_cmd, Motor_Parm_Index mp_Index, Motor_Sub_Index ms_Index, uint32 data32)
{
	// �����ֽ�0-����
	msg->can_buffer.B[0] = data0_cmd;

	// �����ֽ�1��2-������
	msg->can_buffer.B[1] = mp_Index;
	msg->can_buffer.B[2] = mp_Index >> 8;

	// �����ֽ�3-��������
	msg->can_buffer.B[3] = ms_Index;

	// �����ֽ�4-7-����
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
