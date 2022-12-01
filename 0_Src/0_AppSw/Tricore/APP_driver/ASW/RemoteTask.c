/*
 * RemoteTask.c
 *
 *  Created on: 2021��9��13��
 *      Author: cv
 */

#include "RemoteTask.h"
#include "VCU_Task.h"


// T8Sң����ͨ������, ������ֵ
t8s T8S =
{
		.ch1_r_stick_lr = Mid_Value,	// ��ֵ1000
		.ch2_r_stick_fb = Mid_Value,
		.ch3_l_stick_fb = Mid_Value,
		.ch4_l_stick_lr = Mid_Value,
		.ch5_r_switch   = Min_Value,	// ��Сֵ200
		.ch6_r_key      = Max_Value,	// ���ֵ1800
		.ch7_l_switch   = Min_Value,
		.ch8_l_knob     = Mid_Value,
};

// ���ڴ��ڽ����ж�
void Sbus_Decode(uint8 *sbus_data)
{
	if(sbus_data == NULL)
		return ;

	// SBUS_DATA��λ��ǰ����λ�ں�
	T8S.ch1_r_stick_lr = (sbus_data[1]  << 8) | sbus_data[2];
	T8S.ch2_r_stick_fb = (sbus_data[3]  << 8) | sbus_data[4];
	T8S.ch3_l_stick_fb = (sbus_data[5]  << 8) | sbus_data[6];
	T8S.ch4_l_stick_lr = (sbus_data[7]  << 8) | sbus_data[8];
	T8S.ch5_r_switch   = (sbus_data[9]  << 8) | sbus_data[10];
	T8S.ch6_r_key      = (sbus_data[11] << 8) | sbus_data[12];
	T8S.ch7_l_switch   = (sbus_data[13] << 8) | sbus_data[14];
	T8S.ch8_l_knob     = (sbus_data[15] << 8) | sbus_data[16];

	Set_Control_Mode(&T8S);
	//Remote_Chassis_Control();			// test
}

void Set_Control_Mode(t8s *T8s)
{
	if(T8s->ch7_l_switch == Min_Value)
	{
		Chassis.Ctrl_Mode = PC_MODE;
	}
	else if(T8s->ch7_l_switch == Mid_Value)
	{
		Chassis.Ctrl_Mode = STOP_MODE;	 // Ĭ��ֵ
	}
	else if(T8s->ch7_l_switch == Max_Value)
	{
		Chassis.Ctrl_Mode = REMOTE_MODE;
	}
	// T8S�ұ߿������ý���ͣ��ģʽ
	if(T8s->ch5_r_switch == Mid_Value)
	{
		Chassis.Ctrl_Mode = EMERGENCY_STOP_MODE;
	}
}

ctrl_mode Get_Control_Mode(void)
{
	return Chassis.Ctrl_Mode;
}

// ����ǰ��ת��(���ٶ�)����������
void Remote_Chassis_Control(void)
{
	float vel_k, delta_k;

	vel_k = 1.0 * (T8S.ch2_r_stick_fb - Mid_Value) / (Mid_Value - Min_Value);
	delta_k = 1.0* (T8S.ch4_l_stick_lr - Mid_Value) / (Mid_Value - Min_Value);

	if(vel_k < 0)
		Chassis.Vel.Ref = 1.0 * 200 * vel_k;		// ��ֵΪС��0.0
	else
		Chassis.Vel.Ref = 1.0 * 250 * vel_k;		// ��ֵΪС��0.0

	Chassis.Delta.Ref = 1.0 * STEER_MOTOR_MAX_POS * delta_k;
}


