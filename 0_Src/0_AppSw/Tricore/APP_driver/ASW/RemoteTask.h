/*
 * RemoteTask.h
 *
 *  Created on: 2021��9��13��
 *      Author: cv
 */

#ifndef _REMOTETASK_H_
#define _REMOTETASK_H_

#include "Platform_Types.h"
#include "ControlTask.h"

#define Max_Value	0x0708	// ���ֵ1800
#define Mid_Value	0x03E8	// ��ֵ1000
#define Min_Value	0x00C8	// ��Сֵ200

// T8Sң����ͨ������
typedef struct
{
	uint16 ch1_r_stick_lr;
	uint16 ch2_r_stick_fb;
	uint16 ch3_l_stick_fb;
	uint16 ch4_l_stick_lr;	// ң��
	uint16 ch5_r_switch;
	uint16 ch6_r_key;		// ����
	uint16 ch7_l_switch;	// ����
	uint16 ch8_l_knob;		// ��ť
} t8s;

void Sbus_Decode(uint8 *sbus_data);
void Set_Control_Mode(t8s *T8s);
ctrl_mode Get_Control_Mode(void);
void Remote_Chassis_Control(void);

#endif
