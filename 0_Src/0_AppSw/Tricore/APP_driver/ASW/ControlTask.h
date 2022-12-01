/*
 * ControlTask.h
 *
 *  Created on: 2021��9��23��
 *      Author: Administrator
 */

#ifndef _CONTROLTASK_H_
#define _CONTROLTASK_H_

#include "CanbusTask.h"
#include "Platform_Types.h"

typedef enum Chassis_ControlMode
{
	REMOTE_MODE = 0,
	PC_MODE,
	STOP_MODE,
	EMERGENCY_STOP_MODE,
} ctrl_mode;

typedef union
{
	uint32 Integer;
	float  Decimal;
} IEEE_754;

typedef struct
{
	float Ref;		// ״̬�Ĳο�ֵҲ��Ŀ��ֵ
	float Real;		// ״̬��ʵ��ֵ(һ���ɴ��������)
//	float Error;		// ���=Real-Ref
} ref_real;

// ���̱����ṹ�壬����״̬��ʵ��ֵ�Ͳο�ֵ, ����ĳ��٣��Լ��������ֵ�ת�٣�ǰת���������ת��
typedef struct
{
	// ���µ��̵Ŀ�������ֱ������Զ��ң������PC��
	ref_real Delta;			// ǰ��ת�ǣ���λΪdeg��
	ref_real Vel;			// ������(m/s)
	ref_real Accel;			// ������ٶ�(m/s2)
	float32 Steer_Vel;		// ������ת��(λ��ģʽֻ������, Ϊת������ת��)
	float32 Odometer;		// ��̼�(��λm)
	float32 Yaw_Rate;		// ���̵Ľ��ٶȣ����˶�ѧ����õ�
	float32 Vx_LW;			// ����������ڵ�������ٶȣ���λm/s
	float32 Vx_RW;

	// ���µ��̵���Ŀ�������������ĵ��̿��������ɱ궨��������������
	float32 LW_Vel;			// ����ת��,����������
	float32 LW_Accel;		// ���ֽǼ��ٶ�,��������Ǽ��ٶ�ֱ�ӿ�����
	float32 RW_Vel;			// �ҳ�������,����������
	float32 RW_Accel;		// �ҳ��ֽǼ��ٶ�,��������Ǽ��ٶ�ֱ�ӿ�����
	float32 FW_Pos;			// ǰת���ֵ�λ��,λ��������

	ctrl_mode Ctrl_Mode;

} chassis;

void Remote_Mode_Process(void);
void PC_Mode_Process(void);
void Stop_Mode_Process(void);
void EMERGENCY_Stop_Mode_Process(void);
void Change_Control_Mode(mctrl_mode mcMode1, mctrl_mode mcMode2);

void Update_Chassis_States(void);
void Chassis_Init(void);
void Akman_Chassis_Kinematics_Solve(void);
void Chassis_Control_Task(void);

#endif
