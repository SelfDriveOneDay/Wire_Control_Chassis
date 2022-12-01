/*
 * CpuTask.c
 *
 *  Created on: 2021��9��13��
 *      Author: cv
 */

#include "ComputerTask.h"
#include "VCU_Task.h"

// ����PC�˷������ĵ��̿�����
chassis_control_from_pc ChassisCtrlFromPC =
{
	.aim_accel = 0.0,
	.aim_deltaf = 0.0,
	.aim_steer_vel = 0.0,
	.aim_vel = 0.0,
	.control_mode = FALSE,
	.stop_flag = FALSE,
};

// ��PC�˷��͵��̵�״̬��
chassis_states_send_pc ChassisStatesSendPC =
{
	.LM_cur = 0,
	.LM_pose_count = 0,
	.LM_vel = 0,
	.RM_cur = 0,
	.RM_pose_count = 0,
	.RM_vel = 0,
	.SM_vel = 0,
	.control_mode = 0,
//	.cur_deltaf = 0,
	.SM_pose_count = 0,
	.cur_vel = 0,
	.odometer = 0,
	.total_cur = 0,
	.vbus = 0,
};

// ���Գɹ�
void PC_Get_Chassis_States(void)
{
	// ���̵�ǰ�ٶ�(km/h),ת��(deg),��̼�(m),ע�����״̬����ʵֵ��Ҫ��ǰ����
	// ���̵ķ����ٶȸĳ�(m/s)
	ChassisStatesSendPC.cur_vel.S = Chassis.Vel.Real * 100;
	// ���̷���ǰ��ת�����岻�����Ըĳ�ֱ��ת������λ��
	ChassisStatesSendPC.SM_pose_count.S = Motor[SM].Pos_Est.S;
//	ChassisStatesSendPC.cur_deltaf.S = Chassis.Delta.Real * 100;
	ChassisStatesSendPC.odometer.D = Chassis.Odometer;

	// ���ת���ľ���λ�ü���
	ChassisStatesSendPC.LM_pose_count.S = Motor[LM].Pos_Est.S;
	ChassisStatesSendPC.RM_pose_count.S = Motor[RM].Pos_Est.S;

	// �����ת�ٺ͵��̵Ŀ���ģʽ,ע����������������ת�����޷��ŵĲ��ܱ�ʾ����
	ChassisStatesSendPC.LM_vel.S = Motor[LM].Vel_Est;
	ChassisStatesSendPC.RM_vel.S = Motor[RM].Vel_Est;
	ChassisStatesSendPC.SM_vel.S = Motor[SM].Vel_Est;
	ChassisStatesSendPC.control_mode = (uint8)Chassis.Ctrl_Mode;	// ǿ��ת��

	// ��������͵�Դ��ѹ
	ChassisStatesSendPC.LM_cur = Motor[LM].Bus_Cur;
	ChassisStatesSendPC.RM_cur = Motor[RM].Bus_Cur;
	ChassisStatesSendPC.total_cur = Motor[LM].Bus_Cur + Motor[RM].Bus_Cur + Motor[SM].Bus_Cur;
	ChassisStatesSendPC.vbus = Motor[SM].Bus_Volt;	// �������������������ѹ���
}

// PC���̿���
void PC_Chassis_Control(void)
{
	Chassis.Vel.Ref = ChassisCtrlFromPC.aim_vel.S * 0.01;
	Chassis.Accel.Ref = ChassisCtrlFromPC.aim_accel.S * 0.01;
	Chassis.Delta.Ref = ChassisCtrlFromPC.aim_deltaf.S * 0.01;
	Chassis.Steer_Vel = ChassisCtrlFromPC.aim_steer_vel;
//	Chassis.Steer_Vel = ChassisCtrlFromPC.aim_steer_vel * 0.1; �޸�
	// PC�˸ı���̵Ŀ���ģʽ���ݲ�ʹ�ã�
//	Chassis.Ctrl_Mode = (ctrl_mode)ChassisCtrlFromPC.control_mode;	// ��ʱ����ǿ��ת��
}


