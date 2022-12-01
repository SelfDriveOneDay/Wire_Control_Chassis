/*
 * ControlTask.c
 *
 *  Created on: 2021��9��23��
 *      Author: Administrator
 */

#include "ControlTask.h"
#include "VCU_Task.h"
#include "math.h"

chassis Chassis;

// ���̲�����ʼ��
void Chassis_Init(void)
{
	Chassis.Delta.Real = 0.0;
	Chassis.Delta.Ref = 0.0;
	Chassis.Vel.Real = 0.0;
	Chassis.Vel.Ref = 0.0;
	Chassis.Accel.Real = 0.0;
	Chassis.Accel.Ref = 0.0;
	Chassis.Steer_Vel = 0.0;

	Chassis.FW_Pos = 0.0;
	Chassis.LW_Vel = 0.0;
	Chassis.RW_Vel = 0.0;
	Chassis.LW_Accel = 0.0;
	Chassis.RW_Accel = 0.0;

	Chassis.Ctrl_Mode = STOP_MODE;

	Chassis.Odometer = 0.0;
}

// 1.���������Բ���������V�ͺ�ڽ��ٶ�W
// 2.���������Բ���������V��ǰ��ת��delta
// �����������˶�ѧ����,���������ת��,ǰת����ת��
ctrl_mode last_ctrl_mode,current_ctrl_mode;
void Akman_Chassis_Kinematics_Solve(void)
{
	// current_ctrl_mode = PC_MODE; //
	//	current_ctrl_mode = Get_Control_Mode();
	switch(Get_Control_Mode())
	{
		// ң��ģʽ�����㰢����ת�򼸺�
		case REMOTE_MODE:
			last_ctrl_mode = REMOTE_MODE;
			Remote_Mode_Process();
			break;
		case PC_MODE:
			PC_Mode_Process();
			last_ctrl_mode = PC_MODE;
			break;
		case STOP_MODE:
			last_ctrl_mode = STOP_MODE;
			Stop_Mode_Process();
			break;
		case EMERGENCY_STOP_MODE:
			last_ctrl_mode = EMERGENCY_STOP_MODE;
			EMERGENCY_Stop_Mode_Process();
			break;
	}

}

// ң��������ģʽ
void Remote_Mode_Process(void)
{
	float differ_ratio = 0;

	// ң������ͨ��ֵת��Ϊǰ��ת��ֵ��ת��ֵ
	Remote_Chassis_Control();

	if(fabs(Chassis.Delta.Ref) < 6) Chassis.Delta.Ref = 0.0;
	if(fabs(Chassis.Vel.Ref) < 10) Chassis.Vel.Ref = 0.0;

	// ����ٶ�Ϊ�㲻��Ҫ����
	if(Chassis.Vel.Ref == 0)
		differ_ratio = 0.0;
	else
		differ_ratio = 0.1*Chassis.Delta.Ref*Chassis.Vel.Ref/(fabs(Chassis.Delta.Ref)+0.01);

	Chassis.FW_Pos = Chassis.Delta.Ref;
	Chassis.LW_Vel = (Chassis.Vel.Ref - differ_ratio);
	Chassis.RW_Vel = (Chassis.Vel.Ref + differ_ratio);
//	if(Chassis.LW_Vel < 2.0) Chassis.LW_Vel = 0;
//	if(Chassis.RW_Vel < 2.0) Chassis.RW_Vel = 0;

	// ң��ģʽʱ����ת������Ϊλ�ñջ������������Ϊ�ٶȱջ�
	Change_Control_Mode(CLOSE_LOOP_POS, CLOSE_LOOP_VEL);	// ������ʱ

	// ��ǰ���λ�����óɹ��Ŷ�ת����������������һ�����ģ����ⷢ��ʧ�ܺ�����ʱ
	if(Motor[SM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
	{
//		set_aim_pos(SM, Chassis.FW_Pos, 3000);	//  ���ת��Ϊ�̶�ֵ3000rpm

		set_aim_pos(SM, Chassis.FW_Pos, T8S.ch8_l_knob + 1200);
	}
	if(Motor[LM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
	{
		set_aim_vel(LM, Chassis.LW_Vel, 30, 30);
	}

	if(Motor[RM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
	{
		set_aim_vel(RM, Chassis.RW_Vel, 30, 30);
	}

}

// PC����ģʽ,����PC�˷�����CAN���Ŀ��Ƶ����˶�
void PC_Mode_Process(void)
{
	// PC����ģʽ.��ͣ����־��һ���Ϊͣ��ģʽ,��ͣ����־δ��һ��Ϊ�ջ�����ģʽ
	if(ChassisCtrlFromPC.stop_flag == TRUE)
	{
		set_aim_pos(SM, 0, 2000);							// ����λ
		Change_Control_Mode(FREE_STOP, EMERGENCY_STOP);		// �ı����ģʽ
	}
	else
	{
		Change_Control_Mode(CLOSE_LOOP_POS, CLOSE_LOOP_VEL);
		// ���õ��̵�Ŀ�������
		PC_Chassis_Control();

//		Chassis.FW_Pos = Chassis.Delta.Ref * 12.5 / Steering_Shaft_Angle;
		// �����޸�Ϊ�����̵�ת����Ŀ��λ��ֱ����PC�˷��͵�ת����λ��ֱ�Ӹ���
		Chassis.FW_Pos = ChassisCtrlFromPC.aim_SM_pose.S;

		// �˶�ѧ���� ���ٶ���ʱ��Ϊ���� ǰ��ת����ʱ��Ϊ��
		Chassis.Yaw_Rate = Chassis.Vel.Ref * tan(Chassis.Delta.Ref * PI / 180.0);
		// ����������ڵ�������ٶȣ���λm/s
		Chassis.Vx_LW = Chassis.Vel.Ref - Chassis.Yaw_Rate * (V_B/1000) / 2;
		Chassis.Vx_RW = Chassis.Vel.Ref + Chassis.Yaw_Rate * (V_B/1000) / 2;
		// �������ֶԵ����ٶȼ���������ת��
		Chassis.LW_Vel = 3.6 * Chassis.Vx_LW / 0.377 / (R/1000);
		Chassis.RW_Vel = 3.6 * Chassis.Vx_RW / 0.377 / (R/1000);

		// ���͸����������
		// ���������ת������������ٶȵ�ӳ���ϵ,��������ļ��ٶ�����̵ļ��ٶȵ�ӳ���ϵ
		// �����ǵ�صĵ�ѹ�����������ǵ�ص�������ʲ���������޷���ת��ָ��ת�٣�ʵ��ת����Ŀ��ת�ٲ���������
		if(Motor[SM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
		{
			// ת������ת��λ����ǰ��ת�ǵĽǶȵ�ӳ���ϵ��Ҫ�궨
			set_aim_pos(SM, Chassis.FW_Pos, Chassis.Steer_Vel); // �Ķ� ԭ��*10
		}
		if(Motor[LM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
		{
			set_aim_vel(LM, Chassis.LW_Vel*1.78, Chassis.Accel.Ref*20, Chassis.Accel.Ref*20);
//			set_aim_vel(LM, 30*1.8, 50, 50);
		}
		if(Motor[RM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
		{
			set_aim_vel(RM, Chassis.RW_Vel*1.78, Chassis.Accel.Ref*20, Chassis.Accel.Ref*20);
//			set_aim_vel(RM, 30*1.8, 50, 50); // 1.73 - 1.8
		}
	}
}

uint32 nowtime;
// ����ͣ��ģʽ�������
void Stop_Mode_Process(void)
{
	// ת�ǣ��ٶȸ���
	Chassis.Delta.Ref = 0.0;
	Chassis.Vel.Ref = 0.0;

	// ͣ��ģʽ����ת������Ϊ����ֹͣģʽ�������������ֹͣģʽ
	set_aim_pos(SM, 0, 2000);							// ����ʱ
	Change_Control_Mode(FREE_STOP, NORMAL_STOP);		// ����ʱ
}

// �����ᳵģʽ
void EMERGENCY_Stop_Mode_Process(void)
{
	// ת�ǣ��ٶȸ���
	Chassis.Delta.Ref = 0.0;
	Chassis.Vel.Ref = 0.0;

	// ͣ��ģʽ����ת������Ϊ����ֹͣģʽ�������������ֹͣģʽ
	set_aim_pos(SM, 0, 2000);							// ����ʱ
	Change_Control_Mode(FREE_STOP, EMERGENCY_STOP);		// ����ʱ
}

// ע��ÿ�޸�һ�ε��״̬(ֻ��Ҫ��һ�α��ļ���)��������ʱ100ms,��CANģ�黺һ�£��Ա��´����������ͱ���
void Change_Control_Mode(mctrl_mode mcMode1, mctrl_mode mcMode2)
{
	set_control_mode(SM, mcMode1);
	set_control_mode(LM, mcMode2);
	set_control_mode(RM, mcMode2);
}

// �ɵ���ķ������ݸ��µ���״̬
void Update_Chassis_States(void)
{
	Chassis.Vel.Real = 0.377 * (Motor[LM].Vel_Est + Motor[RM].Vel_Est)/2.0 * R / 1000 / 3.6;
	// ǰ��ת�ǵ���ʵֵ��ת������λ����ȷ��������ֻ�Ǵ�Ÿ�����ϵ����������
//	Chassis.Delta.Real = Motor[SM].Pos_Est.S * Steering_Shaft_Angle / 12.5 ;
	Chassis.Odometer =  (Motor[LM].Pos_Est.S + Motor[RM].Pos_Est.S)/Circle_Counts/2 * (2*PI*R) / 1000;
}

// static��̬����ֵ����ı�
uint16 chassisCtrlSch = 0;
// ���̿������񣬷���1ms��ʱ���ж���ִ��
void Chassis_Control_Task(void)
{
	chassisCtrlSch++;

	// ÿ��5ms�������Ϳ�������:�����л���ͬ�Ŀ���ģʽ
	if(chassisCtrlSch % 17 == 0)	// 10ms
	{
		Akman_Chassis_Kinematics_Solve();
	}

	// �����������������Ϳ�������Ͳ�ѯ����Է�ֹ�����������Ӧ������
	// ÿ��11ms�������������Ͳ�ѯ����
	if(chassisCtrlSch % 47 == 0)	//
	{
		 CAN0Tx_Request_Process();
		 Update_Chassis_States();
	}

	// ��PC�˷��͵���״̬��Ϣ
	if(chassisCtrlSch % 37 == 0)	//
	{
		send_chassis_states_2pc();
		IfxPort_togglePin(&MODULE_P33, 11);			//LED4
	}

	if(chassisCtrlSch == 50)  chassisCtrlSch = 0;

}
