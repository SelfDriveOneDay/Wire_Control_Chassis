/*
 * CpuTask.c
 *
 *  Created on: 2021年9月13日
 *      Author: cv
 */

#include "ComputerTask.h"
#include "VCU_Task.h"

// 接收PC端发送来的底盘控制量
chassis_control_from_pc ChassisCtrlFromPC =
{
	.aim_accel = 0.0,
	.aim_deltaf = 0.0,
	.aim_steer_vel = 0.0,
	.aim_vel = 0.0,
	.control_mode = FALSE,
	.stop_flag = FALSE,
};

// 向PC端发送底盘的状态量
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

// 测试成功
void PC_Get_Chassis_States(void)
{
	// 底盘当前速度(km/h),转角(deg),里程计(m),注意底盘状态的真实值需要提前计算
	// 底盘的反馈速度改成(m/s)
	ChassisStatesSendPC.cur_vel.S = Chassis.Vel.Real * 100;
	// 底盘反馈前轮转角意义不大，所以改成直接转向电机的位置
	ChassisStatesSendPC.SM_pose_count.S = Motor[SM].Pos_Est.S;
//	ChassisStatesSendPC.cur_deltaf.S = Chassis.Delta.Real * 100;
	ChassisStatesSendPC.odometer.D = Chassis.Odometer;

	// 电机转动的绝对位置计数
	ChassisStatesSendPC.LM_pose_count.S = Motor[LM].Pos_Est.S;
	ChassisStatesSendPC.RM_pose_count.S = Motor[RM].Pos_Est.S;

	// 电机的转速和底盘的控制模式,注意电机驱动器反馈的转速是无符号的不能表示方向
	ChassisStatesSendPC.LM_vel.S = Motor[LM].Vel_Est;
	ChassisStatesSendPC.RM_vel.S = Motor[RM].Vel_Est;
	ChassisStatesSendPC.SM_vel.S = Motor[SM].Vel_Est;
	ChassisStatesSendPC.control_mode = (uint8)Chassis.Ctrl_Mode;	// 强制转换

	// 电机电流和电源电压
	ChassisStatesSendPC.LM_cur = Motor[LM].Bus_Cur;
	ChassisStatesSendPC.RM_cur = Motor[RM].Bus_Cur;
	ChassisStatesSendPC.total_cur = Motor[LM].Bus_Cur + Motor[RM].Bus_Cur + Motor[SM].Bus_Cur;
	ChassisStatesSendPC.vbus = Motor[SM].Bus_Volt;	// 三个电机驱动器并联电压相等
}

// PC底盘控制
void PC_Chassis_Control(void)
{
	Chassis.Vel.Ref = ChassisCtrlFromPC.aim_vel.S * 0.01;
	Chassis.Accel.Ref = ChassisCtrlFromPC.aim_accel.S * 0.01;
	Chassis.Delta.Ref = ChassisCtrlFromPC.aim_deltaf.S * 0.01;
	Chassis.Steer_Vel = ChassisCtrlFromPC.aim_steer_vel;
//	Chassis.Steer_Vel = ChassisCtrlFromPC.aim_steer_vel * 0.1; 修改
	// PC端改变底盘的控制模式（暂不使用）
//	Chassis.Ctrl_Mode = (ctrl_mode)ChassisCtrlFromPC.control_mode;	// 暂时这样强制转换
}


