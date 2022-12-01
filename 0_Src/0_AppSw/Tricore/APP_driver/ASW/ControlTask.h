/*
 * ControlTask.h
 *
 *  Created on: 2021年9月23日
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
	float Ref;		// 状态的参考值也叫目标值
	float Real;		// 状态的实际值(一般由传感器获得)
//	float Error;		// 误差=Real-Ref
} ref_real;

// 底盘变量结构体，保存状态的实际值和参考值, 整体的车速，以及后驱动轮的转速，前转向电机的输出转角
typedef struct
{
	// 以下底盘的控制量，直接来自远程遥控器或PC端
	ref_real Delta;			// 前轮转角（单位为deg）
	ref_real Vel;			// 纵向车速(m/s)
	ref_real Accel;			// 纵向加速度(m/s2)
	float32 Steer_Vel;		// 方向盘转速(位置模式只能正数, 为转向电机的转速)
	float32 Odometer;		// 里程计(单位m)
	float32 Yaw_Rate;		// 底盘的角速度，由运动学解算得到
	float32 Vx_LW;			// 左右轮相对于地面的线速度，单位m/s
	float32 Vx_RW;

	// 以下底盘电机的控制量，有上面的底盘控制量，由标定计算出电机控制量
	float32 LW_Vel;			// 左车轮转速,轮速有正负
	float32 LW_Accel;		// 左车轮角加速度,驱动电机角加速度直接控制量
	float32 RW_Vel;			// 右车轮轮速,轮速有正负
	float32 RW_Accel;		// 右车轮角加速度,驱动电机角加速度直接控制量
	float32 FW_Pos;			// 前转向轮的位置,位置有正负

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
