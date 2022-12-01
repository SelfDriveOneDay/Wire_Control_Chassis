/*
 * ControlTask.c
 *
 *  Created on: 2021年9月23日
 *      Author: Administrator
 */

#include "ControlTask.h"
#include "VCU_Task.h"
#include "math.h"

chassis Chassis;

// 底盘参数初始化
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

// 1.控制量可以采用纵向车速V和横摆角速度W
// 2.控制量可以采用纵向车速V和前轮转角delta
// 阿克曼底盘运动学解算,后驱动电机转速,前转向电机转角
ctrl_mode last_ctrl_mode,current_ctrl_mode;
void Akman_Chassis_Kinematics_Solve(void)
{
	// current_ctrl_mode = PC_MODE; //
	//	current_ctrl_mode = Get_Control_Mode();
	switch(Get_Control_Mode())
	{
		// 遥控模式不满足阿克曼转向几何
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

// 遥控器控制模式
void Remote_Mode_Process(void)
{
	float differ_ratio = 0;

	// 遥控器的通道值转换为前轮转角值和转速值
	Remote_Chassis_Control();

	if(fabs(Chassis.Delta.Ref) < 6) Chassis.Delta.Ref = 0.0;
	if(fabs(Chassis.Vel.Ref) < 10) Chassis.Vel.Ref = 0.0;

	// 如果速度为零不需要差速
	if(Chassis.Vel.Ref == 0)
		differ_ratio = 0.0;
	else
		differ_ratio = 0.1*Chassis.Delta.Ref*Chassis.Vel.Ref/(fabs(Chassis.Delta.Ref)+0.01);

	Chassis.FW_Pos = Chassis.Delta.Ref;
	Chassis.LW_Vel = (Chassis.Vel.Ref - differ_ratio);
	Chassis.RW_Vel = (Chassis.Vel.Ref + differ_ratio);
//	if(Chassis.LW_Vel < 2.0) Chassis.LW_Vel = 0;
//	if(Chassis.RW_Vel < 2.0) Chassis.RW_Vel = 0;

	// 遥控模式时，将转向电机改为位置闭环，驱动电机改为速度闭环
	Change_Control_Mode(CLOSE_LOOP_POS, CLOSE_LOOP_VEL);	// 有死延时

	// 当前面的位置设置成功才对转向电机驱动器发送下一条报文，避免发送失败和死延时
	if(Motor[SM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
	{
//		set_aim_pos(SM, Chassis.FW_Pos, 3000);	//  电机转速为固定值3000rpm

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

// PC控制模式,根据PC端发来的CAN报文控制底盘运动
void PC_Mode_Process(void)
{
	// PC控制模式.若停车标志置一则改为停车模式,若停车标志未置一改为闭环控制模式
	if(ChassisCtrlFromPC.stop_flag == TRUE)
	{
		set_aim_pos(SM, 0, 2000);							// 方向复位
		Change_Control_Mode(FREE_STOP, EMERGENCY_STOP);		// 改变控制模式
	}
	else
	{
		Change_Control_Mode(CLOSE_LOOP_POS, CLOSE_LOOP_VEL);
		// 设置底盘的目标控制量
		PC_Chassis_Control();

//		Chassis.FW_Pos = Chassis.Delta.Ref * 12.5 / Steering_Shaft_Angle;
		// 这里修改为：底盘的转向电机目标位置直接由PC端发送的转向电机位置直接给出
		Chassis.FW_Pos = ChassisCtrlFromPC.aim_SM_pose.S;

		// 运动学解算 角速度逆时针为正， 前轮转角逆时针为正
		Chassis.Yaw_Rate = Chassis.Vel.Ref * tan(Chassis.Delta.Ref * PI / 180.0);
		// 左右轮相对于地面的线速度，单位m/s
		Chassis.Vx_LW = Chassis.Vel.Ref - Chassis.Yaw_Rate * (V_B/1000) / 2;
		Chassis.Vx_RW = Chassis.Vel.Ref + Chassis.Yaw_Rate * (V_B/1000) / 2;
		// 由左右轮对地线速度计算左右轮转速
		Chassis.LW_Vel = 3.6 * Chassis.Vx_LW / 0.377 / (R/1000);
		Chassis.RW_Vel = 3.6 * Chassis.Vx_RW / 0.377 / (R/1000);

		// 发送给电机驱动器
		// 驱动电机的转速与底盘整体速度的映射关系,驱动电机的加速度与底盘的加速度的映射关系
		// 可能是电池的电压不够，或者是电池的输出功率不够，电机无法运转的指定转速，实际转速与目标转速差两倍左右
		if(Motor[SM].Ctrl_State.Ctrl_Mode_Fg == TRUE)
		{
			// 转向电机的转动位置与前轮转角的角度的映射关系需要标定
			set_aim_pos(SM, Chassis.FW_Pos, Chassis.Steer_Vel); // 改动 原来*10
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
// 正常停车模式处理进程
void Stop_Mode_Process(void)
{
	// 转角，速度给零
	Chassis.Delta.Ref = 0.0;
	Chassis.Vel.Ref = 0.0;

	// 停车模式，将转向电机改为自由停止模式，驱动电机紧急停止模式
	set_aim_pos(SM, 0, 2000);							// 死延时
	Change_Control_Mode(FREE_STOP, NORMAL_STOP);		// 死延时
}

// 紧急提车模式
void EMERGENCY_Stop_Mode_Process(void)
{
	// 转角，速度给零
	Chassis.Delta.Ref = 0.0;
	Chassis.Vel.Ref = 0.0;

	// 停车模式，将转向电机改为自由停止模式，驱动电机紧急停止模式
	set_aim_pos(SM, 0, 2000);							// 死延时
	Change_Control_Mode(FREE_STOP, EMERGENCY_STOP);		// 死延时
}

// 注意每修改一次电机状态(只需要发一次报文即可)，就死延时100ms,让CAN模块缓一下，以便下次能正常发送报文
void Change_Control_Mode(mctrl_mode mcMode1, mctrl_mode mcMode2)
{
	set_control_mode(SM, mcMode1);
	set_control_mode(LM, mcMode2);
	set_control_mode(RM, mcMode2);
}

// 由电机的反馈数据更新底盘状态
void Update_Chassis_States(void)
{
	Chassis.Vel.Real = 0.377 * (Motor[LM].Vel_Est + Motor[RM].Vel_Est)/2.0 * R / 1000 / 3.6;
	// 前轮转角的真实值由转向电机的位置来确定，这里只是大概给出关系，放弃不用
//	Chassis.Delta.Real = Motor[SM].Pos_Est.S * Steering_Shaft_Angle / 12.5 ;
	Chassis.Odometer =  (Motor[LM].Pos_Est.S + Motor[RM].Pos_Est.S)/Circle_Counts/2 * (2*PI*R) / 1000;
}

// static静态变量值不会改变
uint16 chassisCtrlSch = 0;
// 底盘控制任务，放在1ms定时器中断中执行
void Chassis_Control_Task(void)
{
	chassisCtrlSch++;

	// 每隔5ms向电机发送控制命令:可以切换不同的控制模式
	if(chassisCtrlSch % 17 == 0)	// 10ms
	{
		Akman_Chassis_Kinematics_Solve();
	}

	// 错开了向电机控制器发送控制命令和查询命令，以防止电机控制器响应不过来
	// 每隔11ms向电机驱动器发送查询命令
	if(chassisCtrlSch % 47 == 0)	//
	{
		 CAN0Tx_Request_Process();
		 Update_Chassis_States();
	}

	// 向PC端发送底盘状态信息
	if(chassisCtrlSch % 37 == 0)	//
	{
		send_chassis_states_2pc();
		IfxPort_togglePin(&MODULE_P33, 11);			//LED4
	}

	if(chassisCtrlSch == 50)  chassisCtrlSch = 0;

}
