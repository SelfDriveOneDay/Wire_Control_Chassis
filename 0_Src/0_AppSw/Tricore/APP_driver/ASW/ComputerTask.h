/*
 * ComTask.h
 *
 *  Created on: 2021年9月13日
 *      Author: cv
 */

#ifndef _COMPUTERTASK_H_
#define _COMPUTERTASK_H_

#include "Platform_Types.h"
#include "CanbusTask.h"

// 从PC端接收到的CAN控制指令,PC端发送一条CAN报文即可,CANTxID = 0x128(PC send)
typedef struct
{
//	data16 aim_vel;       		// bit0-bit15,16bits,精度×0.01,uint:km/h or (m/s)
//	data16 aim_accel;			// bit16-bit31,16bits,精度×0.01,uint:m/s^2
	// 增加aim_SM_pose
	data16 aim_vel;       		// bit0-bit11,12bits,精度×0.01,uint:(m/s),+-2048/100
	data16 aim_accel;			// bit12-bit21,10bits,精度×0.01,uint:(m/s2),+-512/100
	data16 aim_SM_pose;			// bit22-bit31,10bits,精度*1，目标转向电机位置,+-512(转向电机实际运动范围+-280)
	data16 aim_deltaf;			// bit32-bit47,16bits,精度×0.01,uint:deg,目标前轮转角，用于差速
	uint16 aim_steer_vel;		// bit48-bit59,12bits,精度×0.1,uint:RPM(方向盘转速只能是正值，转向电机转速最高3000，精度为1)
	uint8 stop_flag;			// bit60-bit61,2bits
	uint8 control_mode;	    	// bit62-bit63,2bits,待定,控制模式切换权限仅为T8S遥控器 注意变量名不能和枚举的变量类型同名
}chassis_control_from_pc;

// 向PC端发送的数据,要拆分成两条不同CANID的报文发送,CANTxID1 = 0x325, CANTxID2 = 0x389(MCU send)
typedef struct
{
	// CANTxID1=0x312
	data16 cur_vel;				// uint:km/h -60-60,精度*0.01
//	data16 cur_deltaf;			// uint:deg -35-35,精度*0.01(弃用)
	data16 SM_pose_count;		// -280-280(dai)
	data32 odometer;			// uint:m,精度*0.01

	// CANTxID2=0x342
	data32 LM_pose_count;
	data32 RM_pose_count;
//	data32 Swheel_pose_count;	// -280-280

	// CANTxID3=0x362如果能获取转速的方向就用有符号数据
	data16 LM_vel;				// 0-550RPM 精度×1RPM
	data16 RM_vel;				// 0-550RPM 精度×1RPM
	data16 SM_vel;				// 0-300RPM 精度×1RPM
	uint8 control_mode;			// 控制模式只有三种


	// CANTxID4=0x382
	uint16 LM_cur;				// 流经左电机的总电流精度×0.01A
	uint16 RM_cur;				// 流经右电机的总电流精度×0.01A
	uint16 total_cur;			// 流经电源的总电流精度×0.01A
	uint16 vbus;				// 电源电压精度×0.1V

}chassis_states_send_pc;

void PC_Get_Chassis_States(void);
void PC_Chassis_Control(void);

#endif
