/*
 * ComTask.h
 *
 *  Created on: 2021��9��13��
 *      Author: cv
 */

#ifndef _COMPUTERTASK_H_
#define _COMPUTERTASK_H_

#include "Platform_Types.h"
#include "CanbusTask.h"

// ��PC�˽��յ���CAN����ָ��,PC�˷���һ��CAN���ļ���,CANTxID = 0x128(PC send)
typedef struct
{
//	data16 aim_vel;       		// bit0-bit15,16bits,���ȡ�0.01,uint:km/h or (m/s)
//	data16 aim_accel;			// bit16-bit31,16bits,���ȡ�0.01,uint:m/s^2
	// ����aim_SM_pose
	data16 aim_vel;       		// bit0-bit11,12bits,���ȡ�0.01,uint:(m/s),+-2048/100
	data16 aim_accel;			// bit12-bit21,10bits,���ȡ�0.01,uint:(m/s2),+-512/100
	data16 aim_SM_pose;			// bit22-bit31,10bits,����*1��Ŀ��ת����λ��,+-512(ת����ʵ���˶���Χ+-280)
	data16 aim_deltaf;			// bit32-bit47,16bits,���ȡ�0.01,uint:deg,Ŀ��ǰ��ת�ǣ����ڲ���
	uint16 aim_steer_vel;		// bit48-bit59,12bits,���ȡ�0.1,uint:RPM(������ת��ֻ������ֵ��ת����ת�����3000������Ϊ1)
	uint8 stop_flag;			// bit60-bit61,2bits
	uint8 control_mode;	    	// bit62-bit63,2bits,����,����ģʽ�л�Ȩ�޽�ΪT8Sң���� ע����������ܺ�ö�ٵı�������ͬ��
}chassis_control_from_pc;

// ��PC�˷��͵�����,Ҫ��ֳ�������ͬCANID�ı��ķ���,CANTxID1 = 0x325, CANTxID2 = 0x389(MCU send)
typedef struct
{
	// CANTxID1=0x312
	data16 cur_vel;				// uint:km/h -60-60,����*0.01
//	data16 cur_deltaf;			// uint:deg -35-35,����*0.01(����)
	data16 SM_pose_count;		// -280-280(dai)
	data32 odometer;			// uint:m,����*0.01

	// CANTxID2=0x342
	data32 LM_pose_count;
	data32 RM_pose_count;
//	data32 Swheel_pose_count;	// -280-280

	// CANTxID3=0x362����ܻ�ȡת�ٵķ�������з�������
	data16 LM_vel;				// 0-550RPM ���ȡ�1RPM
	data16 RM_vel;				// 0-550RPM ���ȡ�1RPM
	data16 SM_vel;				// 0-300RPM ���ȡ�1RPM
	uint8 control_mode;			// ����ģʽֻ������


	// CANTxID4=0x382
	uint16 LM_cur;				// �����������ܵ������ȡ�0.01A
	uint16 RM_cur;				// �����ҵ�����ܵ������ȡ�0.01A
	uint16 total_cur;			// ������Դ���ܵ������ȡ�0.01A
	uint16 vbus;				// ��Դ��ѹ���ȡ�0.1V

}chassis_states_send_pc;

void PC_Get_Chassis_States(void);
void PC_Chassis_Control(void);

#endif
