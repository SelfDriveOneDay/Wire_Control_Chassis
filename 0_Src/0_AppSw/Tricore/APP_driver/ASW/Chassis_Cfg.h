/*
 * Chassis_Cfg.h
 *
 *  Created on: 2021年9月24日
 *      Author: Administrator
 */

#ifndef _CHASSIS_CFG_H_
#define _CHASSIS_CFG_H_

#define PI 3.1415926
#define V_L 1800			// 轴距,单位mm
#define V_B 1200			// 轮距,单位mm
#define R 507.8/2						// 轮胎半径 公式计算得到,单位mm
#define Max_Delta 45 					// 最大转角45度
#define Steering_Shaft_Angle 1.5 		// 转向电机转一个位置对应的转向主轴增加的角度
#define Circle_Counts 138				// 驱动电机每转一圈增加的计数值
#define Steering_Machine_ratio	35.5	// 转向器主轴转动角度与前轮转角的比例关系需要标定

// 驱动电机极对数23对
// 转向电机极对数4对
// 转向电机位置控制精度，转向电机转动一个位置 齿轮齿条转向器主轴转动的角度为 1.5° = 120°/极个数(4*2)/减速比(10)
// 驱动电机转一圈对应电机位置计数值为 138 = 360/(120/极个数(23*2)/减速比(1))
#endif
