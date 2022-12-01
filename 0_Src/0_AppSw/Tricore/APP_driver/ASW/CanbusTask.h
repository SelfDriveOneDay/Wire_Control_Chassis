#ifndef _CANBUSTASK_H_
#define _CANBUSTASK_H_

#include "Platform_Types.h"
#include "BSW/CanApp.h"

// 单位换算系数
#define PWM_K         1000      // PWM占空比换算系数
#define STEER_POS_K   1.5       // 齿轮齿条输入轴转角    1个位置即delta=120/8/10=1.5°
#define DRIVE_POS_K   2.61      // 驱动电机(驱动轮)转角  1个位置即delta=120/46/1=2.61°
#define CURRENT_K     100       // 电流(A)换算系数
#define REVERSE_K     10        // 电机换向频率换算系数
#define VOLTAGE_K     10        // 电压(V)换算系数
#define TEMPERATURE_K 10        // 温度(.C)换算系数

// 电机参数, 之所以用枚举变量可以节省内存，但是变量有相同的值不适合枚举
enum Motors_Parameters
{
	// 电机的极个数、减速比
	STEER_MOTOR_POLES = 8, 		    // 转向电机极个数
	STEER_MOTOR_RATIO = 10,   	    // 转向电机减速比
	DRIVE_MOTOR_POLES = 46,   	    // 驱动电机极个数
	DRIVE_MOTOR_RATIO = 1,    	    // 驱动电机减速比

	// 电机的最大转速、最大电流、极限位置
	STEER_MOTOR_MAX_VEL = 3000,     // 转向电机的最高转速3000RPM
	STEER_MOTOR_MAX_CURRENT = 15,   // 转向电机的最大电流15A
	STEER_MOTOR_MAX_POS = 280,      // 转向电机的极限位置280=420度,1.2圈=288个位置值
	DRIVE_MOTOR_MAX_VEL = 550,      // 驱动电机的最大转速550RPM
	DRIVE_MOTOR_MAX_CURRENT = 35,   // 驱动电机的最大电流35A
	STEER_MOTOR_MAX_ACCEL = 100,   	// 转向电机最大加速度rad/s2, 过大会报错
	DRIVE_MOTOR_MAX_ACCEL = 100,	// 驱动电机最大加速度ras/s2, 过大会报错
};

// CAN总线上传输报文的实际ID为CAN_MSG_ID = Can_Tx_Rx_ID + CanNode_ID

// Can_Tx_Rx_ID表示CAN报文功能的ID，CAN_Tx_ID则报文为主控向电机驱动器写数据或请求数据
enum Can_Tx_Rx_ID
{
    CAN_TX_ID = 0x600,
    CAN_RX_ID = 0x580,
	CAN_TXPC_ID1 = 0x312,
	CAN_TXPC_ID2 = 0x342,
	CAN_TXPC_ID3 = 0x362,
	CAN_TXPC_ID4 = 0x382,
	CAN_RXPC_ID = 0x128,
};

// 各电机驱动CanNode_ID编号，范围0x01~0x7F
enum CanNode_ID
{
    STEERING_MOTOR_CAN_ID    = 0x01,
    LEFT_DRIVE_MOTOR_CAN_ID  = 0x02,
    RIGHT_DRIVE_MOTOR_CAN_ID = 0x03,
};

// 电机索引
typedef enum Motor_Index
{
	SM,	// 转向电机
	LM,	// 左驱动电机
	RM,	// 右驱动电机
} motor_index;

// 发送报文的命令字节，要发送的数据大小要和命令字节匹配
typedef enum Data0_Cmd
{
    TX_ONE_BYTE   = 0x2F,
    TX_TWO_BYTE   = 0x2B,
    TX_THREE_BYTE = 0x27,
    TX_FOUR_BYTE  = 0x23,
    RX_DATA       = 0x40,	// 读取数据
} Data0_Cmd;

enum Data0_State
{
	DATA_LEN_1 = 0x4F,
	DATA_LEN_2 = 0x4B,
	DATA_LEN_3 = 0x47,
	DATA_LEN_4 = 0x43,
	TX_SUCCESS = 0x60,	// 传送成功，接收报文的4-7字节均无效
	TX_SUSPEND = 0x80,	// 传送中止，四个字节为中止代码
};

typedef enum Motor_Parm_Index
{
	// Motor_Control_Index  电机控制对象字典对应的索引号，只有RESET_MOTOR_POS_COUNT只写，其他可读可写
	MOTOR_CONTROL_MODE = 0x2000,
    MOTOR_CONTROL_QUANTITY,      // pwm, vel, current
    POS_MODE,
    AIM_POS,
    OPEN_LOOP_PWM_RISE_TIME,
    OPEN_LOOP_PWM_DOWN_TIME,
    CLOSE_LOOP_ACCEL,
    CLOSE_LOOP_DECEL,
    MAX_VEL,
    VEL_UINT = 0x200A,
    ACCEL_UINT,
    RESET_MOTOR_POS_COUNT = 0x200F,

	// Motor_State_Index  电机的实时状态字典对应的索引号,对应的字典全是只读
	PHASE_CURRENT_ESTIMATE = 0x2100,
	PWM_VALUE,
	MOTOR_REVERSE_FREQ,
	POS_COUNT = 0x2105,				// 绝对位置计数值
	POS_CONTROL_FINISH_STATE,
	RELATIVE_POS_COUNT = 0x2109,	// 相对位置计数值
	VEL_ESTIMATE,
	POS_CONTROL_FINISH_REMAIN_TIME,
	BUS_CURRENT,
	BUS_VOLTAGE,
	IN1_IN3_LEVEL,
	POWER_TUBE_TEMPERATURE = 0x210F, //0x210F
	MOTOR_STALLED_STATE = 0x2111,
	MOTOR_ERROR_STATE,
	VO_OUTPUT_STATE,
	SQ1_SQ2_LEVEL,
	HALL_LEVEL,
	SWITCH_STATE,
	IN1_INPUT_MODE,
	IN1_INPUT_PULSE,
	IN1_INPUT_ANALOG,

	// Can_Parameter_Index  通讯参数对象字典索引，只有SYNC_TIME只读，其他可读可写
	CAN_NODE_ID = 0x2201,
	CAN_BAUD_RATE,
	SYNC_COUNT,
	SYNC_TIME,
} Motor_Parm_Index;

// 电机参数的子索引号
typedef enum Motor_Sub_Index
{
	SINDEX_0,
	SINDEX_1,
	SINDEX_2,
	SINDEX_3,
} Motor_Sub_Index;

// 电机控制模式，不支持扭矩闭环，支持位置闭环时设置电机的目标转速
typedef enum Motor_Control_Mode
{
    OPEN_LOOP_PWM,
    CLOSE_LOOP_VEL,
    OPEN_LOOP_TORQUE,       // 该驱动器没有力矩闭环控制方式，一般用于电机堵转控制
    CLOSE_LOOP_POS,
    NORMAL_STOP = 0x10,
    EMERGENCY_STOP,         // 电机停止并保持一定打得阻力矩
    FREE_STOP,
} mctrl_mode;

// 位置控制模式：绝对、相对
typedef enum Pos_Mode
{
    ABSOLUTE_POS_MODE,
    RELATIVE_POS_MODE,
} pos_mode;

// 电机速度单位
typedef enum Vel_Uint
{
    HZ,
    RPM,
} vel_uint;

// 电机加速度单位
typedef enum ACCEL_Uint
{
    HZ_S,
    RAD_S2,
} accel_uint;

// 电机位置控制状态
typedef enum Motor_Pos_Control_State
{
    NO_FINISH,
    FINISH,
} pos_state;

// 电机堵转状态
typedef enum Motor_Stalled_State
{
    NO_STALLED,
    FORWARD_SATLLED_STOP,
    REVERSE_STALLED_STOP,
} stall_state;

// 电机故障状态
typedef enum Motor_Error_State
{
    NO_ERROR,
    NO_STUDY,
    STALLED_STOP,
    HALL_ERROR,
    UNABLE_TO_AIM_VEL,
    RESERVE,
    OVERCURRENT_SHUTDOWN,
    OVERHEAT_SHUTDOWN,
    OVERVOLTAGE_SHUTDOWN,
    UNDERVOLTAGE_SHUTDOWN,
    SHORTCIRCUIT_SHUTDOWN,
} error_state;

// data16和data32变量类型的定义移到了Canapp.h
// 定义16位数据共同体
typedef union
{
	sint16 S;	// 有符号16数据
	uint16 U;	// 有符号16数据的补码，正数的补码即其原码，负数的补码即其反码加1
} data16;

// 定义32位数据共同体
typedef union
{
	sint32 S;
	uint32 U;
	float32 D;
} data32;

// 电机控制参数状态：保存对应参数有没有设置成功, 为1表明成功，为0设置失败
typedef struct
{
	boolean Ctrl_Mode_Fg;
	boolean Ctrl_Qty_Fg;
	boolean Aim_Pos_Fg;
	boolean CL_Accel_Fg;
	boolean CL_Decel_Fg;
} ctrl_state;


// 储存电机状态的结构体变量
typedef struct
{
    // 电机的控制量和估计值
	data16 Aim_Vel;			// 转每分
    data32 Aim_Pos;			// 电机转过的位置，一个位置对应一个位置系数
    data16 Aim_Cur;			// 电机相电流,实际电流乘/除电流系数
    uint16 Aim_Accel;		// 速度闭环加速-加速度
    uint16 Aim_Decel;		// 速度闭环减速-减速度
    uint16 Vel_Est;			// 速度测量值(估计值) uint16就够了 sint16也可以
    data32 Pos_Est;			// 位置测量值(估计值),实际位置乘/除位置系数
//    data32 Relative_POS_Est;// 相对位置测量值(估计值)，电机停止相对位置计数值清零
    uint16 Cur_Est;			// 相电流测量值(估计值),实际电流乘/除电流系数

    // 电机控制模式(只写)
    mctrl_mode  MCtrl_Mode;		// 电机控制模式
    pos_mode    Pos_Mode;		// 位置模式
    vel_uint    Vel_Uint;		// 速度单位
    accel_uint  Accel_Uint;		// 加速度单位

    // 电机状态(只读)，不能用自定义的枚举变量类型可能报错
    uint8  Pos_State;			// 位置完成状态， 0未完成，1完成
    uint16 Max_Vel;				// 最大转速
    uint16 Bus_Cur;				// 母线电流
    uint16 Bus_Volt;			// 母线电压
    uint16 Tube_Temp;			// 功率管温度
    // uint8  Stall_State;			// 堵转状态
    uint8  Error_State;			// 电机错误

// 提高CAN控制实时性有关变量：MCU虽然向电机驱动器发送了控制命令，但是不知道电机控制器有没有正确接收到，所以定义了这些变量
    ctrl_state Ctrl_State;		// 电机控制参数设置状态
    mctrl_mode MCtrl_Mode_Real;	// 驱动器实际电机控制模式
    data16	   Aim_Vel_Real;	// 驱动器目标速度的真实值
    data32     Aim_Pos_Real;	// 驱动器目标位置的真实值
//    data16     Aim_Cur_Real;	// 驱动器目标电流的真实值
    uint16 	   Aim_Accel_Real;	// 驱动器目标加速度的真实值
    uint16 	   Aim_Decel_Real;	// 驱动器目标加速度的真实值

} motor;

// 接收驱动器的应答报文，并对CAN报文数据进行解析
void CAN0Rx_Respond_Process(CAN_Message *msg);
void motor_control_param_callback(motor_index mIndex, uint16 param_index);
void get_motor_state_param(motor_index mIndex, uint16 param_index, CAN_Message *msg);

// 向电机驱动器发送数据请求，请求电机转速，转速为电机轴的转速
void CAN0Tx_Request_Process(void);
void send_request_msg(CAN_Message *txmsg);

// PC
void CAN0Rx_Process_For_PC(CAN_Message *msg);
void switch_canmsgobj_send_2pc(CAN_Message *msg, uint8 sw);
void send_chassis_states_2pc(void);

// 电机参数初始化，通过CAN总线设置电机的控制量
void motor_params_init(void);
void set_control_mode(motor_index mIndex, mctrl_mode mcMode);
void set_aim_pos(motor_index mIndex, sint32 aim_pos, sint16 aim_vel);
void set_aim_vel(motor_index mIndex, sint16 aim_vel, uint16 aim_accel, uint16 aim_decel);
void set_aim_torque(motor_index mIndex, float cur);
void set_aim_accel(motor_index mIndex, uint16 aim_accel, uint16 aim_decel);
void accel_init(motor_index mIndex, uint16 aim_accel, uint16 aim_decel);
void set_motor_uint(vel_uint vel_u, accel_uint accel_u);
void set_max_vel(uint16 SM_max_vel, uint16 LM_max_vel, uint16 RM_max_vel);
void set_pos_mode(motor_index mIndex, pos_mode pMode);
void clear_pos_count(motor_index mIndex);
void switch_canid_send(motor_index mIndex, CAN_Message *msg, uint8 sw);
void fill_can_msg_buff(CAN_Message *msg, Data0_Cmd data0_cmd, Motor_Parm_Index mp_Index, Motor_Sub_Index ms_Index, uint32 data32);
void Can_Message_Init(CAN_Message *msg, uint32 id, uint32 dataLow, uint32 dataHigh, IfxMultican_DataLengthCode lengthCode);

#endif
