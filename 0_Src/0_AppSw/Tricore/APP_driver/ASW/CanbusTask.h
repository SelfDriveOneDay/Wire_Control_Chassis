#ifndef _CANBUSTASK_H_
#define _CANBUSTASK_H_

#include "Platform_Types.h"
#include "BSW/CanApp.h"

// ��λ����ϵ��
#define PWM_K         1000      // PWMռ�ձȻ���ϵ��
#define STEER_POS_K   1.5       // ���ֳ���������ת��    1��λ�ü�delta=120/8/10=1.5��
#define DRIVE_POS_K   2.61      // �������(������)ת��  1��λ�ü�delta=120/46/1=2.61��
#define CURRENT_K     100       // ����(A)����ϵ��
#define REVERSE_K     10        // �������Ƶ�ʻ���ϵ��
#define VOLTAGE_K     10        // ��ѹ(V)����ϵ��
#define TEMPERATURE_K 10        // �¶�(.C)����ϵ��

// �������, ֮������ö�ٱ������Խ�ʡ�ڴ棬���Ǳ�������ͬ��ֵ���ʺ�ö��
enum Motors_Parameters
{
	// ����ļ����������ٱ�
	STEER_MOTOR_POLES = 8, 		    // ת����������
	STEER_MOTOR_RATIO = 10,   	    // ת�������ٱ�
	DRIVE_MOTOR_POLES = 46,   	    // �������������
	DRIVE_MOTOR_RATIO = 1,    	    // ����������ٱ�

	// ��������ת�١�������������λ��
	STEER_MOTOR_MAX_VEL = 3000,     // ת���������ת��3000RPM
	STEER_MOTOR_MAX_CURRENT = 15,   // ת������������15A
	STEER_MOTOR_MAX_POS = 280,      // ת�����ļ���λ��280=420��,1.2Ȧ=288��λ��ֵ
	DRIVE_MOTOR_MAX_VEL = 550,      // ������������ת��550RPM
	DRIVE_MOTOR_MAX_CURRENT = 35,   // ���������������35A
	STEER_MOTOR_MAX_ACCEL = 100,   	// ת���������ٶ�rad/s2, ����ᱨ��
	DRIVE_MOTOR_MAX_ACCEL = 100,	// ������������ٶ�ras/s2, ����ᱨ��
};

// CAN�����ϴ��䱨�ĵ�ʵ��IDΪCAN_MSG_ID = Can_Tx_Rx_ID + CanNode_ID

// Can_Tx_Rx_ID��ʾCAN���Ĺ��ܵ�ID��CAN_Tx_ID����Ϊ��������������д���ݻ���������
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

// ���������CanNode_ID��ţ���Χ0x01~0x7F
enum CanNode_ID
{
    STEERING_MOTOR_CAN_ID    = 0x01,
    LEFT_DRIVE_MOTOR_CAN_ID  = 0x02,
    RIGHT_DRIVE_MOTOR_CAN_ID = 0x03,
};

// �������
typedef enum Motor_Index
{
	SM,	// ת����
	LM,	// ���������
	RM,	// ���������
} motor_index;

// ���ͱ��ĵ������ֽڣ�Ҫ���͵����ݴ�СҪ�������ֽ�ƥ��
typedef enum Data0_Cmd
{
    TX_ONE_BYTE   = 0x2F,
    TX_TWO_BYTE   = 0x2B,
    TX_THREE_BYTE = 0x27,
    TX_FOUR_BYTE  = 0x23,
    RX_DATA       = 0x40,	// ��ȡ����
} Data0_Cmd;

enum Data0_State
{
	DATA_LEN_1 = 0x4F,
	DATA_LEN_2 = 0x4B,
	DATA_LEN_3 = 0x47,
	DATA_LEN_4 = 0x43,
	TX_SUCCESS = 0x60,	// ���ͳɹ������ձ��ĵ�4-7�ֽھ���Ч
	TX_SUSPEND = 0x80,	// ������ֹ���ĸ��ֽ�Ϊ��ֹ����
};

typedef enum Motor_Parm_Index
{
	// Motor_Control_Index  ������ƶ����ֵ��Ӧ�������ţ�ֻ��RESET_MOTOR_POS_COUNTֻд�������ɶ���д
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

	// Motor_State_Index  �����ʵʱ״̬�ֵ��Ӧ��������,��Ӧ���ֵ�ȫ��ֻ��
	PHASE_CURRENT_ESTIMATE = 0x2100,
	PWM_VALUE,
	MOTOR_REVERSE_FREQ,
	POS_COUNT = 0x2105,				// ����λ�ü���ֵ
	POS_CONTROL_FINISH_STATE,
	RELATIVE_POS_COUNT = 0x2109,	// ���λ�ü���ֵ
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

	// Can_Parameter_Index  ͨѶ���������ֵ�������ֻ��SYNC_TIMEֻ���������ɶ���д
	CAN_NODE_ID = 0x2201,
	CAN_BAUD_RATE,
	SYNC_COUNT,
	SYNC_TIME,
} Motor_Parm_Index;

// �����������������
typedef enum Motor_Sub_Index
{
	SINDEX_0,
	SINDEX_1,
	SINDEX_2,
	SINDEX_3,
} Motor_Sub_Index;

// �������ģʽ����֧��Ť�رջ���֧��λ�ñջ�ʱ���õ����Ŀ��ת��
typedef enum Motor_Control_Mode
{
    OPEN_LOOP_PWM,
    CLOSE_LOOP_VEL,
    OPEN_LOOP_TORQUE,       // ��������û�����رջ����Ʒ�ʽ��һ�����ڵ����ת����
    CLOSE_LOOP_POS,
    NORMAL_STOP = 0x10,
    EMERGENCY_STOP,         // ���ֹͣ������һ�����������
    FREE_STOP,
} mctrl_mode;

// λ�ÿ���ģʽ�����ԡ����
typedef enum Pos_Mode
{
    ABSOLUTE_POS_MODE,
    RELATIVE_POS_MODE,
} pos_mode;

// ����ٶȵ�λ
typedef enum Vel_Uint
{
    HZ,
    RPM,
} vel_uint;

// ������ٶȵ�λ
typedef enum ACCEL_Uint
{
    HZ_S,
    RAD_S2,
} accel_uint;

// ���λ�ÿ���״̬
typedef enum Motor_Pos_Control_State
{
    NO_FINISH,
    FINISH,
} pos_state;

// �����ת״̬
typedef enum Motor_Stalled_State
{
    NO_STALLED,
    FORWARD_SATLLED_STOP,
    REVERSE_STALLED_STOP,
} stall_state;

// �������״̬
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

// data16��data32�������͵Ķ����Ƶ���Canapp.h
// ����16λ���ݹ�ͬ��
typedef union
{
	sint16 S;	// �з���16����
	uint16 U;	// �з���16���ݵĲ��룬�����Ĳ��뼴��ԭ�룬�����Ĳ��뼴�䷴���1
} data16;

// ����32λ���ݹ�ͬ��
typedef union
{
	sint32 S;
	uint32 U;
	float32 D;
} data32;

// ������Ʋ���״̬�������Ӧ������û�����óɹ�, Ϊ1�����ɹ���Ϊ0����ʧ��
typedef struct
{
	boolean Ctrl_Mode_Fg;
	boolean Ctrl_Qty_Fg;
	boolean Aim_Pos_Fg;
	boolean CL_Accel_Fg;
	boolean CL_Decel_Fg;
} ctrl_state;


// ������״̬�Ľṹ�����
typedef struct
{
    // ����Ŀ������͹���ֵ
	data16 Aim_Vel;			// תÿ��
    data32 Aim_Pos;			// ���ת����λ�ã�һ��λ�ö�Ӧһ��λ��ϵ��
    data16 Aim_Cur;			// ��������,ʵ�ʵ�����/������ϵ��
    uint16 Aim_Accel;		// �ٶȱջ�����-���ٶ�
    uint16 Aim_Decel;		// �ٶȱջ�����-���ٶ�
    uint16 Vel_Est;			// �ٶȲ���ֵ(����ֵ) uint16�͹��� sint16Ҳ����
    data32 Pos_Est;			// λ�ò���ֵ(����ֵ),ʵ��λ�ó�/��λ��ϵ��
//    data32 Relative_POS_Est;// ���λ�ò���ֵ(����ֵ)�����ֹͣ���λ�ü���ֵ����
    uint16 Cur_Est;			// ���������ֵ(����ֵ),ʵ�ʵ�����/������ϵ��

    // �������ģʽ(ֻд)
    mctrl_mode  MCtrl_Mode;		// �������ģʽ
    pos_mode    Pos_Mode;		// λ��ģʽ
    vel_uint    Vel_Uint;		// �ٶȵ�λ
    accel_uint  Accel_Uint;		// ���ٶȵ�λ

    // ���״̬(ֻ��)���������Զ����ö�ٱ������Ϳ��ܱ���
    uint8  Pos_State;			// λ�����״̬�� 0δ��ɣ�1���
    uint16 Max_Vel;				// ���ת��
    uint16 Bus_Cur;				// ĸ�ߵ���
    uint16 Bus_Volt;			// ĸ�ߵ�ѹ
    uint16 Tube_Temp;			// ���ʹ��¶�
    // uint8  Stall_State;			// ��ת״̬
    uint8  Error_State;			// �������

// ���CAN����ʵʱ���йر�����MCU��Ȼ���������������˿���������ǲ�֪�������������û����ȷ���յ������Զ�������Щ����
    ctrl_state Ctrl_State;		// ������Ʋ�������״̬
    mctrl_mode MCtrl_Mode_Real;	// ������ʵ�ʵ������ģʽ
    data16	   Aim_Vel_Real;	// ������Ŀ���ٶȵ���ʵֵ
    data32     Aim_Pos_Real;	// ������Ŀ��λ�õ���ʵֵ
//    data16     Aim_Cur_Real;	// ������Ŀ���������ʵֵ
    uint16 	   Aim_Accel_Real;	// ������Ŀ����ٶȵ���ʵֵ
    uint16 	   Aim_Decel_Real;	// ������Ŀ����ٶȵ���ʵֵ

} motor;

// ������������Ӧ���ģ�����CAN�������ݽ��н���
void CAN0Rx_Respond_Process(CAN_Message *msg);
void motor_control_param_callback(motor_index mIndex, uint16 param_index);
void get_motor_state_param(motor_index mIndex, uint16 param_index, CAN_Message *msg);

// ����������������������������ת�٣�ת��Ϊ������ת��
void CAN0Tx_Request_Process(void);
void send_request_msg(CAN_Message *txmsg);

// PC
void CAN0Rx_Process_For_PC(CAN_Message *msg);
void switch_canmsgobj_send_2pc(CAN_Message *msg, uint8 sw);
void send_chassis_states_2pc(void);

// ���������ʼ����ͨ��CAN�������õ���Ŀ�����
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
