#pragma once
#include <stdint.h>
#include "drive_uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usart.h"
#include "stdbool.h"
#include "drive_ws2812.h"

//ROS串口DMA接收缓数组存大小
#define ROS_UART_SIZE 25

#define ACTION_UART_SIZE 35
#define POSITION_UART_SIZE 35

#define LORA_UART_SIZE 35

// 激光测距串口DMA接收缓存数组存大小
#define LaserPositionin_UART_SIZE 15

//队列大小
#define CAN1_TxPort_SIZE 8
#define CAN2_TxPort_SIZE 8
#define UART_TxPort_SIZE 4
#define Recieve_ROS_Port_SIZE 4
#define Send_ROS_Port_SIZE 4
#define Chassia_Port_SIZE 4
#define Broadcast_Port_SIZE 2
#define LaserPositionin_Port_SIZE 1
#define Enable_LaserModuleTask_SIZE 1
#define Relocate_Port_SIZE 4

#define LED_Port_SIZE 20
#define VIEW_UART_SIZE 50

//can数据帧类型定义
#define USE_CAN1_STDID 1  //使用标准ID
#define USE_CAN2_STTID 0  //使用扩展ID

//舵轮底盘加速度开启
#define USE_VEL_ACCEL 1

//使用调试任务
#define USE_DEBUG_TASK 0

#define Ring_or_ATUO_MODE 1     //若为1则是运动学方程方案，0则为环方案

#ifdef __cplusplus
extern "C" {
#endif 

extern xQueueHandle Port;
extern xQueueHandle CAN1_TxPort;
extern xQueueHandle CAN2_TxPort;
extern xQueueHandle UART_TxPort;
extern xQueueHandle Recieve_ROS_Port;
extern xQueueHandle Send_ROS_Port;
extern xQueueHandle Chassia_Port;
extern xQueueHandle Broadcast_Port;
extern xQueueHandle Receive_LaserModuleData_1_Port;			// 激光测距模块1串口DMA接收队列
extern xQueueHandle Receive_LaserModuleData_2_Port;			// 激光测距模块2串口DMA接收队列
extern xQueueHandle Enable_LaserModule_Port;
extern xQueueHandle Relocate_Port;
extern xQueueHandle LED_Port;


extern uint8_t Uart3_Rx_Buff_for_action[ACTION_UART_SIZE];

extern uint8_t Uart2_Rx_Buff_for_lora[LORA_UART_SIZE];

extern uint8_t Uart3_Rx_Buff_for_position[POSITION_UART_SIZE];

extern uint8_t Uart6_Rx_Buff[LaserPositionin_UART_SIZE];		// 激光测距模块1串口DMA接收缓存数组
extern uint8_t Uart4_Rx_Buff[LaserPositionin_UART_SIZE];		// 激光测距模块2串口DMA接收缓存数组

extern uint8_t Uart1_Rx_Buff_for_view[VIEW_UART_SIZE];

extern uint8_t Laser_EorD;

extern bool relocate_signal;


typedef enum CHASSIS_STATUS
{
	OFF,
	INIT,
	ON
}CHASSIS_STATUS;

//机器人底盘运动参数结构体
typedef struct Robot_Twist_t
{
	struct {
		float x;
		float y;
		float z;
	} linear;

	struct {
		float x;
		float y;
		float z;
	} angular;

	struct{
		float column;
		float angle_set;
	} pitch;
}Robot_Twist_t;


typedef enum PLAYLIST
{
	STOP,
	ROBOT_START=1,
	START_ZONE,
	RESTATR_ZONE,
	GAME_OVER,

	PATH_ONE,
	PATH_TWO,
	TAKE_MODE,
	PUT_MODE,

	POS_ERROR,

	HAND_MODE,
	AUTO_MODE
}PLAYLIST;


typedef struct Robot_Status_t
{
	PLAYLIST robot_init;
	PLAYLIST path_mode;
	PLAYLIST sensor;
	PLAYLIST control_mode;
}Robot_Status_t;



//CAN发送数据结构体
typedef struct CAN_TxMsg
{
    uint8_t data[8];//数据数组，8位
    uint32_t id;	//ID
    uint8_t len;	//数据长度
}CAN_TxMsg;


//UART发送数据结构体
typedef struct UART_TxMsg
{
    UART_HandleTypeDef *huart;	//串口句柄
    uint16_t len;				//数据长度
    void* data_addr;			//数据地址，使用时把地址赋值给这个指针。数据强转为uint8_t
}UART_TxMsg;


//达妙电机状态参数枚举
typedef enum DM_MOTORCMD
{
	CMD_MOTOR_SPEED = 0x01,
	CMD_MOTOR_POSITION = 0x02,
	CMD_MOTOR_ENABLE = 0xFC,
	CMD_MOTOR_DISABLE = 0xFD,
	CMD_MOTOR_ZERO_SET = 0xFE,
}DM_MOTORCMD;


//VESC状态参数结构体，来源于VESC驱动源码
typedef uint32_t systime_t;

typedef struct {
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct {
	int id;
	systime_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	int id;
	systime_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	int id;
	systime_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	int id;
	systime_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;

//VESC驱动器的can命令枚举
typedef enum {
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_FILL_RX_BUFFER				= 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG			= 6,
	CAN_PACKET_PROCESS_RX_BUFFER			= 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER			= 8,
	CAN_PACKET_STATUS						= 9,
	CAN_PACKET_SET_CURRENT_REL				= 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL		= 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE		= 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL	= 13,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_PING							= 17,
	CAN_PACKET_PONG							= 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC			= 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES		= 20,
	CAN_PACKET_CONF_CURRENT_LIMITS			= 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS	= 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN		= 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN	= 24,
	CAN_PACKET_CONF_FOC_ERPMS				= 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS			= 26,
	CAN_PACKET_STATUS_5						= 27,
	CAN_PACKET_POLL_TS5700N8501_STATUS		= 28,
	CAN_PACKET_CONF_BATTERY_CUT				= 29,
	CAN_PACKET_CONF_STORE_BATTERY_CUT		= 30,
	CAN_PACKET_SHUTDOWN						= 31,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4			= 32,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8			= 33,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12			= 34,
	CAN_PACKET_IO_BOARD_DIGITAL_IN			= 35,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL	= 36,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM		= 37,
	CAN_PACKET_BMS_V_TOT					= 38,
	CAN_PACKET_BMS_I						= 39,
	CAN_PACKET_BMS_AH_WH					= 40,
	CAN_PACKET_BMS_V_CELL					= 41,
	CAN_PACKET_BMS_BAL						= 42,
	CAN_PACKET_BMS_TEMPS					= 43,
	CAN_PACKET_BMS_HUM						= 44,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT		= 45,
	CAN_PACKET_PSW_STAT						= 46,
	CAN_PACKET_PSW_SWITCH					= 47,
	CAN_PACKET_BMS_HW_DATA_1				= 48,
	CAN_PACKET_BMS_HW_DATA_2				= 49,
	CAN_PACKET_BMS_HW_DATA_3				= 50,
	CAN_PACKET_BMS_HW_DATA_4				= 51,
	CAN_PACKET_BMS_HW_DATA_5				= 52,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL			= 53,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL			= 54,
	CAN_PACKET_UPDATE_PID_POS_OFFSET		= 55,
	CAN_PACKET_POLL_ROTOR_POS				= 56,
	CAN_PACKET_NOTIFY_BOOT					= 57,
	CAN_PACKET_STATUS_6						= 58,
	CAN_PACKET_GNSS_TIME					= 59,
	CAN_PACKET_GNSS_LAT						= 60,
	CAN_PACKET_GNSS_LON						= 61,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP			= 62
}
CAN_PACKET_ID;



void DataPool_Init(void);

#ifdef __cplusplus
}
#endif 
