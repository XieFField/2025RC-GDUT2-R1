/**
 * @file motor.h
 * @author Yang JianYi (287643517@qq.com) 、 Wu Jia
 * @brief 电机调用函数，每款电机驱动器都已经封装成一个类，通过调用类的函数来实现电机的控制。包括C610、C620、GM6020、VESC等。
 *        如果要新增电机类，请务必继承Motor_Base类，以保证接口的统一性。
 *        考虑增加达妙电机的驱动文件。
 *
 * ************************************************************************************************************
 * 使用说明：
 *   Motor_SendMsgs函数自动包含了所有电机的CAN报文处理函数，只需要调用Motor_SendMsgs函数即可。
 *   电机的回馈数据需要在service_communication.cpp文件中的CAN接收函数中进行处理。调用对应的update函数。
 *   例如：在CAN1的接收函数中，调用M3508.update函数，传入接收到的can_rx_data数据。
 * 
 *   电机的ID号需要在创建电机对象的时候进行设置。
 * 
 * 1.大疆系电机，创建一个PID，将PID计算后的结果赋值给Out(单位是mA)，然后调用Motor_SendMsgs函数发送数据。
 * 2.VESC电机，创建一个VESC类，先选定模式(VESC_MODE)，然后将Out(注意！！！，由于每种模式的不同，Out的单位也不同)赋值后，
 *   调用Motor_SendMsgs函数发送数据。
 *   SET_eRPM: Out单位是erpm。erpm = rpm * pole_pairs
 *   SET_CURRENT: Out单位是mA
 *   SET_DUTY: Out单位是0.01%
 *   SET_POS: 单位应是度，这里虽然流出了接口。但是最好还是不要使用，VESC的位置控制不太稳定(这是由于电机本身的编码器精度而决定)。
 *            范围只有0-360度，而且不支持负值(在上位机是只能测得0~360°)。
 *   SET_BRAKE: Out单位是mA
 * 3.达妙电机，创建一个DM_Driver类，此处的接口不适用Out。改为Vel_Out和Pos_Out，分别代表速度和位置。然后调用Motor_SendMsgs函数发送数据。
 *   CMD_MOTOR_SPEED: 速度控制，需要设置Vel_Out
 *   CMD_MOTOR_POSITION: 位置控制, 需要设置Pos_Out和Vel_Out
 *   CMD_MOTOR_ENABLE: 使能， 使能后电机才能运行。在程序初始化就可以开始使能。建议在service_config.cpp中进行调用。
 *   CMD_MOTOR_DISABLE: 失能
 *   CMD_MOTOR_ZERO_SET: 零点标定
 *
 * *************************************************************************************************************
 * @version 0.1 将原本两个电机驱动接口合并为一个，增加了达妙电机的驱动接口
 * @date create: 2024-05-16     update: 2024-7-19
 * 
 * @version 0.2 修复了大疆电机ID号的bug，增加了电机ID号的检查函数
 * @date create: 2024-05-16     update: 2025-5-9
 * @version 0.3 修复了SendMsg函数中VESC和DM的ID判断问题
 * @date create: 2024-05-16     update: 2025-5-13
 *
 */
#pragma once

#include <stdint.h>
#include "drive_can.h"
#include "drive_can.h"
#include "data_pool.h"
#include "tool.h"

#ifdef __cplusplus

/**
 * @brief 本杰明电调的模式使用枚举
 */
typedef enum VESC_MODE
{
    SET_NULL,   //无
    SET_eRPM,   //电气转速闭环模式
    SET_CURRENT,//电流闭环模式
    SET_DUTY,   //占空比模式 
    SET_POS,    //位置模式，不推荐使用
    SET_BRAKE,  //刹车，作用不如SET_eRPM
} VESC_MODE;

typedef enum MOTOR_FLAG
{
    RM_MOTOR,   //RM电机
    VESC_MOTOR, //本杰明电调
    DM_MOTOR,   //达妙电机
    NONE_MOTOR,
} MOTOR_FLAG;


//限幅函数
template <typename T>
void motor_constraint(T *val, T min, T max)
{
    if (*val > max)
    {
        *val = max;
    }
    else if (*val < min)
    {
        *val = min;
    }
}


class Motor_Base
{
public:
    Motor_Base(uint8_t id) : ID(id) {}
    virtual ~Motor_Base() {}
    const uint8_t ID = 0;
    virtual MOTOR_FLAG GET_MOTOR_FLAG() const { return NONE_MOTOR; }
    virtual void update(uint8_t can_rx_data[]) = 0;
    virtual bool check_id(uint32_t StdID) const { return StdID == this->receive_id_init() + (uint8_t)ID; }
    float get_angle() const { return angle; }
    float get_encoder() const { return encoder; }
    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) = 0;
    float encoder_offset = 0;
    float Out = 0; /*!< Output ampere value that sent to motor */
protected:
    uint16_t encoder = 0;
    float angle = 0, last_encoder = 0;
    bool encoder_is_init = false;

    virtual uint32_t receive_id_init() const { return 0; };
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 1; }
    virtual void update_angle(uint8_t can_rx_data[])
    {
        encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
        if (encoder_is_init)
        {
            if (this->encoder - this->last_encoder < -4096)
                this->round_cnt++;
            else if (this->encoder - this->last_encoder > 4096)
                this->round_cnt--;
            else
            {
            }
        }
        else
        {
            encoder_offset = encoder;
            encoder_is_init = true;
        }

        this->last_encoder = this->encoder;
        int32_t total_encoder = round_cnt * 8192 + encoder - encoder_offset;
        angle = total_encoder / ENCODER_ANGLE_RATIO() / GET_MOTOR_DESCRIPTION();
    }

private:
    int32_t round_cnt = 0;
    virtual int16_t ENCODER_MAX() const { return 8192; }
    virtual float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }
    virtual float MAX_CURRENT() const { return 65535; }
};

class Motor_Speed : public Motor_Base
{
public:
    Motor_Speed(uint8_t id) : Motor_Base(id) {}
    virtual ~Motor_Speed() {}
    virtual int32_t get_speed() const { return this->speed; }
    virtual void update(uint8_t can_rx_data[]) = 0;

protected:
    int16_t speed = 0;
    virtual void update_speed(uint8_t can_rx_data[])
    {
        speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};


class RM_Common : public Motor_Speed
{
public:
    RM_Common(uint8_t id) : Motor_Speed(id) {}
    virtual MOTOR_FLAG GET_MOTOR_FLAG() const { return RM_MOTOR; }

protected:
    virtual ~RM_Common() {}
    virtual uint32_t receive_id_init() const { return 0x200; }
    virtual uint32_t send_id_high() const { return 0x1FF; }
    virtual uint32_t send_id_low() const { return 0x200; }
    virtual float MAX_CURRENT() const { return 10000; }

    void prepareCANMsg(CAN_TxMsg &CAN_TxMsg, int16_t current_out) const   
    {

        if (ID <= 4 && ID > 0)
        {
            CAN_TxMsg.data[ID * 2 - 2] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[ID * 2 - 1] = (uint8_t)current_out & 0xff;
            CAN_TxMsg.id = send_id_low();
            CAN_TxMsg.len = 8;
        }
        else if (ID <= 8 && ID > 4)
        {
            CAN_TxMsg.data[ID * 2 - 10] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[ID * 2 - 9] = (uint8_t)current_out & 0xff;
            CAN_TxMsg.id = send_id_high();
            CAN_TxMsg.len = 8;
        }
    }
};

class Motor_C610 : public RM_Common
{
public:
    virtual ~Motor_C610() {}
    virtual float MAX_CURRENT() const { return 10000; }
    Motor_C610(uint8_t id) : RM_Common(id) {}
    virtual void update(uint8_t can_rx_data[]) override
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
    }
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }

protected:
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 36; }

private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};

class Motor_C620 : public RM_Common
{
public:
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_C620(uint8_t id) : RM_Common(id) {}
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

    virtual void update(uint8_t can_rx_data[])
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }

protected:
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 19; }

private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};

class Motor_GM6020 : public RM_Common
{
public:
    virtual ~Motor_GM6020() {}
    virtual uint32_t recieve_id_init() const { return 0x204; }
    virtual uint32_t send_id_low() const { return 0x1ff; }
    virtual uint32_t send_id_high() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 3000; }
    uint16_t set_encoder_offset(uint16_t offset)
    {
        this->encoder_offset = offset;
        this->last_encoder = offset;
        this->encoder_is_init = true;
        return this->encoder;
    }
    Motor_GM6020(uint8_t id) : RM_Common(id) {}
    virtual void update(uint8_t can_rx_data[])
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        this->temperature = can_rx_data[6];
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        motor_constraint(&(this->Out), -MAX_CURRENT(), MAX_CURRENT());
        prepareCANMsg(CAN_TxMsg, (int16_t)(this->Out));
    }

    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};

class VESC : public Motor_Speed
{
public:
    virtual MOTOR_FLAG GET_MOTOR_FLAG() const { return VESC_MOTOR; }
    VESC(uint8_t id) : Motor_Speed(id) {} // 括号中为VESC的CAN ID
    virtual ~VESC() {}
    VESC_MODE Mode;
    void update_vesc(CAN_RxBuffer *Buffer)
    {
        ID_check = Buffer->header.ExtId & 0xff;
        if (ID_check == this->ID)
        {
            cmd = (Buffer->header.ExtId >> 8); // 获取对应的帧头
            update(Buffer->data);
        }
    }
    virtual int32_t get_speed() const { return this->speed; }
    int16_t get_tarque() const { return this->tarque; }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        CAN_TxMsg.len = 8;
        switch (this->Mode)
        {
        case SET_eRPM: // erpm = rpm * pole_pairs
        {
            int index = 0;
            CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_RPM << 8);
            _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
            break;
        }

        case SET_CURRENT:
        {
            int index = 0;
            CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_CURRENT << 8); // mA
            _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
            break;
        }

        case SET_DUTY: // 0.01%
        {
            int index = 0;
            CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
            _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)(this->Out * 100000), &index);
            break;
        }

        case SET_POS:
        {
            int index = 0;
            CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_POS << 8);
            _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out * 1000000, &index);
            break;
        }

        case SET_BRAKE:
        {
            int index = 0;
            CAN_TxMsg.id = this->ID | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
            _tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)this->Out, &index);
            break;
        }

        default:
            break;
        }
    }

protected:
    virtual void update(uint8_t can_rx_data[])
    {
        switch (cmd)
        {
        case CAN_PACKET_STATUS:
            update_speed(can_rx_data);
            break;
        case CAN_PACKET_STATUS_4:
            update_angle(can_rx_data);
            break;
        default:
            break;
        }
    }

    virtual void update_speed(uint8_t can_rx_data[])
    {
        index = 0;
        this->speed = _tool_buffer_get_int32(can_rx_data, &index);
        this->tarque = (float)_tool_buffer_get_int16(can_rx_data, &index) * 100.0f; // mA
        this->duty = (float)_tool_buffer_get_int16(can_rx_data, &index) / 1000.0f;
    }

    virtual void update_angle(uint8_t can_rx_data[])
    {
        angle = (float)(can_rx_data[6] << 8 | can_rx_data[7]) / 50.0f;
    }

private:
    uint8_t ID_check;
    uint16_t cmd;
    int index = 0;
    float tarque = 0;
    float duty = 0;
};

class DM_Driver : public Motor_Speed
{
public:
    DM_Driver(uint8_t id) : Motor_Speed(id) {} // 括号中为VESC的CAN ID
    virtual ~DM_Driver() {}
    virtual MOTOR_FLAG GET_MOTOR_FLAG() const { return DM_MOTOR; }
    virtual void update(uint8_t can_rx_data[]) override
    {
        if (can_rx_data[0] == this->ID)
        {
            p_int = (can_rx_data[1] << 8) | can_rx_data[2];         // 电机位置数据
            v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);  // 电机速度数据
            i_int = ((can_rx_data[4] & 0xF) << 8) | can_rx_data[5]; // 电机扭矩数据

            angle = DM_uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16); // 转子机械角度
            speed = DM_uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12); // 实际转子转速
            tarque = DM_uint_to_float(i_int, I_MIN, I_MAX, 12);        // 实际转矩电流
        }
    }

    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        switch (this->Motor_Status)
        {
        case CMD_MOTOR_ENABLE:
        {
            CAN_TxMsg.id = 0x100 + this->ID;
            CAN_TxMsg.len = 8;
            for (int i = 0; i < 7; i++)
                CAN_TxMsg.data[i] = 0xFF;
            CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_ENABLE;
            break;
        }

        case CMD_MOTOR_DISABLE:
        {
            CAN_TxMsg.id = 0x100 + this->ID;
            CAN_TxMsg.len = 8;
            for (int i = 0; i < 7; i++)
                CAN_TxMsg.data[i] = 0xFF;
            CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_DISABLE;
            break;
        }

        case CMD_MOTOR_ZERO_SET:
        {
            CAN_TxMsg.id = 0x100 + this->ID;
            CAN_TxMsg.len = 8;
            for (int i = 0; i < 7; i++)
                CAN_TxMsg.data[i] = 0xFF;
            CAN_TxMsg.data[7] = (uint8_t)CMD_MOTOR_ZERO_SET;
            break;
        }

        case CMD_MOTOR_SPEED:
        {
            uint8_t *vbuf;
            vbuf = (uint8_t *)&this->Vel_Out;

            CAN_TxMsg.id = 0x200 + (uint8_t)this->ID;
            CAN_TxMsg.len = 4;

            for (int i = 0; i < 4; i++)
                CAN_TxMsg.data[i] = *(vbuf + i);
            break;
        }

        case CMD_MOTOR_POSITION:
        {
            uint8_t *vbuf, *pbuf;
            vbuf = (uint8_t *)&this->Vel_Out;
            pbuf = (uint8_t *)&this->Pos_Out;

            CAN_TxMsg.id = 0x100 + (uint8_t)this->ID;
            CAN_TxMsg.len = 8;

            for (int i = 0; i < 4; i++)
                CAN_TxMsg.data[i] = *(pbuf + i);
            for (int i = 4; i < 8; i++)
                CAN_TxMsg.data[i] = *(vbuf + i - 4);
            break;
        }
        default:
            break;
        }
    }

    virtual int32_t get_speed() const { return this->speed; }
    int16_t get_tarque() const { return this->tarque; }
    virtual float get_angle() const { return this->angle; }
    DM_MOTORCMD Motor_Status;
    float Vel_Out = 0, Pos_Out = 0;

protected:
private:
    int p_int, v_int, i_int;
    int16_t tarque = 0;
    const float MIT_P_MIN = -12.5f, MIT_P_MAX = 12.5f;
    const float MIT_V_MIN = -500.0f, MIT_V_MAX = 500.0f;
    const float I_MAX = 18.0f, I_MIN = -18.0f;
};


/**
 * @brief 电机数据发送接口
 */

template <class Motor_Type, int N>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
{
    CAN_TxMsg can_txmsg_high={0}, can_txmsg_low={0};
    bool low = false;
    bool high = false;
    for (int i = 0; i < N; i++)
    {
        //判断电机类型
        if (motor[i].GET_MOTOR_FLAG() == RM_MOTOR)
        {
            if (motor[i].ID <= 4 && motor[i].ID > 0)
            {
                motor[i].CanMsg_Process(can_txmsg_low);
                low = true;
            }
            else if (motor[i].ID <= 8 && motor[i].ID > 4)
            {
                motor[i].CanMsg_Process(can_txmsg_high);
                high = true;
            }
        }

        if (motor[i].GET_MOTOR_FLAG() == VESC_MOTOR || motor[i].GET_MOTOR_FLAG() == DM_MOTOR)
        {
                motor[i].CanMsg_Process(can_txmsg_low);
                low = true;
        }
    }

    //将报文发送到任务中
    if (hcan == &hcan1)
    {
        if (low)
            xQueueSend(CAN1_TxPort, &can_txmsg_low, portMAX_DELAY);
        if (high)
            xQueueSend(CAN1_TxPort, &can_txmsg_high, portMAX_DELAY);
    }
    else if (hcan == &hcan2)
    {
        if (low)
            xQueueSend(CAN2_TxPort, &can_txmsg_low, portMAX_DELAY);
        if (high)
            xQueueSend(CAN2_TxPort, &can_txmsg_high, portMAX_DELAY);
    }
    
    //\/============================================================/\//
    
    /*用于测试解决大疆电机CAN帧被覆盖的问题*/
/*
    // 静态变量保存当前CAN消息状态（保证多次调用间数据不丢失）
    static CAN_TxMsg can_txmsg_low = {0};
    static CAN_TxMsg can_txmsg_high = {0};
    static bool low_initialized = false;
    static bool high_initialized = false;
    
    // 处理当前数组中的电机
    for (int i = 0; i < N; i++)
    {
        if (motor[i].GET_MOTOR_FLAG() == RM_MOTOR)
        {
            if (motor[i].ID <= 4 && motor[i].ID > 0)
            {
                // 只初始化一次
                if (!low_initialized)
                {
                    can_txmsg_low.id = 0x200;  // 低ID组的标准ID
                    can_txmsg_low.len = 8;
                    low_initialized = true;
                }
                
                // 只设置当前电机的位置，不影响其他位置
                uint8_t index = motor[i].ID * 2 - 2;
                int16_t current = (int16_t)(motor[i].Out);
                can_txmsg_low.data[index] = (uint8_t)(current >> 8);
                can_txmsg_low.data[index + 1] = (uint8_t)(current & 0xFF);
            }
            else if (motor[i].ID <= 8 && motor[i].ID > 4)
            {
                // 只初始化一次
                if (!high_initialized)
                {
                    can_txmsg_high.id = 0x1FF;  // 高ID组的标准ID
                    can_txmsg_high.len = 8;
                    high_initialized = true;
                }
                
                // 只设置当前电机的位置，不影响其他位置
                uint8_t index = motor[i].ID * 2 - 10;
                int16_t current = (int16_t)(motor[i].Out);
                can_txmsg_high.data[index] = (uint8_t)(current >> 8);
                can_txmsg_high.data[index + 1] = (uint8_t)(current & 0xFF);
            }
        }
        else if (motor[i].GET_MOTOR_FLAG() == VESC_MOTOR || motor[i].GET_MOTOR_FLAG() == DM_MOTOR)
        {
            // 立即处理并发送VESC/DM电机
            CAN_TxMsg can_txmsg = {0};
            motor[i].CanMsg_Process(can_txmsg);
            if (hcan == &hcan1)
            {
                xQueueSend(CAN1_TxPort, &can_txmsg, portMAX_DELAY);
            }
            else if (hcan == &hcan2)
            {
                xQueueSend(CAN2_TxPort, &can_txmsg, portMAX_DELAY);
            }
        }
    }

    // 在适当的时候调用发送函数（比如在机构任务的最后）
    // 或者添加一个显式的Motor_SendAll函数来触发发送
//\/============================================================/\//
*/
}

template <class Motor_Type>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type &motor)
{
    Motor_Type motor_arr[1] = {motor};
    Motor_SendMsgs(hcan, motor_arr);
}

// 新增函数：实际发送所有累积的CAN消息
// 不加上inline编译会说我多重定义，但我只有这里定义了这个函数，我也不清楚具体原因
/*
inline void Motor_SendAll(CAN_HandleTypeDef *hcan)
{
    static CAN_TxMsg can_txmsg_low = {0};
    static CAN_TxMsg can_txmsg_high = {0};
    static bool low_initialized = false;
    static bool high_initialized = false;

    if (low_initialized)
    {
     if (hcan == &hcan1)
     {
         xQueueSend(CAN1_TxPort, &can_txmsg_low, portMAX_DELAY);
     }
     else if (hcan == &hcan2)
     {
         xQueueSend(CAN2_TxPort, &can_txmsg_low, portMAX_DELAY);
     }
     low_initialized = false;
    }

    if (high_initialized)
    {
     if (hcan == &hcan1)
     {
         xQueueSend(CAN1_TxPort, &can_txmsg_high, portMAX_DELAY);
     }
     else if (hcan == &hcan2)
     {
         xQueueSend(CAN2_TxPort, &can_txmsg_high, portMAX_DELAY);
     }
     high_initialized = false;
    }
}
*/

#endif
