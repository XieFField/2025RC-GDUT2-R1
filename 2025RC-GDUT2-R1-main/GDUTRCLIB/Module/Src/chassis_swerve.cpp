/**
 * @file Swerve_Chassis.cpp
 * @author Yang JinaYi (2807643517@qq.com)
 * @brief 舵轮底盘控制类，包括舵轮底盘的速度控制、PID参数初始化、电机控制等
 * @note 1)使用该文件，需要在main.c中调用Chassis_Pid_Init函数进行PID参数初始化，在任务循环中调用chassis.Control(twist)进行速度控制，调用chassis.Motor_Control()进行电机控制
 *       2)由于八期R2底盘采用GM6020作为舵向，VESC驱动轮向电机，因此需要为这个类创建两个对象，一个是Motor_GM6020，一个是VESC。并且需要在service_communication.cpp
 *         中的can接收函数更新电机的实时参数信息，包括速度、角度等。
 *       3)如果需要更改轮向电机类型，ChassisVel_Trans_MotorRPM函数中的电机极对数需要更改。(有人继承寄轩师兄的舵轮的话)
 * @version 0.1
 * @date 2024-04-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_swerve.h"
template <typename Type>
Type ABS(Type a)
{
    if(a<0)
        return -a;
    else
        return a;
}



/**
 * @brief 底盘控制函数
 * 
 * @param cmd_vel 底盘速度参数结构体
 */
void Swerve_Chassis::Control(Robot_Twist_t cmd_vel)
{
    static int32_t last_target_wheel_vel[4]={0};   //上一时刻给轮子的速度赋值
    static int32_t last_wheelmotor_speed[4]={0};    //上一时刻轮子的实际转速
    update_timeStamp();

    for(int i=0; i<4; i++)
    {
        if(Chassis_Safety_Check(25000)==true)
        {
            //底盘速度限幅
            cmd_vel_.linear.x = cmd_vel.linear.x>Speed_Max.linear.x?Speed_Max.linear.x:cmd_vel.linear.x;
            cmd_vel_.linear.y = cmd_vel.linear.y>Speed_Max.linear.y?Speed_Max.linear.y:cmd_vel.linear.y;
            cmd_vel_.angular.z = cmd_vel.angular.z>Speed_Max.angular.z?Speed_Max.angular.z:cmd_vel.angular.z;
            
            Velocity_Calculate(cmd_vel_,&swerve[i]);

            //使用加速度控制底盘速度,
            #if USE_VEL_ACCEL
            if(swerve[i].wheel_vel > 0 && swerve[i].wheel_vel >= last_target_wheel_vel[i])
                swerve[i].wheel_vel = last_target_wheel_vel[i] + accel_vel*dt*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
            else if (swerve[i].wheel_vel < 0 && swerve[i].wheel_vel <= last_target_wheel_vel[i])
                swerve[i].wheel_vel = last_target_wheel_vel[i] - accel_vel*dt*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
            else
            {}

            last_target_wheel_vel[i] = swerve[i].wheel_vel;
            #endif      

            //在底盘运动速度比较低时，才能进行后退。防止反冲电流过大
            if(last_wheelmotor_speed[i]*swerve[i].wheel_vel<0 && ABS(SwerveWheelMotor[i].get_speed())-1000>=0)
            {
                SwerveWheelMotor[i].Mode = SET_eRPM;
                SwerveWheelMotor[i].Out = 0;
            }
            else
            {
                SwerveWheelMotor[i].Mode = SET_eRPM;
                SwerveWheelMotor[i].Out = swerve[i].wheel_vel;
            }

            //电机速度赋值
            PID_Rudder_Speed[i].current = SwerveRudderMotor[i].get_speed();
            PID_Rudder_Pos[i].current = SwerveRudderMotor[i].get_angle();
            PID_Rudder_Pos[i].target = swerve[i].target_angle;
            PID_Rudder_Speed[i].target = PID_Rudder_Pos[i].Adjust();
            SwerveRudderMotor[i].Out = PID_Rudder_Speed[i].Adjust();
            last_wheelmotor_speed[i] = SwerveWheelMotor[i].get_speed();
        }

        if(Chassis_Safety_Check(25000)==false)
        {
            SwerveWheelMotor[i].Mode = SET_CURRENT;
            SwerveWheelMotor[i].Out = 0;
        }
    }
    
}


/**
 * @brief 底盘电机控制函数
 * 
 * @return int 
 */
int Swerve_Chassis::Motor_Control(void)
{
    Motor_SendMsgs(&hcan1, SwerveRudderMotor);

    //由于担心是一个任务同时发送太多can帧，防止阻塞严重，把can帧速度拉低。
    //方法比较简陋，希望后来者可以改善这个问题并做好封装
	static int send_flag=0;
    if(send_flag<1)
    {
        Motor_SendMsgs(&hcan2, SwerveWheelMotor[0]);
    }
    else if (send_flag>=1&&send_flag<2)
    {
        Motor_SendMsgs(&hcan2, SwerveWheelMotor[1]);
    }
    else if (send_flag>=2&&send_flag<3)
    {
        Motor_SendMsgs(&hcan2, SwerveWheelMotor[2]);
    }
    else if (send_flag>=3&&send_flag<4)
    {
        Motor_SendMsgs(&hcan2, SwerveWheelMotor[3]);
    }
    else
    {
        send_flag = -1;
    }

    send_flag++;
    return 0;
}


/**
 * @brief 底盘速度计算
 */
void Swerve_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve)
{
    float wheel_Vx=0, wheel_Vy=0;
    switch(swerve->num)
    {
        case 1:
            wheel_Vx = (cmd_vel.linear.x + Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y + Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 2:
            wheel_Vx = (cmd_vel.linear.x - Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y + Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 3:
            wheel_Vx = (cmd_vel.linear.x - Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y - Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 4:
            wheel_Vx = (cmd_vel.linear.x + Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y - Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        default:
            break;
    }

    swerve->wheel_vel = sqrt(wheel_Vx*wheel_Vx + wheel_Vy*wheel_Vy)*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
    swerve->target_angle = atan2f(wheel_Vy,wheel_Vx)/PI*180;   // -180~180
    
    //底盘速度赋值为0时，刹车
    if(ABS(cmd_vel.linear.x)-0.02f<=0&&ABS(cmd_vel.linear.y)-0.02f<=0&&ABS(cmd_vel.angular.z)-0.02f<=0)
    {
        //设置刹车电流   
        SwerveWheelMotor[swerve->num-1].Mode = SET_BRAKE;
        SwerveWheelMotor[swerve->num-1].Out = 10;

        //四个轮子均小于100erpm时，超过2s锁住底盘
        Chassis_Lock(swerve);
    }

    //底盘舵向的劣弧计算
    RudderAngle_Adjust(swerve); 
    swerve->now_angle = swerve->target_angle;
}


/**
 * @brief 舵向角度调整，保证舵向角度在一个周期，并进行劣弧计算
 * 
 * @param swerve 
 */
void Swerve_Chassis::RudderAngle_Adjust(Swerve_t *swerve)
{
    
    float error=0;
    int N1=0,N2=0;
    //保证电机的实时角度在一个360度的周期之内
    // N = (int)(swerve->now_angle/180.0f) - (int)(swerve->now_angle/360.0f);
    if(swerve->now_angle>=0)
        N1 = swerve->now_angle/360;
    else
        N1 = swerve->now_angle/360-1;

    if(swerve->target_angle>=0)
        N2 = swerve->target_angle/360;
    else
        N2 = swerve->target_angle/360-1;

    N = N1-N2;  //now_angle>0&&now_angle<180   ---->  N=0
                                                                            //now_angle>=180&&now_angle<360  ---->  N=1
                                                                            //now_angle>=360                 ---->  N=1
                                                                            //now_angle<=-360                 ---->  N=-1
                                                                            //now_angle>-360&&now_angle<=180  ---->  N=-1
                                                                            //now_angle>180&&now_angle<=0     ---->  N=0


    swerve->target_angle = swerve->target_angle + N*360.0f;
    swerve->real_angle = SwerveRudderMotor[swerve->num-1].get_angle() - N*360.0f;  //未经过劣弧计算的实际角度

    error = abs(swerve->target_angle - swerve->now_angle);

    if(swerve->target_angle < swerve->now_angle)
    {
        if(error > 270)
        {
            swerve->target_angle = swerve->target_angle + 360.0f;
        }
        else if(error > 90)
        {
            swerve->target_angle = swerve->target_angle + 180.0f;
            swerve->wheel_vel = -swerve->wheel_vel;
        }
    }
    else
    {
        if(error > 270)
        {
            swerve->target_angle = swerve->target_angle - 360.0f;
        }
        else if(error > 90)
        {
            swerve->target_angle = swerve->target_angle - 180.0f;
            swerve->wheel_vel = -swerve->wheel_vel;
        }
    }
}


/**
 * @brief 舵轮底盘安全检测，检测电机的最大电流是否超过设定值。电流值过大持续时间超过1s，蜂鸣器报警
 * 
 * @param Current_Max 
 * @return true 电流安全
 * @return false 电流过大
 */
bool Swerve_Chassis::Chassis_Safety_Check(float Current_Max)
{
    static int beep_flag = 0;
    static uint32_t start_time = 0, now_time=0, real_time=0;
    for(int i=0; i<4; i++)
    {
        if(SwerveWheelMotor[i].get_tarque() > Current_Max)
        {
            
            if(beep_flag==0)
            {
                start_time = get_systemTick()/1000;
                beep_flag = 1;
            }

            now_time = get_systemTick()/1000;
            if(now_time<start_time)
            {
                real_time = 0xFFFFFFFF-start_time+now_time;
            }
            else
            {
                real_time = now_time-start_time;
            }
            
            if(real_time>2000&&beep_flag==1)
            {
                Set_PwmFreq(&htim10, 5);
                Set_PwmDuty(&htim10, TIM_CHANNEL_1, 50);
                beep_flag = 2;
                return false;
            }
        }
    }

    return true;
}


void Swerve_Chassis::Chassis_Lock(Swerve_t *swerve)
{
    static int reset_flag=0;
    static int32_t stop_start_time=0;
    
    if(ABS(SwerveWheelMotor[swerve->num-1].get_speed())<100)
    {   
        if(reset_flag==0)
        {
            reset_flag = 1;
            stop_start_time = get_systemTick()/1000;
        }

        if(get_systemTick()/1000-stop_start_time>2000) 
        {
            switch(swerve->num)
            {
                case 1:
                    swerve->target_angle = theta/2;
                    break;
                case 2:
                    swerve->target_angle = -theta/2;
                    break;
                case 3:
                    swerve->target_angle = theta/2;
                    break;
                case 4:
                    swerve->target_angle = -theta/2;
                    break;
                default:
                    break;
            }
        }
        else
        {
            //保证底盘停止时，舵向电机不会转动
            if(ABS(swerve->wheel_vel)< 100)
                swerve->target_angle = swerve->now_angle-N*360.0f;
        }
    }
    else
    {
        //保证底盘停止时，舵向电机不会转动
        if(ABS(swerve->wheel_vel)<100)
            swerve->target_angle = swerve->now_angle-N*360.0f;
        reset_flag = 0;
    }
}


Robot_Twist_t Swerve_Chassis::Get_Robot_Speed(void)
{
    Robot_Twist_t real_twist;
    real_twist.linear.x = (SwerveWheelMotor[0].get_speed()*cos(swerve[0].real_angle*PI/180) + 
                           SwerveWheelMotor[1].get_speed()*cos(swerve[1].real_angle*PI/180) + 
                           SwerveWheelMotor[2].get_speed()*cos(swerve[2].real_angle*PI/180) + 
                           SwerveWheelMotor[3].get_speed()*cos(swerve[3].real_angle*PI/180))/
                           (4.0f * MotorRPM_Trans_ChassisVel(Wheel_Radius, 21));

    real_twist.linear.y = (SwerveWheelMotor[0].get_speed()*sin(swerve[0].real_angle*PI/180) +
                           SwerveWheelMotor[1].get_speed()*sin(swerve[1].real_angle*PI/180) +
                           SwerveWheelMotor[2].get_speed()*sin(swerve[2].real_angle*PI/180) +
                           SwerveWheelMotor[3].get_speed()*sin(swerve[3].real_angle*PI/180))/
                           (4.0f * MotorRPM_Trans_ChassisVel(Wheel_Radius, 21));
    
    real_twist.angular.z = (SwerveWheelMotor[0].get_speed()*cos(swerve[0].real_angle*PI/180) + 
                            SwerveWheelMotor[1].get_speed()*cos(swerve[1].real_angle*PI/180) - 
                            SwerveWheelMotor[2].get_speed()*cos(swerve[2].real_angle*PI/180) - 
                            SwerveWheelMotor[3].get_speed()*cos(swerve[3].real_angle*PI/180))/
                            (4.0f*Chassis_Radius*COS * MotorRPM_Trans_ChassisVel(Wheel_Radius, 21));
    return real_twist;
}


