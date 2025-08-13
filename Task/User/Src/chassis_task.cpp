/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务调用
 * @version 1.8218319532
 * @attention 由于和视觉建图，所以代码里保留了不少测试功能，可视情况修改
 * 
 * @version 0.2 视觉与position数据采样抉择
 * 
 * @version 0.3 以后这份代码将包含挑战赛和竞技赛两套坐标系以及状态机，投篮赛我将把接球机构的控制移除，
 *              只保留投篮机构的控制，防止出现操作手，也就是我的紧张而引起的错误操作。
 *              更换挑战赛和竞技赛的模式选择需要前往speed_action.h文件修改CHANGE_MODE的值
 *              
 *              模式判断可以直接通过灯带判断，若为挑战赛模式则为常态黄灯
 *                                         若为竞技赛模式则为常态蓝灯
 * 
 *              当为挑战赛模式时候，lora的双车通讯会被关闭；改为竞技赛模式才会打开
 * 
 * @attention   当值为1的时候为挑战赛模式(篮筐坐标(0.0f,0.0f)，启动时候车屁股抵住底板，面向对面篮筐，
 *              启动时候需先用激光重定位).
 *              当值为0的时候为竞技赛模式(篮筐坐标(0.0f,13.096f)，启动时候车屁股抵住底板，面向对面篮筐，
 *              启动时需先试用激光重定位).
 * 
 *              position的坐标系为左手系，即站在车启动位置看，x轴正方向为左方，y轴正方向为前方。
 *              
 *              目前气压大和气压小的球的出射速度不同，后面可以考虑打两套表，通过改宏定义的值来选择
 *              之后上场时候的一分钟调试时间就好似绝佳的测试机会，应当好好把握。
 * 
 * @version Final
 * @date    2025-8-13
 * @brief   删去一些先前的测试语句
 */
#include <cmath> // Add this at the top of the file if not already included
#include "chassis_task.h"
#include "speed_plan.h"
#include "shoot.h"
#include "position.h"
#include "drive_uart.h"
#include "LaserPositioning_Task.h"
#include "ViewCommunication.h"
#include "drive_uart.h"

extern float Laser_Y_return;
extern float Laser_X_return;

#define EXTRUSION_WITH_20MM  0 //值为1则启用20mm挤压量下的拟合，否则是23mm下的


Ws2812b_SIGNAL_T send_signal = SIGNAL_WAIT;

extern float receivey;
extern float receiveyaw;
float dribble_speed_E = 57250;
PID_T yaw_pid = {0};
PID_T omega_pid = {0};
PID_T point_X_pid = {0};
PID_T point_Y_pid = {0};
PID_T vision_pid = {0};
PID_T vision_yaw_pid = {0};
Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(1180.f,-1320.645996, 50000); //俯仰最大角度 推球最大角度 摩擦轮加速度限幅 shootacc rpm/s^2
CONTROL_T ctrl;
float target_speed = 63450;     //测试使用


SHOOT_JUDGEMENT_E shoot_judge = POSITION;

#if CHANGE_MODE
float HOOP_X = 0.000000000f;
float HOOP_Y = 0.000000000f;
#else

    #if TEST
        float HOOP_X = 0.000000000f;
        float HOOP_Y = 0.000000000f;
    #else

        float HOOP_X = 0.000000000f;
        float HOOP_Y = 13.096000000f;

    #endif

#endif


float auto_pitch = 0.0f;


ShootController SHOOT;  //投篮拟合对象
ShootController::Shoot_Info_E shoot_info = {0};
float pitch_level = 1;  //1 2 3 分别对应 近 中 远

bool shoot_lock = false;

#ifdef EXTRUSION_20MM 
// 模拟小仰角样条数据
const ShootController::SplineSegment smallPitchTable[] = {
    {-42999.4884f, 27008.1149f, 9081.7162f, 35500.0000f},
    {-42999.4884f, -81.5628f, 14736.2921f, 38200.0000f},
    {78313.0705f, -25881.2559f, 9543.7284f, 40800.0000f},
    {-57752.7935f,  21106.5864f, 8588.7945f, 42300.0000f},
    {65198.1036f,  -13545.0897f, 10101.0938f, 44400.0000f},
    {-53039.6207f, 25573.7724f, 12506.8303f, 46400.0000f},
    {-53039.6207f, -6250.0000f, 16371.5848f, 49500.0000f},
};

const float smallPitchDistances[] = {1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f};

// 模拟中仰角样条数据
const ShootController::SplineSegment midPitchTable[] ={
    {8333.3333f, -10000.0000f, 14666.6667f, 45000.0000f},
    {8333.3333f, -5000.0000f, 11666.6667f, 47600.0000f},
    {-16666.6667f, 0.0f, 10666.6667f, 49800.0000f},
    {20833.3333f, -10000.0000f, 8666.6667f, 51800.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
};

const float midPitchDistances[] = {2.6f, 2.8f, 3.0f, 3.2f, 3.4f, 3.6f};

 // 模拟大等仰角样条数据
const ShootController::SplineSegment largePitchTable[] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.2f, 0.0f, 0.0f, 0.0f},
    {1.4f, 0.0f, 0.0f, 0.0f},
    {1.6f, 0.0f, 0.0f, 0.0f},
    {1.8f, 0.0f, 0.0f, 0.0f},
    {2.0f, 0.0f, 0.0f, 0.0f},
    {2.2f, 0.0f, 0.0f, 0.0f},
    {2.4f, 0.0f, 0.0f, 0.0f}
};

const float largePitchDistances[] = {3.6, 3.8f, 4.0f, 4.2f, 4.4f, 4.6f, 4.8f};

#else
// 模拟小仰角样条数据 (7.28)数据

/*
const ShootController::SplineSegment smallPitchTable[] = {
    {-10730.5463f, 17688.3278f,  -1108.4437f, 34550.0000},
    
    {-10730.5463f,  11250.0000f, 4679.2219f, 34950.0000f},
    
    { 16152.7316f,  4811.6722f, 7891.5563f, 36250.0000f},
    
    {-66380.3803f, 14503.3112f, 11754.5530f,  38150.0000f},
    
    {99368.7894f,  -25324.9170f, 9590.2318f, 40550.0000f},
    
    {-81094.7774f, 34296.3567f, 11384.5198f, 42250.0000f},
    
    {62510.3204f,  -14360.5098f, 15371.6891f, 45250.0000f},
    
    {-106446.5040f,   23145.6824f, 17128.7237f, 48250.0000f},
    
    { 88275.6956f,  -40722.2200f, 13613.4162f, 51750.0000f},
    
    {-46656.2784f,  -15750.5696f, 7216.1372f,  55250.0000f},
    
    {-34241.3935f,   13259.0812f,  6717.8395f,  56450.0000f},
    
    {26116.1562f,  -7285.7550f, 7912.5047f, 58050.0000f},
    
    {-20223.2312f,  8383.9387f, 8132.1415f, 59550.0000f},
    
    {-20223.2312f,  -20223.2312f, 9058.9292f, 61350.0000f},
};

*/
/*
// 模拟小仰角样条数据 (7.30 数据)
const ShootController::SplineSegment smallPitchTable[] = {
    {3544.4309f, 7873.3414f,  1283.5545f, 33240.0000},
    
    { 3544.4309f,  10000.0000f, 4858.2228f, 33840.0000f},
    
    { -55222.1547f,  12126.6586f,  9283.554f, 35240.0000f},
    
    {92344.1880f, -21006.6343f,  7507.559f,  37140.0000f},
    
    {-101654.5972f,  34399.8785f, 10186.2082f, 38540.0000f},//
    
    {89274.2010f, -26592.8798f, 11747.6079f, 41140.0000f},//
    
    {-92942.2067f,  26971.6408f, 11823.3601f, 43140.0000f},//
    
    { 107494.6256f,   -28793.6832f,  11458.9516f, 45840.0000f},//
    
    { -92036.2959f,  35703.0922f, 12840.8334f, 47840.0000f},//
    
    {33150.5581f,  -19518.6854f,  16077.7148f,  51100.0000f},//
    
    { -20565.9363f,   371.6494f,   12248.3076f,  53800.0000f},//
    
    {11613.1873f,  -11967.9124f, 9929.0550f, 56100.0000f},//
    
    { 11613.1873f,  -5000.0000f, 6535.4725f, 57700.0000f},//
    
     { 11613.1873f,  -5000.0000f, 6535.4725f, 57700.0000f},//
     { 11613.1873f,  -5000.0000f, 6535.4725f, 57700.0000f},//
};
*/
// 模拟小仰角样条数据 (7.31)数据
/*
const ShootController::SplineSegment smallPitchTable[] = {
    {-11610.4228f, 18216.2537f,  -1178.8338f, 34550.0000f},//
    
    {-11610.4228f, 11250.0000f,  4714.4169f, 34950.0000f},//
    
    {  20552.1138f, 4283.7463f,  7821.1662f, 36250.0000f},//
    
    {-83098.0323f, 16615.0146f, 12000.918f,  38150.0000f},//
    
    {161840.0154f,  -33243.8048f, 8675.1603f, 40550.0000f},//
    
    { -195512.0294f, 63860.204f, 14798.4403f, 42250.0000f},//
    
    {120208.1021f,  -53447.0131f, 16881.0785f, 46200.0000f},//
    
    {-16570.3788f,   18677.8481f, 9927.2455f, 48400.0000f},// 2.4
    
    { -53926.5867f,  8735.6208f, 15409.9393f,  51000.0000f},// 
    
    { 57276.7256f,  -23620.3312f, 12432.9972f,  54000.0000f},//
    
    {-50180.3158f,  10745.7041f,  9858.0718f,  56000.0000f},//
    
    {18444.5374f,  -19362.4853f, 8134.7156f, 58000.0000f},//
    
    {13902.1660f,  -8295.7629f,  2603.0659f, 59000.0000f},//
    
    {13446.7984f,  45.5368f, 953.0207f, 59300.0000f},//
    {-5189.3597, 8113.6158, 2584.8512, 59600.0000 },//
    {-5189.3597, 5000.0000, 5207.5744,60400.0000 }, // 4.0 - 4.2
};
*/


// 模拟小仰角样条数据 (8.01)数据
const ShootController::SplineSegment smallPitchTable[] = {
    {-10769.4686f, 17711.6811f,  -1111.5575f, 34550.0000f},//
    
    {-10769.4686f, 11250.0000f,  4680.7787f, 34950.0000f},//
    
    {  16347.3429f,  4788.3189f,  7888.4425f, 36250.0000f},//1.4
    
    {-67119.9030f, 14596.7246f, 11765.4512f,  38150.0000f},//1.6
    
    { 102132.2690f,  -25675.2172f, 9549.7527f, 40550.0000f},//
    
    { -97659.1729f, 35604.1442f, 11535.5381f, 42250.0000f},//2.0
    
    {88504.4225f,  -22991.3595f, 14058.0950f, 45200.0000f},//
    
    {-100108.5171f,   30111.2940f, 15482.0819f, 47800.0000f},// 2.4
    
    { 49429.6460f, -29953.8163f, 15513.5774f,  51300.0000f},// 
    
    {14889.9331f,   -296.0287f,  9463.6084f,  53600.0000f},//2.8
    
    {-46489.3783f,  8637.9311f,  11131.9889f,  55600.0000f},//
    
    {21067.5801f,  -19255.6958f, 9008.4360f, 57800.0000f},//3.2
    
    {-280.9421f,  -6615.1478,  3834.2672f, 59000.0000f},//
    
    {17556.1884f, -6783.7131f, 1154.4951f, 59500.0000f},//

    {-5189.3597, 8113.6158, 2584.8512, 59600.0000 },//

    {-5189.3597, 8113.6158, 2584.8512, 59600.0000 },// 4.0 - 4.2
};

const float smallPitchDistances[] = {1.0f, 1.2f, 1.4f, 1.6f, 1.8f, 2.0f, 2.2f, 2.4f, 2.6f, 2.8f, 3.0f,
                                     3.2f, 3.4f, 3.6f, 3.8f, 4.0f, 4.2f};

// 模拟中仰角样条数据 
const ShootController::SplineSegment midPitchTable[] ={
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
};

const float midPitchDistances[] = {4.0f, 4.2f, 4.4f, 4.6f, 4.8f, 5.0f};

// 模拟大等仰角样条数据(估计用不上了)
const ShootController::SplineSegment largePitchTable[] = {
    {20833.3333f, 2500.0000f, 2500.0000f, 53300.0000f},
};

const float largePitchDistances[] = {10.8f};


/*单函数拟合*/

const ShootController::SplineSegment OnePitchTable = {-1955.6030, 13825.1428, -19276.2305, 41681.6278};



#endif

void LED_InfoSend(void);

/**
 * @brief 更新pitch_level，后续考虑封装到ShootController的类中，
 *        
 * @brief 懒得封了
 */
int UpdatePitchLevel(float distance, int current_level)
{
    // 俯仰切换临界值，来自各Pitch的距离样本点
    // const float SMALL_TO_MID = 3.6f;  // smallPitchDistances[6]
    // const float MID_TO_SMALL = 2.4f;  // smallPitchDistances[0]
    // const float MID_TO_LARGE = 3.8f;  // midPitchDistances[1]
    // const float LARGE_TO_MID = 3.6f;  // midPitchDistances[0]

    const float SMALL_TO_MID = smallPitchDistances[15];  // = 4.0f
    const float MID_TO_SMALL = midPitchDistances[0];  // = 4.0f

    switch (current_level)
    {
        case 1: // 小仰角
            if (distance > SMALL_TO_MID)
                return 2;
            return 1;

        case 2: // 中仰角
            if (distance < MID_TO_SMALL)
                return 1;
//            if (distance > MID_TO_LARGE)
//                return 3;
            return 2;

        default:
            return 1;
    }
}



int i = 0;
void Chassis_Task(void *pvParameters)
{
    
    
    static float lock_angle = 0.0f;

    RELOCATTION_E relocate_on = NOT;
    xQueueSend(LED_Port, &send_signal, pdTRUE);
    for(;;)
    {   
        /*用于测试*/

      if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
      {
        
        /*==底盘控制==*/
           if(ctrl.chassis_ctrl == CHASSIS_COM_MODE)
           {
               //普通控制模式
               chassis.Control(ctrl.twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_CALIBRA_MODE)
           {
               //激光校准模式
               //还没做
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_LOW_MODE) //低速模式
           {
            #if TEST
                ctrl.twist.linear.x = ctrl.twist.linear.x * (2.7 / 3.7)  * 0.2;
                ctrl.twist.linear.y = ctrl.twist.linear.y * (2.7 / 3.7) * 0.2;
            #else
                ctrl.twist.linear.x = ctrl.twist.linear.x * 0.7;
                ctrl.twist.linear.y = ctrl.twist.linear.y * 0.7;
            #endif
                ctrl.twist.angular.z = ctrl.twist.angular.z ;
                chassis.Control(ctrl.twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_OFF)
           {
               //底盘关闭
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           else if(ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET)
           {
                 ctrl.twist.linear.x = ctrl.twist.linear.x * 0.9;
                 ctrl.twist.linear.y = ctrl.twist.linear.y * 0.9;
                 ctrl.twist.angular.z = ctrl.twist.angular.z  ;
           }
           else if(ctrl.chassis_ctrl == CHASSIS_DRIBBLE_LOW)
           {
                ctrl.twist.linear.x = ctrl.twist.linear.x * 0.2;
                ctrl.twist.linear.y = ctrl.twist.linear.y * 0.2;
                chassis.Control(ctrl.twist);
           }
           else
           {
               Robot_Twist_t twist = {0};
               chassis.Control(twist);
           }
           /*=================================================================*/

           /*==俯仰控制==*/
           if(ctrl.pitch_ctrl == PITCH_HAND_MODE)
               launch.PitchControl(0);

           else if(ctrl.pitch_ctrl == PITCH_AUTO_MODE)
               launch.PitchControl(0);

           else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
               launch.PitchControl(701);
               
           else if(ctrl.pitch_ctrl == PITCH_RESET_MODE)
               launch.PitchControl(0);
               
           else if(ctrl.pitch_ctrl == PITCH_LOCK_MODE)
           {
                lock_angle = launch.LauncherMotor[0].get_angle();
                launch.PitchControl(lock_angle);
           }
           else if(ctrl.pitch_ctrl == PITCH_DRIBBLE_MODE)
               launch.PitchControl(700);

           else if(ctrl.pitch_ctrl == PITCH_DRIBBLE_RESET_MODE)
                launch.PitchControl(0);
           
           else
                launch.PitchControl(0);
           /*==================================================================*/
            
           if(ctrl.robot_crtl == SHOOT_MODE)
           {
                /*==射球控制==*/
                if(ctrl.friction_ctrl == FRICTION_OFF_MODE)
                {
                    launch.ShootControl(false,false,0);
                    shoot_lock = false;
                }
                else if(ctrl.friction_ctrl == FRICTION_ON_MODE)
                {
                    if(ctrl.shoot_ctrl == SHOOT_OFF)
                    {
                        //launch.ShootControl(false,true,target_speed);
                        launch.ShootControl(false,true,shoot_info.shoot_speed);
                        shoot_lock = false;
                    }
                    else
                    {
                       //launch.ShootControl(true,true,target_speed);
                       launch.ShootControl(true,true,shoot_info.shoot_speed);
                       shoot_lock = true;
                    }
                }
            }
           /*===================================================================*/
           else if(ctrl.robot_crtl == BALL_MODE)
           {
                if(ctrl.dribble_ctrl == DRIBBLE_OFF)
                    launch.DribbleControl(false, false, dribble_speed_E);

                if(ctrl.dribble_ctrl == DRIBBLE_ON)
                   launch.DribbleControl(true, false, dribble_speed_E);

                else if(ctrl.dribble_ctrl == DRIBBLE_CATCH_ON)
                   launch.DribbleControl(false, true, 0);

                else
                   launch.DribbleControl(false, false, dribble_speed_E);
           }
           /*接球机构控制*/
           if(ctrl.catch_ball == CATCH_OFF)
           {
                launch.Catch_Ctrl(false,-5100);
           }
           else if(ctrl.catch_ball == CATCH_ON)
           {
                launch.Catch_Ctrl(true,-5100);
           }
           else
           {
                launch.Catch_Ctrl(false,-5100);
               //CATCH_OFF 接球关闭
           }

           if(ctrl.laser_ctrl == LASER_CALIBRA_ON)
            {
                relocate_on = BY_LASAER;
                xQueueSend(Relocate_Port, &relocate_on, pdTRUE);
            }
            else if(ctrl.laser_ctrl == LASER_CALIBRA_OFF && ctrl.shoot_ctrl == SHOOT_ON 
                && _tool_Abs(receiveyaw) < 2.0 && ctrl.chassis_ctrl == CHASSIS_LOCK_TARGET && shoot_judge == VISION)
            {   
                relocate_on = BY_VISION;
                xQueueSend(Relocate_Port, &relocate_on, pdTRUE);
            }
            else
            {
                relocate_on = NOT;
                xQueueSend(Relocate_Port, &relocate_on, pdTRUE);
            }

            chassis.Motor_Control();
            launch.LaunchMotorCtrl(); 
            LED_InfoSend(); 
       }
       
        osDelay(1);
    }
}

void PidParamInit(void)
{       
    chassis.Pid_Param_Init(0, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(1, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 
    chassis.Pid_Param_Init(2, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10); 

    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(0,15.0f, 0.015f, 0.0f, 3000.0f, 8000.0f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(2,18.0f, 0.015f, 0.0f, 10000.0f, 10000.0f, 0);
    launch.Pid_Mode_Init(2,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(3,18.0f, 0.015f, 0.0f, 10000.0f, 10000.0f, 0);
    launch.Pid_Mode_Init(3,0.1f, 0.0f, false, true);

//    //用于控制目标角度的角速度pid
    pid_param_init(&yaw_pid, PID_Position, 2.5, 0.0f, 0, 0.12f, 360, 0.7, 0, 0.1);
    pid_param_init(&vision_yaw_pid, PID_Position, 2.5, 0.0f, 0, 0.12f, 360, 0.82f, 0, 0.055f);
    pid_param_init(&omega_pid, PID_Incremental, 1.5, 0.0f, 0, 0.065f, 360, 0.3, 0.0008,0.02);
    pid_param_init(&vision_pid, PID_Incremental, 1.5, 0.0f, 0, 0.065f, 360, 0.60f, 0.0008f, 0);
    
//	//用于控制半径大小的法向速度pid
    pid_param_init(&point_X_pid, PID_Position, 2.0, 0.0f, 0, 0.1f, 180.0f, 1.0f, 0.0f, 0.66f);
}



void LED_InfoSend(void)
{
    if(RealPosData.world_x == INFINITY || RealPosData.world_y == INFINITY 
        || (RealPosData.world_x == 0 && RealPosData.world_y == 0 && RealPosData.world_yaw == 0))
        // 无穷大、未成功接收数据报错
        send_signal = SIGNAL_FAIL;

    else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
        //接球
        send_signal = SIGNAL_CATCH;

    else if(ctrl.laser_ctrl == LASER_CALIBRA_ON)
        send_signal = SIGNAL_WAIT;

    else
    #if CHANGE_MODE
        send_signal = SIGNAL_SHOOT;
    #else
        send_signal = SIGNAL_NORMAL;
    #endif
    
    xQueueSend(LED_Port, &send_signal, pdTRUE);
}

float error_read = 0.0f;

void Shoot_JudgeTask(void *pvParameters)
{

    /*
    // 初始化大仰角样条数据
    SHOOT.Init(largePitchTable, largePitchDistances, sizeof(largePitchDistances)/sizeof(float), 3);
    */
    // 初始化中仰角样条数据
    //SHOOT.Init(midPitchTable, midPitchDistances, sizeof(midPitchDistances)/sizeof(float), 2);
    
    static float distance_error = 0.0f; //发射距离误差修正

    // 初始化小仰角样条数据
    SHOOT.Init(smallPitchTable, smallPitchDistances, sizeof(smallPitchDistances)/sizeof(float), 1);
    for(;;)
    {
        xQueueReceive(Shoot_ERROR_Port, &distance_error, pdTRUE);
        error_read = distance_error;
       if(receivey != 0)
       {
            shoot_judge = VISION;
            shoot_info.hoop_distance = receivey;
       }
       else
       {
           shoot_judge = POSITION;
           SHOOT.GetShootInfo(HOOP_X, HOOP_Y, RealPosData.world_x, RealPosData.world_y, &shoot_info);
       }
        xQueueSend(Shoot_Judge_Port, &shoot_judge, pdTRUE);

        /**
         * @brief 原先的三次样条拟合
         */
//        pitch_level = UpdatePitchLevel(shoot_info.hoop_distance + distance_error, pitch_level);
//        shoot_info.shoot_speed = SHOOT.GetShootSpeed(shoot_info.hoop_distance + distance_error, pitch_level);

         if(!shoot_lock)  
           // shoot_info.shoot_speed = SHOOT.GetShootSpeed_ByOne(shoot_info.hoop_distance + distance_error, &OnePitchTable); //旧点
            //shoot_info.shoot_speed = SHOOT.GetShootSpeed_Beyond(shoot_info.hoop_distance + distance_error); // 8.7 临时点
           // shoot_info.shoot_speed = SHOOT.GetShootSpeed_OnSite(shoot_info.hoop_distance  + distance_error); // 现场点
           shoot_info.shoot_speed = SHOOT.GetShootSpeed_After(shoot_info.hoop_distance + distance_error);      //8.8临时点
         if(shoot_info.shoot_speed < 33000)
         {
            shoot_info.shoot_speed = 35000;
         }

        if(pitch_level == 1)        //可变俯仰，已废弃
            auto_pitch = 0.0f;
        else if(pitch_level == 2)
            auto_pitch = 0.0f;
        else
            auto_pitch = 0.0f;
    }
    osDelay(1);
}



