<<<<<<< Updated upstream
=======
<<<<<<< HEAD
>>>>>>> Stashed changes
# 2025RC-GDUT2-R1
## 挑战赛射球车代码项目

```cpp
在motor.h中的void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
template <class Motor_Type, int N>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
{
    CAN_TxMsg can_txmsg_high = {0}, can_txmsg_low = {0};    //这里的局部变量一定赋予初始值0
    ···········
    ···········
}
```
这里不给其赋初始值的话，会导致CAN发送的数据是乱码。
后续若出现电机不听使唤的情况，第一时间检查有关数据发送部分的代码。
```cpp
void CAN1_RxCallBack(CAN_RxBuffer *RxBuffer)
{
#if USE_CAN1_STDID
    if(RxBuffer->header.IDE==CAN_ID_STD)
    {
        switch (RxBuffer->header.StdId)
        {   
            case 0x201:
                chassis.WheelMotor[0].update(RxBuffer->data);
                break;

            case 0x202:
                chassis.WheelMotor[1].update(RxBuffer->data);
                break;
            
            case 0x203:
                chassis.WheelMotor[2].update(RxBuffer->data);
                break;
            
            case 0x205:
                launch.LauncherMotor[0].update(RxBuffer->data);
                break;

            case 0x206:
                launch.LauncherMotor[1].update(RxBuffer->data);
                break;
        }
    }
#else
    if(RxBuffer->header.IDE==CAN_ID_EXT)
    {   
        switch (RxBuffer->header.ExtId)
        {   
            
        }
    }
#endif
}
```
后续若要增加电机，或者修改电机id，务必检查CAN的回调函数是否需要修改。

**Updata：2025-05-11**
<<<<<<< Updated upstream
=======
=======
# 2025RC-GDUT2-R1
## 挑战赛射球车代码项目

```cpp
在motor.h中的void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
template <class Motor_Type, int N>
void Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
{
    CAN_TxMsg can_txmsg_high = {0}, can_txmsg_low = {0};    //这里的局部变量一定赋予初始值0
    ···········
    ···········
}
```
这里不给其赋初始值的话，会导致CAN发送的数据是乱码。
后续若出现电机不听使唤的情况，第一时间检查有关数据发送部分的代码。
```cpp
void CAN1_RxCallBack(CAN_RxBuffer *RxBuffer)
{
#if USE_CAN1_STDID
    if(RxBuffer->header.IDE==CAN_ID_STD)
    {
        switch (RxBuffer->header.StdId)
        {   
            case 0x201:
                chassis.WheelMotor[0].update(RxBuffer->data);
                break;

            case 0x202:
                chassis.WheelMotor[1].update(RxBuffer->data);
                break;
            
            case 0x203:
                chassis.WheelMotor[2].update(RxBuffer->data);
                break;
            
            case 0x205:
                launch.LauncherMotor[0].update(RxBuffer->data);
                break;

            case 0x206:
                launch.LauncherMotor[1].update(RxBuffer->data);
                break;
        }
    }
#else
    if(RxBuffer->header.IDE==CAN_ID_EXT)
    {   
        switch (RxBuffer->header.ExtId)
        {   
            
        }
    }
#endif
}
```
后续若要增加电机，或者修改电机id，务必检查CAN的回调函数是否需要修改。

**Updata：2025-05-11**
>>>>>>> bd15825501030eebe6a532cac77a093bb6230fa9
>>>>>>> Stashed changes
