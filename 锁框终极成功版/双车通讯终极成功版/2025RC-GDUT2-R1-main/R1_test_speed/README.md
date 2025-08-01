<<<<<<< Updated upstream
# 2025RC-GDUT2-R1
## 挑战赛射球车代码项目

### CAN方面
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
后续若要增加电机，或者修改电机id，务必检查CAN的回调函数是否需要修改。

在大疆CAN帧发送部分，上半片必须四个电机电流数据赋值一起发送，下半片也是如此。当前代码框架中很难解决这个问题，唯一想到的方法是将所有电机实例化，即全局变量。2026rc时候再解决这个问题吧

### LORA模块方面
lora.c中做了宏定义LORA_ON，将其值改为1则启用LORA，改为0则禁用。在没有外接LORA模块时候应当禁用，否则初始化无法通过

