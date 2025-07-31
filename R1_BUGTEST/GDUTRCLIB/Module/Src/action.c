#include "action.h"
#include <math.h>

ACTION_GL_POS ACTION_GL_POS_DATA = {0};
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0};
/*
  @使用方法：
  使用串口3的232通道进行通讯，
  action数据存放在ACTION_GL_POS_DATA[]中，如果要调用请外部申明
  在.h文件中需要填入安装误差
  转换到车中心的坐标存放在ROBOT_REAL_POS_DATA[]中，如果要调用请外部申明
  更新XY坐标时需要调用Update_X，Update_Y。并且在两个函数中间要delay10ms
  
  此文件用于action而非position
*/



uint32_t Action_UART3_RxCallback(uint8_t *buf, uint16_t len)
{
	uint8_t count = 0;
	uint8_t i = 0;
	
	
	union 
	{
		uint8_t data[24];
		float ActVal[6];
	} posture;
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == 0x0d)   //接收包头1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 1:
			{
				if (buf[i] == 0x0a) //接收包头2
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 2:
			{
				uint8_t j;
				
				
				if (i > len - 26)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 24; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 3:
			{
				if (buf[i] == 0x0a)  //接收包尾1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 4:
			{
				if (buf[i] == 0x0d)  //接收包尾2
				{	
					//在接收包尾2后才开始启动回调
					Update_Action_gl_position(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	
	
	return 0;
}


void Update_Action_gl_position(float value[6])
{
	
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;

	
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X = value[3]; 
	ACTION_GL_POS_DATA.POS_Y = value[4]; 
	ACTION_GL_POS_DATA.W_Z = value[5];


	
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;

	

   //差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;


	ROBOT_REAL_POS_DATA.world_w = ACTION_GL_POS_DATA.ANGLE_Z;
	//ROBOT_CHASSI.world_w = ROBOT_REAL_POS_DATA.world_w;


	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	ROBOT_REAL_POS_DATA.world_x = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_X * sinf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
	//ROBOT_CHASSI.world_x = ROBOT_REAL_POS_DATA.world_x;
	ROBOT_REAL_POS_DATA.world_y = ACTION_GL_POS_DATA.REAL_Y + INSTALL_ERROR_Y * cosf(ROBOT_REAL_POS_DATA.world_w * PI / 180.f);
	//ROBOT_CHASSI.world_y = ROBOT_REAL_POS_DATA.world_y;
}

//字符串拼接
void Stract (char strDestination[],char strSource[] , int num)
{
    int i = 0 , j = 0;
	  while(strDestination[i] != '\0') i++;
	  
	  for (j = 0; j < num ; j++)
	        {
			   strDestination[i++] = strSource[j];  
			}
}

//更新X坐标
void Update_X(float New_X)
{
    char Update_x[8] = {'A', 'C', 'T', 'X', 0, 0, 0, 0};
    
    union
    {
        float X;
        char data[4];
    } New_set;

    New_set.X = New_X;

    Stract(&Update_x[4], New_set.data, 4);

    HAL_UART_Transmit(&huart3, (uint8_t *)Update_x, 8, HAL_MAX_DELAY);
}
//更新Y坐标
void Update_Y(float New_Y)
{
    char Update_y[8] = {'A', 'C', 'T', 'Y', 0, 0, 0, 0};
    
    union
    {
        float Y;
        char data[4];
    } New_set;

    New_set.Y = New_Y;

    Stract(&Update_y[4], New_set.data, 4);

    HAL_UART_Transmit(&huart3, (uint8_t *)Update_y, 8, HAL_MAX_DELAY);
}



