/**
 ******************************************************************************
 * @file    drive_iic.c
 * @version V1.0
 * @date    2024-7-17
 * @brief   用gpio模拟i2c总线, 适用于STM32系列CPU。该模块不包括应用层命令帧，仅包括I2C总线基本操作函数。
 * @note    本驱动参考了ST官方I2C驱动，适用于STM32系列CPU。
 * 
 *  **********************************************************************************
 * @attention
 * 1)是否使用硬件定时器来实现延时函数，取决于USE_TIM_DELAY的值，如果USE_TIM_DELAY为1，则使用硬件定时器，否则使用软件延时。
 * 2)使用该驱动时，需要配置配置IIC_PIN_Typedef结构体
 * 
 *******************************************************************************
 * IIC 操作流程：主芯片发出一个start信号，然后发送出设备地址(7位地址跟1位读写位)
 *             然后等待设备的应答信号(在主机发送一个字节数据之后，从机在第9个脉冲周期进行应答，如果SDA为0，则表示应答，
 *             如果SDA=1，则表示无应答)，如果设备应答，主芯片就可以发送或者接收数据了。
 *             主设备发送一字节数据给从设备，并等待回应，每传输1字节数据，接收方要有一个回应信号(确定数据是否接收完成)
 *             然后再发送下一个字节，直到数据传输完成，最后发送一个stop信号。
 ******************************************************************************
*/
#include "drive_iic.h"
#define USE_TIM_DELAY 0
#define IIC_DELAY_CNT 20

#if USE_TIM_DELAY
#define i2c_Delay delay_us_nos
#else
/**
 * @brief 软件IIC延时函数
 * @param nus 延时时间, nus = 1接近于延时1us
 */
void i2c_Delay(uint8_t nus)
{
    uint8_t i2c_delay_time = 0;
    for(int i = 0; i < nus; i++)
    {
        i2c_delay_time = IIC_DELAY_CNT;
        for(int j = 0; j < i2c_delay_time; j++);
    }
}
#endif

/**
 * @brief CPU发起I2C总线启动信号
 * 
 * @param iic_pin IIC引脚结构体
 */
void i2c_Start(IIC_PIN_Typedef *iic_pin)
{
    /* SDA Output */
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 1);  
	xIIC_SCL(iic_pin, 1);
    i2c_Delay(1);

    /* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	xIIC_SDA(iic_pin, 0);  
	i2c_Delay(1);
	
	/* 钳住IIC总线，准备收发数据 */
	xIIC_SCL(iic_pin, 0);  
}


/**
 * @brief i2c停止信号
 * 
 * @param iic_pin 
 */
void i2c_Stop(IIC_PIN_Typedef *iic_pin)
{
    /* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
    xSDA_OUT(iic_pin);
    xIIC_SDA(iic_pin, 0);
    xIIC_SCL(iic_pin, 1);
    i2c_Delay(1);

    /* 停止发送IIC信号 */
    xIIC_SDA(iic_pin, 1);
}

/**
  * @brief  iic initialization
  * @param  hiic: iic handler
  * @retval void
  */
void i2c_Init(IIC_PIN_Typedef *iic_pin)
{ 
	/* SCL High */
	xIIC_SCL(iic_pin,1);

	/* SDA High */  
	xIIC_SDA(iic_pin,1);
}

/**
 * @brief 设置SDA为输入模式
 * 
 * @param iic_pin 
 */
void xSDA_IN(IIC_PIN_Typedef *iic_pin)              		    
{
    /*清除引脚的模式位*/
	iic_pin->IIC_GPIO_PORT->MODER&=~(3<<(iic_pin->IIC_SDA_PIN_NUM*2));
    /*设置引脚为输入模式*/
	iic_pin->IIC_GPIO_PORT->MODER|= (0<<(iic_pin->IIC_SDA_PIN_NUM*2));	    
}


/**
 * @brief 设置SDA为输出模式
 * 
 * @param iic_pin 
 */
void xSDA_OUT(IIC_PIN_Typedef *iic_pin)             		    
{
    /*清除引脚的模式位*/
	iic_pin->IIC_GPIO_PORT->MODER&=~(3<<(iic_pin->IIC_SDA_PIN_NUM*2));
    /*设置引脚为输入模式*/
	iic_pin->IIC_GPIO_PORT->MODER|= (1<<(iic_pin->IIC_SDA_PIN_NUM*2)); 		
}


/**
 * @brief SCL输出高低电平
 * 
 * @param iic_pin 
 * @param x 0为低电平，1为高电平
 */
void xIIC_SCL(IIC_PIN_Typedef *iic_pin,const char x)        
{   
    if(x!=0)
        HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SCL_PIN,GPIO_PIN_SET);
	else
        HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SCL_PIN,GPIO_PIN_RESET);
}


/**
 * @brief SDA输出高低电平
 * 
 * @param iic_pin 
 * @param x 0为低电平，1为高电平
 */
void xIIC_SDA(IIC_PIN_Typedef *iic_pin,const char x)        
{
    if(x!=0)
        HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SDA_PIN,GPIO_PIN_SET);
    else
		HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SDA_PIN,GPIO_PIN_RESET);	
}


/**
 * @brief 读SDA口线状态 
 * 
 * @param iic_pin 
 * @return GPIO_PinState 
 */
GPIO_PinState xREAD_SDA(IIC_PIN_Typedef *iic_pin)
{
    GPIO_PinState bitstatus;
	
    if((iic_pin->IIC_GPIO_PORT->IDR & iic_pin->IIC_SDA_PIN) != (uint32_t)GPIO_PIN_RESET)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }
    return bitstatus;	
}


/**
 * @brief 向I2C总线发送一个字节
 * @note CPU向I2C总线设备发送8bit数据
 * @param iic_pin IIC引脚结构体
 */
void i2c_SendByte(IIC_PIN_Typedef *iic_pin, uint8_t _ucByte)
{
    uint8_t i;
    xSDA_OUT(iic_pin); /* SDA Output */
    xIIC_SCL(iic_pin, 0);   

    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++)
    {
        xIIC_SDA(iic_pin, (_ucByte&0x80)>>7);
        i2c_Delay(1);
        xIIC_SCL(iic_pin,1);
        i2c_Delay(1);
        xIIC_SCL(iic_pin,0);

        if (i == 7)
        {
            xIIC_SDA(iic_pin,0); // 释放总线
        }
        _ucByte <<= 1; /* 左移一个bit */
        i2c_Delay(1);
    }
}


/**
 * @brief 从I2C总线读取一个字节
 * @note CPU从I2C总线设备读取8bit数据
 * @param iic_pin IIC引脚结构体
 * @param ack 0表示发送不应答，1表示发送应答
 * @return uint8_t 读到的数据
 */
uint8_t i2c_ReadByte(IIC_PIN_Typedef *iic_pin, uint8_t ack)
{
    uint8_t i, value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    xSDA_IN(iic_pin);
    
    for (i = 0; i < 8; i++)
    {
        xIIC_SCL(iic_pin, 0);
		i2c_Delay(1);
		xIIC_SCL(iic_pin, 1);
        value <<= 1;

        if (xREAD_SDA(iic_pin))
        {
            value++;
        }
        i2c_Delay(1);
    }

    if(!ack)
	{
		/* 发送不应答信号 */
		i2c_NAck(iic_pin);
	}
	else 
	{
		/* 发送应答信号 */
		i2c_Ack(iic_pin);
	}

    return value;
}


/**
 * @brief CPU产生一个时钟，并读取器件的ACK应答信号
 * 
 * @param iic_pin 
 * @return uint8_t 返回0表示正确应答，1表示无器件响应
 */
uint8_t i2c_WaitAck(IIC_PIN_Typedef *iic_pin)
{
    uint8_t ucErrTime=0;

    xSDA_IN(iic_pin);       /* SDA Input */
    xIIC_SDA(iic_pin, 1);   /* CPU释放SDA总线 */
    i2c_Delay(1);
    
    xIIC_SCL(iic_pin, 1);   /* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    i2c_Delay(1);

    while(xREAD_SDA(iic_pin))   /* CPU读取SDA口线状态 */
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            i2c_Stop(iic_pin);
            return 1;
        }
    }
    xIIC_SCL(iic_pin,0);   /* CPU驱动SCL = 0 */
    i2c_Delay(1);
    return 0;
}

/**
 * @brief 产生一个ACK信号
 * 
 * @param iic_pin 
 */
void i2c_Ack(IIC_PIN_Typedef *iic_pin)
{
    xIIC_SCL(iic_pin, 0);   
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 0);  
	i2c_Delay(1);

	xIIC_SCL(iic_pin, 1);  
	i2c_Delay(1);
    
	xIIC_SCL(iic_pin, 0);  
}


/**
 * @brief 产生一个NACK信号
 * 
 * @param iic_pin 
 */
void i2c_NAck(IIC_PIN_Typedef *iic_pin)
{
    xIIC_SCL(iic_pin, 0); 
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 1);  
	i2c_Delay(1);

	xIIC_SCL(iic_pin, 1);  
	i2c_Delay(1);

	xIIC_SCL(iic_pin, 0);  
}


/**
  * @brief  iic 向设备的寄存器写入一个字节
  * @param  iic_pin: iic结构体
  * @param  addr: 设备地址
  * @param  reg: 设备寄存器地址
  * @param  data: 发送的数据
  * @retval 0,success
  *         1,fail
  */
uint8_t i2c_Device_Write_Byte(IIC_PIN_Typedef *iic_pin, uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_Start(iic_pin);
	
	/* 发送设备地址，将地址左移一位(IIC地址一般是7位，7位设备地址，后接1位读/写位) */
	i2c_SendByte(iic_pin, (addr<<1)|0);

    /* 判读设备是否应答 */
	if(i2c_WaitAck(iic_pin))	    
	{
		i2c_Stop(iic_pin);
		return 1;
	}

	/* 发送寄存器地址 */
    i2c_SendByte(iic_pin,reg);	
    i2c_WaitAck(iic_pin);		
        
    /* 发送数据到寄存器 */    
	i2c_SendByte(iic_pin,data);    
	if(i2c_WaitAck(iic_pin))	    
	{
		i2c_Stop(iic_pin);
		return 1;
	}
    i2c_Stop(iic_pin);
	return 0;
}



/**
  * @brief  iic 从设备的寄存器读取一个字节
  * @param  hiic: iic结构体
  * @param  addr: 设备地址
  * @param  reg: 设备寄存器地址
  * @retval 读到的数据      
  */
uint8_t i2c_Device_Read_Byte(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg)
{
	uint8_t res;
    i2c_Start(iic_pin);
	
	/* 发送设备地址，将地址左移一位(IIC地址一般是7位，7位设备地址，后接1位读/写位) */
	i2c_SendByte(iic_pin,(addr<<1)|0);
	i2c_WaitAck(iic_pin);		
  
	/* 发送寄存器地址 */
    i2c_SendByte(iic_pin,reg);	
    i2c_WaitAck(iic_pin);		
    i2c_Start(iic_pin);
	
	/* 发送设备地址和读命令 */
	i2c_SendByte(iic_pin,(addr<<1)|1);
    i2c_WaitAck(iic_pin);		
  
	/* 读取数据并发送不应答信号 */
	res = i2c_ReadByte(iic_pin,0);
	
	/* 停止信号 */
    i2c_Stop(iic_pin);			
	return res;
}

/**
  * @brief  iic 向设备的寄存器写入多个字节
  * @param  hiic: iic结构体
  * @param  addr: 设备地址
  * @param  reg: 设备寄存器地址
  * @param  len: 写入的字节长度
  * @param  buf: 写入的数据
  * @retval 0,success
  *         1,fail
  */
uint8_t i2c_Device_Write_Len(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;
    i2c_Start(iic_pin);
	
	/* 发送设备地址，将地址左移一位(IIC地址一般是7位，7位设备地址，后接1位读/写位) */
	i2c_SendByte(iic_pin,(addr<<1)|0);
	if(i2c_WaitAck(iic_pin))	
	{
		i2c_Stop(iic_pin);
		return 1;
	}
	
	/* 发送寄存器地址 */
    i2c_SendByte(iic_pin,reg);	
    i2c_WaitAck(iic_pin);		
	for(i=0;i<len;i++)
	{
		/* 发送数据到寄存器 */ 
		i2c_SendByte(iic_pin,buf[i]);	
		if(i2c_WaitAck(iic_pin))		
		{
			i2c_Stop(iic_pin);
			return 1;
		}
	}
    i2c_Stop(iic_pin);
	return 0;
}

/**
  * @brief  iic 连续读取多个字节
  * @param  hiic: iic 结构体
  * @param  addr: 设备地址
  * @param  reg: 设备寄存器地址
  * @param  len: 读取的字节长度
  * @param  buf: 读取的数据
  * @retval 0,success
  *         1,fail
  */
uint8_t i2c_Device_Read_Len(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
 	i2c_Start(iic_pin);
	
	/* 发送设备地址，将地址左移一位(IIC地址一般是7位，7位设备地址，后接1位读/写位) */
	i2c_SendByte(iic_pin,(addr<<1)|0);
	if(i2c_WaitAck(iic_pin))	
	{
		i2c_Stop(iic_pin);
		return 1;
	}
	
	/* 发送寄存器地址 */
    i2c_SendByte(iic_pin,reg);	
    i2c_WaitAck(iic_pin);		
    i2c_Start(iic_pin);
	
	/* 发送设备地址和读命令 */
	i2c_SendByte(iic_pin,(addr<<1)|1);
    i2c_WaitAck(iic_pin);		
	while(len)
	{
		/* 接收完最后一个字节数据后，发送不应答信号 */
		if(len==1)
            *buf=i2c_ReadByte(iic_pin,0);
		else 
            *buf=i2c_ReadByte(iic_pin,1);	

		len--;
		buf++;
	}

	/* 停止信号 */
    i2c_Stop(iic_pin);	
	return 0;
}
