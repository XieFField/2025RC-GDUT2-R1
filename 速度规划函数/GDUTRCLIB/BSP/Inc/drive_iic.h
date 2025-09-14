#include "stdint.h"
#include "stm32f4xx.h"
#include "drive_tim.h"

//IIC结构体
typedef struct 
{
	GPIO_TypeDef * IIC_GPIO_PORT;   //IIC端口
	uint32_t IIC_SCL_PIN;           //IIC_SCL引脚
	uint32_t IIC_SDA_PIN;           //IIC_SDA引脚
	uint32_t IIC_SCL_PIN_NUM;       //IIC_SCL引脚编号
	uint32_t IIC_SDA_PIN_NUM;	    //IIC_SDA引脚编号
}IIC_PIN_Typedef;


void xSDA_IN(IIC_PIN_Typedef *iic_pin);
void xSDA_OUT(IIC_PIN_Typedef *iic_pin);
void i2c_Delay(uint8_t nus);
void i2c_Start(IIC_PIN_Typedef *iic_pin);
void i2c_Stop(IIC_PIN_Typedef *iic_pin);
void xIIC_SCL(IIC_PIN_Typedef *iic_pin,const char x);
void xIIC_SDA(IIC_PIN_Typedef *iic_pin,const char x);
GPIO_PinState xREAD_SDA(IIC_PIN_Typedef *iic_pin);
void i2c_SendByte(IIC_PIN_Typedef *iic_pin, uint8_t _ucByte);
uint8_t i2c_ReadByte(IIC_PIN_Typedef *iic_pin, uint8_t ack);
uint8_t i2c_WaitAck(IIC_PIN_Typedef *iic_pin);
void i2c_Ack(IIC_PIN_Typedef *iic_pin);
void i2c_NAck(IIC_PIN_Typedef *iic_pin);
void i2c_Init(IIC_PIN_Typedef *iic_pin);
uint8_t i2c_Device_Write_Byte(IIC_PIN_Typedef *iic_pin, uint8_t addr, uint8_t reg, uint8_t data);
uint8_t i2c_Device_Read_Byte(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg);
uint8_t i2c_Device_Write_Len(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t i2c_Device_Read_Len(IIC_PIN_Typedef *iic_pin, uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
