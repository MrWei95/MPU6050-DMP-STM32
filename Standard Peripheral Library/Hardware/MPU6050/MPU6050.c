#include "MPU6050.h"
//#include "i2c.h"
#include "Time.h"

struct tm Time;

void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 5000;															// 给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)						// 循环等待指定事件
	{
		Timeout --;															// 等待时，计数值自减
		if (Timeout == 0)													// 自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;															// 跳出等待，不等了
		}
	}
}

void MPU6050_WriteReg(uint8_t RegAdd,uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);										// 硬件I2C生成起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					// 等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADD, I2C_Direction_Transmitter);		// 硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	// 等待EV6
	
	I2C_SendData(I2C2, RegAdd);												// 硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			// 等待EV8
	
	I2C_SendData(I2C2, Data);												// 硬件I2C发送数据
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				// 等待EV8_2
	
	I2C_GenerateSTOP(I2C2, ENABLE);	
}
uint8_t MPU6050_ReadReg(uint8_t RegAdd)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);										// 硬件I2C生成起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					// 等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADD, I2C_Direction_Transmitter);		// 硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	// 等待EV6
	
	I2C_SendData(I2C2, RegAdd);												// 硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				// 等待EV8_2
	
	I2C_GenerateSTART(I2C2, ENABLE);										// 硬件I2C生成重复起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					// 等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADD, I2C_Direction_Receiver);			// 硬件I2C发送从机地址，方向为接收
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		// 等待EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);									// 在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C2, ENABLE);											// 在接收最后一个字节之前提前申请停止条件
	
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);				// 等待EV7
	Data = I2C_ReceiveData(I2C2);											// 接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);									// 将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}
uint8_t MPU6050_WriteData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t const *data)
{
	uint8_t i;
	
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2,(slave_addr<<1)|0,I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2,reg_addr);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	for(i=0;i<length;i++)
	{
		I2C_SendData(I2C2,data[i]);
		
		MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	}
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2,ENABLE);
	return 0;
}
uint8_t MPU6050_ReadData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data)
{
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2,(slave_addr<<1)|0,I2C_Direction_Transmitter);
	
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2,reg_addr);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2,ENABLE);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2,(slave_addr<<1)+1,I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	while(length){
		if(length == 1)
		{
			I2C_AcknowledgeConfig(I2C2,DISABLE);
			I2C_GenerateSTOP(I2C2,ENABLE);
			MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED);
			*data=I2C_ReceiveData(I2C2);
		}
		else
		{
			MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED);
			*data=I2C_ReceiveData(I2C2);
		}
		length--;
		data++;
	}
	I2C_AcknowledgeConfig(I2C2,ENABLE);
	I2C_GenerateSTOP(I2C2,ENABLE);
	return 0;
}

//void stm32_get_ms(unsigned long *stm)
void stm32_get_ms(uint32_t *stm)
{
//	*stm = mktime(&Time);
}


void MPU6050_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed=50000;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_OwnAddress1=0x00;
	I2C_Init(I2C2,&I2C_InitStruct);
	
	I2C_Cmd(I2C2,ENABLE);
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0X01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0X00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0X09);
	MPU6050_WriteReg(MPU6050_CONFIG,0X06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0X18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0X18);
}


void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
					 int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t DataH,DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}
