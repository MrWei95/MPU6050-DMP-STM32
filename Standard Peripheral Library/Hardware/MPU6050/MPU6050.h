#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "main.h"

#define MPU6050_ADD 0XD0

#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

void MPU6050_WriteReg(uint8_t RegAdd,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAdd);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
					 int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);
uint8_t MPU6050_WriteData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t const *data);
uint8_t MPU6050_ReadData(uint8_t slave_addr, uint8_t reg_addr,uint8_t length,uint8_t *data);
void stm32_get_ms(uint32_t *stm);

#endif
