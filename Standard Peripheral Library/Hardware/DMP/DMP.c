#include "DMP.h"
#include <math.h>

#define q30  1073741824.0f

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static inline void run_self_test(void)
{
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }

//    /* Report results. */
//    test_packet[0] = 't';
//    test_packet[1] = result;
//    send_packet(PACKET_TYPE_MISC, test_packet);
}
uint8_t read_dmp(float *pitch, float *roll, float *yaw)
{
	uint8_t more;
	long quat[4];									// 四元数
	int16_t gyro[3], accel[3], sensors;
	unsigned long sensor_timestamp;					// 传感器时间戳
	
	float q0 = 1.0f,q1 = 0.0f,q2 = 0.0f,q3 = 0.0f;
	
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
		return 1;
	if(sensors & 0x100)
	{
		q0 = quat[0]/q30;
		q1 = quat[1]/q30;
		q2 = quat[2]/q30;
		q3 = quat[3]/q30;
		
		*pitch = asin(-2*q1*q3 + 2*q0*q2)* 57.3f;											// 四元数解算, 俯仰角(Pitch)-->绕着X轴旋转
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3f;	// 四元数解算, 翻滚角(Roll)-->绕着Y轴旋转
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3f;					// 四元数解算, 偏航角(Yaw)-->绕着Z轴旋转
		
		return 0;
	}
	
	else
		return 2;
}

uint8_t DMP_Init(void)
{
	uint8_t res = 0;
	uint8_t RES = 0;
	RES = mpu_init();
	if(RES == 0)
	{
		OLED_ShowString(0, 0, "Setting sensors", OLED_6X8);
		OLED_Update();
		res = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
		if(res)return 88;
		
		OLED_ShowString(0, 8, "Setting fifo", OLED_6X8);
		OLED_Update();
		res=mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);		// 设置FIFO
		if(res)return 2;
		
		OLED_ShowString(0, 16, "Setting rate", OLED_6X8);
		OLED_Update();
		res=mpu_set_sample_rate(100);							// 设置采样率
		if(res)return 3;
		
		OLED_ShowString(0, 24, "Setting dmp firmware", OLED_6X8);
		OLED_Update();
		res=dmp_load_motion_driver_firmware();					// 加载dmp固件
		if(res)return 4;
		
		OLED_ShowString(0, 32, "Setting orientation", OLED_6X8);
		OLED_Update();
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));// 设置陀螺仪方向
		if(res)return 5;
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
								DMP_FEATURE_TAP |
								DMP_FEATURE_ANDROID_ORIENT |
								DMP_FEATURE_SEND_RAW_ACCEL |
								DMP_FEATURE_SEND_CAL_GYRO |
								DMP_FEATURE_GYRO_CAL);
		if(res)return 6;
		
		OLED_ShowString(0, 40, "Setting fifo rate", OLED_6X8);
		OLED_Update();
		res=dmp_set_fifo_rate(50);		// 设置DMP输出速率(最大不超过200Hz)
		if(res)return 7;
		
		OLED_ShowString(0, 48, "Self Test", OLED_6X8);
		OLED_Update();
		run_self_test();
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 9; 
	}
//	else
//		return RES;
//	for(uint16_t rd=0; rd<300; rd++)				
//		{
//			//读取MPU内置DMP的姿态
//			read_dmp(&pitch,&roll,&yaw);
//			Delay_ms(20);
//		}
	return 0;
}
