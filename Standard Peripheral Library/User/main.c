#include "main.h"

uint8_t MPU_State;

int main(void)
{
	float pitchx, rolly,  yawz;
	
	OLED_Init();
	OLED_Clear();
	MPU6050_Init();
	MPU_State=DMP_Init();
	OLED_Clear();
	
	OLED_ShowString(5, 0, "X:", OLED_8X16);
	OLED_ShowString(5, 20, "Y:", OLED_8X16);
	OLED_ShowString(5, 40, "Z:", OLED_8X16);
	OLED_Update();
	while (1)
	{
		read_dmp(&rolly, &pitchx, &yawz);
		OLED_ShowSignedNum(25, 0, pitchx, 4, OLED_8X16);
		OLED_ShowSignedNum(25, 20, rolly, 4, OLED_8X16);
		OLED_ShowSignedNum(25, 40, yawz, 4, OLED_8X16);
		OLED_Update();
	}
}
