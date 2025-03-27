#ifndef _DMP_H_
#define _DMP_H_

#include "main.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"

uint8_t DMP_Init(void);
uint8_t read_dmp(float *pitch, float *roll, float *yaw);
static inline void run_self_test(void);

#endif
