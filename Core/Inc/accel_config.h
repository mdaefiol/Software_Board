/*
 * accel_config.h
 *
 *  Created on: May 9, 2023
 *      Author: mariana.daefiol
 */

#ifndef INC_ACCEL_CONFIG_H_
#define INC_ACCEL_CONFIG_H_

#include "stm32f1xx_hal.h"

typedef struct
{
	int16_t Ax, Ay, Az;     // acelerometro 1
	int16_t Ax2, Ay2, Az2;	// acelerometro 2
} AccelData;

typedef struct
{
	int16_t Gx, Gy, Gz; 	// velocidade angular 1
	int16_t Gx2, Gy2, Gz2; 	// velocidade angular 2
} GyroData;

// ACELERÔMETRO
#define MPU6050_ADDR 		0xD0
#define SMPLRT_DIV_REG 		0x19
#define GYRO_CONFIG_REG 	0x1B
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_XOUT_H_REG 	0x43
#define PWR_MGMT_1_REG 		0x6B
#define WHO_AM_I_REG 		0x75

void MPU6050_Config(void);
void MPU6050_2_Config(void);

#endif /* INC_ACCEL_CONFIG_H_ */
