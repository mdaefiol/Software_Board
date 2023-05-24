/*
 * sensor_init.h
 *
 *  Created on: 19 de abr de 2023
 *      Author: mariana.daefiol
 */

#ifndef INC_SENSOR_INIT_H_
#define INC_SENSOR_INIT_H_

typedef struct{
	float EMA_x, EMA_y,  EMA_z, EMA_press;
} Data_EMA;

typedef struct{
	float acc_x1, acc_y1, acc_z1, acc_x2, acc_y2, acc_z2;
	float gy_x, gy_y, gy_z, gy_x2, gy_y2, gy_z2;
	float altitude, pressure, temperature;
} Data_transf;

void calculate_EMA(void);

/* TO DEBUG
void data_att(void);
void transf_BMP280(void);
void transf_MPU6050_1(void);
void transf_MPU6050_2(void);
*/

#endif /* INC_SENSOR_INIT_H_ */
