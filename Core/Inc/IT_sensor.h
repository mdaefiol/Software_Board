/*
 * IT_sensor.h
 *
 *  Created on: 7 de mai de 2023
 *      Author: mariana.daefiol
 */

#ifndef INC_IT_SENSOR_H_
#define INC_IT_SENSOR_H_

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z; // aceleraçao subida
	float pressao;
	float altitude;
} SIL_DATA;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_IT_SENSOR_H_ */
