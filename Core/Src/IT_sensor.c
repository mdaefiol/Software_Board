/*
 * IT_sensor.c
 *
 *  Created on: 7 de mai de 2023
 *      Author: mariana.daefiol
 */
#include "global_include.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

extern AccelData acc_data;
extern GyroData gy_data;
extern BMP280_data bmp_data;
extern Data_transf data_rec;

uint8_t Data[6];
uint8_t data_accel = 0x00;
uint8_t data_bar = 0x00;
int32_t tRaw, pRaw;

uint8_t rec_data[14];
uint8_t rec_data2[14];

static int data_received_count = 0;
uint8_t data_UART;


// VARIAVEIS PARA SOFTWARE IN THE LOOP:
uint8_t rx_byte[1] = {0x01};
uint32_t data_hex;
float data_float;
static uint8_t rx_buffer[16];

// SOFTWARE IN THE LOOP
SIL_DATA simulated_data;

uint8_t press_receiv = 0;
int MPU_Count = 0;


// INTERRUPÇAO CONFIGURADA PARA 5 m/s (200Hz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == htim4.Instance){
		/*MPU_Count++;
		if ((MPU_Count%2) == 1){   // 5m/s
			HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data, 14);
		//	HAL_I2C_Mem_Read_IT(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data2, 14);
			data_accel = 0x01;
		}
		if ((MPU_Count%20) == 0){   // 50m/s
    		//HAL_I2C_Mem_Read_IT(&hi2c2, BMP280_ADD, PRESS_MSB_REG, 1, Data, 6);
    		MPU_Count = 0;
    		data_bar = 0x01;
		*/

			// Recebe dados pela SERIAL para SOFTWARE IN THE LOOP
			if (data_received_count == 0) {
				HAL_UART_Transmit_IT(&huart2, rx_byte, 1);
				data_received_count = 1;
			}
			else if (data_received_count != 0){
				HAL_UART_Receive_IT(&huart2, rx_buffer, 16);
				data_UART = 0x01;
				data_received_count++;
    	}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c->Instance == hi2c1.Instance){
    		acc_data.Ax = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    		acc_data.Ay = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    		acc_data.Az = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    		gy_data.Gx = (int16_t)(rec_data[8] << 8  | rec_data[9]);
    		gy_data.Gy = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    		gy_data.Gz = (int16_t)(rec_data[12] << 8 | rec_data[13]);
    		/* To debug
    		transf_MPU6050_1();
    		*/
    }

    if (hi2c->Instance == hi2c2.Instance) {
    	if(data_accel == 0x01){
    		acc_data.Ax2 = (int16_t)(rec_data2[0] << 8 | rec_data2[1]);
    		acc_data.Ay2 = (int16_t)(rec_data2[2] << 8 | rec_data2[3]);
    		acc_data.Az2 = (int16_t)(rec_data2[4] << 8 | rec_data2[5]);
    		gy_data.Gx2 = (int16_t)(rec_data2[8] << 8  | rec_data2[9]);
    		gy_data.Gy2 = (int16_t)(rec_data2[10] << 8 | rec_data2[11]);
    		gy_data.Gz2 = (int16_t)(rec_data2[12] << 8 | rec_data2[13]);
    		/* To debug
    		 transf_MPU6050_2();
    		*/
    	}
    	if(data_bar == 0x01){
    		pRaw = (Data[0]<<12)|(Data[1]<<4)|(Data[2]>>4);
    		tRaw = (Data[3]<<12)|(Data[4]<<4)|(Data[5]>>4);

    		data_bar = 0x00;

    		if (data_bar == 0x00){
    			if (tRaw != 0x800000){
    				bmp_data.temperature = (bmp280_compensate_T_int32(tRaw));  // x/100.0 temp
    			}
    			else{
    				bmp_data.temperature = 0; // value in case temp measurement was disabled
    			}
    			if (pRaw != 0x800000){
    				// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    				// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    				bmp_data.pressure = (bmp280_compensate_P_int32(pRaw));
    			}
    			else{
    				bmp_data.pressure = 0; // value in case temp measurement was disabled
    			}
    		}

    		else{
    			bmp_data.temperature = bmp_data.pressure = 0;
    		}
    	/* To debug
    	transf_BMP280();
    	*/
    }
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){

}

// SOFTWARE IN THE LOOP
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == huart2.Instance){
		if(data_UART == 0x01) {
			data_hex = (rx_buffer[0] << 24) | (rx_buffer[1] << 16) | (rx_buffer[2] << 8) | rx_buffer[3];
			data_float = *(float*)&data_hex;
			simulated_data.accel_x = data_float;

			data_hex = (rx_buffer[4] << 24) | (rx_buffer[5] << 16) | (rx_buffer[6] << 8) | rx_buffer[7];
			data_float = *(float*)&data_hex;
			simulated_data.accel_y = data_float;

			data_hex = (rx_buffer[8] << 24) | (rx_buffer[9] << 16) | (rx_buffer[10] << 8) | rx_buffer[11];
			data_float = *(float*)&data_hex;
			simulated_data.accel_z = data_float;

			data_hex = (rx_buffer[12] << 24) | (rx_buffer[13] << 16) | (rx_buffer[14] << 8) | rx_buffer[15];
			data_float = *(float*)&data_hex;
			simulated_data.pressao = data_float;

			simulated_data.altitude = 44330.0*(1.0 - pow((simulated_data.pressao/101325), 0.1903));
			calculate_EMA();
			press_receiv = 1;
		}
		data_UART = 0x00;
	}
}
