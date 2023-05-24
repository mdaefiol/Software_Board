/*
 * accel_config.c
 *
 *  Created on: May 9, 2023
 *      Author: mariana.daefiol
 */
#include "accel_config.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

AccelData acc_data;
GyroData gy_data;

void MPU6050_Config(void){

    uint8_t check;
    uint8_t data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);	// WHO_AM_I ~ 6050

    if (check == 104){
    	// dispositivo está presente
        data = 0; 	 	 // registro de gerenciamento de energia 0x6B devemos escrever todos os 0s para ativar o sensor
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

        data = 0x07;	 // define DATA RATE de 1KHz escrevendo no registrador SMPLRT_DIV
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        data = 0x00;	 // define a configuração do acelerômetro e giroscópio em ACCEL_CONFIG e GYRO_CONFIG
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
    }
}

void MPU6050_2_Config(void){

	uint8_t check2;
	uint8_t data2;

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check2, 1, 1000); 	// WHO_AM_I ~ 6050

	if (check2 == 104){
		// dispositivo está presente
		data2 = 0;		 // registro de gerenciamento de energia 0x6B devemos escrever todos os 0s para ativar o sensor
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data2, 1, 1000);

		data2 = 0x07;	 // define DATA RATE de 1KHz escrevendo no registrador SMPLRT_DIV
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data2, 1, 1000);

		data2 = 0x00;    // define a configuração do acelerômetro e giroscópio em ACCEL_CONFIG e GYRO_CONFIG
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data2, 1, 1000);

		data2 = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data2, 1, 1000);
	}
}
