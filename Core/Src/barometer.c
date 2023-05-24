/*
 * barometer.c
 *
 *  Created on: Apr 6, 2023
 *      Author: mariana.daefiol
 */

#include "barometer.h"

extern I2C_HandleTypeDef hi2c2;

BMP280_data bmp_data;

int32_t t_fine;
uint8_t trimdata[24];

uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3,dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

uint8_t chipID;
float pressureSeaLevel = 101325;

// Leitura de corte, dados armazenados na memória do sensor e utilizados para calculos de novos valores.
void dataRead(void)
{
	// Lê NVM de 0x88 a 0xA1
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, 0x88, 1, trimdata, 24, 1000);

	dig_T1 = (trimdata[1] << 8) | trimdata[0];
	dig_T2 = (trimdata[3] << 8) | trimdata[2];
	dig_T3 = (trimdata[5] << 8) | trimdata[4];
	dig_P1 = (trimdata[7] << 8) | trimdata[5];
	dig_P2 = (trimdata[9] << 8) | trimdata[6];
	dig_P3 = (trimdata[11] << 8) | trimdata[10];
	dig_P4 = (trimdata[13] << 8) | trimdata[12];
	dig_P5 = (trimdata[15] << 8) | trimdata[14];
	dig_P6 = (trimdata[17] << 8) | trimdata[16];
	dig_P7 = (trimdata[19] << 8) | trimdata[18];
	dig_P8 = (trimdata[21] << 8) | trimdata[20];
	dig_P9 = (trimdata[23] << 8) | trimdata[22];

}

int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{

	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, ID_REG, 1, &chipID, 1, 1000);

	if (chipID == 0x58) {
		// Lê os parâmetros do Trimming
		dataRead();

		uint8_t datatowrite = 0;
		uint8_t datacheck = 0;

		// Reinicializa o dispositivo
		datatowrite = 0xB6;  // reinicia a sequência
		if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, RESET_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		{
			return -1;
		}

		HAL_Delay (100);

		// grava o tempo de espera e o coeff do filtro IIR em 0xF5
		datatowrite = (t_sb <<5) |(filter << 2);
		if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, CONFIG_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		{
			return -1;
		}
		HAL_Delay (100);
		HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, CONFIG_REG, 1, &datacheck, 1, 1000);
		if (datacheck != datatowrite)
		{
			return -1;
		}

		// grava a sobreamostragem de pressão e temperatura junto com o modo para 0xF4
		datatowrite = (osrs_t <<5) |(osrs_p << 2) | mode;
		if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
		{
			return -1;
		}
		HAL_Delay (100);
		HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADD, CTRL_MEAS_REG, 1, &datacheck, 1, 1000);
		if (datacheck != datatowrite)
		{
			return -1;
		}

		return 0;
	}
}

int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1))) >> 12)*((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = (((int64_t)t_fine) - 128000);
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 +((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1) >>33;

	if (var1 == 0)
			{
				return -1; //evita exceção causada por divisão por zero
			}

	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (uint32_t)p;
}
