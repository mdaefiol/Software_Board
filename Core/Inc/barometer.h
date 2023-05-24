/*
 * barometer.h
 *
 *  Created on: Apr 6, 2023
 *      Author: mariana.daefiol
 */

#ifndef INC_BAROMETER_H_
#define INC_BAROMETER_H_

#include "stm32f1xx_hal.h"

#define BMP280_I2C &hi2c2
#define BMP280_ADD 0xEC 	//como o SDIO está no terra, o endereço c/ 7bits é 0x76, mas 0x76<<1= 0xEC

typedef struct
{
	int32_t pressure, temperature, altitude;
} BMP280_data;

// Definições de oversampling
#define OSRS_OFF    	0x00
#define OSRS_1      	0x01
#define OSRS_2      	0x02
#define OSRS_4      	0x03
#define OSRS_8      	0x04
#define OSRS_16     	0x05

// Definições de MODO
#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby
#define T_SB_0p5    	0x00
#define T_SB_62p5   	0x01
#define T_SB_125    	0x02
#define T_SB_250    	0x03
#define T_SB_500    	0x04
#define T_SB_1000   	0x05
#define T_SB_2000     	0x06
#define T_SB_4000     	0x07

// Coeficientes IIR Filter
#define IIR_OFF     	0x00
#define IIR_2       	0x01
#define IIR_4       	0x02
#define IIR_8       	0x03
#define IIR_16      	0x04

// DEFINIÇÕES DE REGISTRADORES
#define ID_REG      	0xD0	//0x58
#define RESET_REG  		0xE0	//0xB6
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7

void dataRead(void);
int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter);
int32_t bmp280_compensate_T_int32(int32_t adc_T);
uint32_t bmp280_compensate_P_int32(int32_t adc_P);

#endif /* INC_BAROMETER_H_ */
