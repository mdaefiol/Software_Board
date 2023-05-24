/*
 * sensor_init.c
 *
 *  Created on: 19 de abr de 2023
 *      Author: mariana.daefiol
 */
#include "global_include.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern AccelData acc_data;
extern GyroData gy_data;
extern BMP280_data bmp_data;

extern FRAM_STATES FRAM_state;
extern SIL_DATA simulated_data;

Data_EMA ema_receive;
Data_transf data_rec;

float g = 9.80665;   // aceleração da gravidade
float alpha = 0.8;	 // suavizaçao de dados

uint8_t send_FRAM[16];
float get_data[4];

static int count = 0;


void calculate_EMA(void) {
	// SOFTWARE IN THE LOOP
    get_data[0] = ((simulated_data.accel_x)/g);
    get_data[1] = ((simulated_data.accel_y)/g);
    get_data[2] = ((simulated_data.accel_z)/g);
    get_data[3] = (simulated_data.pressao);
/*
    get_data[0] = (acc_data.Ax)/16384.0;
    get_data[1] = (acc_data.Ay)/16384.0;
    get_data[2] = (acc_data.Az)/16384.0;
    get_data[3] = (bmp_data.pressure)/256.0;
*/
    ema_receive.EMA_x = alpha * get_data[0] + (1 - alpha) * ema_receive.EMA_x;
    ema_receive.EMA_y = alpha * get_data[1] + (1 - alpha) * ema_receive.EMA_y;
    ema_receive.EMA_z = alpha * get_data[2] + (1 - alpha) * ema_receive.EMA_z;
    ema_receive.EMA_press = alpha * get_data[3] + (1 - alpha) * ema_receive.EMA_press;

    count++;
    if (count == 10) {  	// Verifica se o contador atingiu 10 para gravar EMA na FRAM
    	count = 0;

    	// Atualiza os valores de data_transf com os últimos valores de EMA
    	float *data_transf = malloc(4 * sizeof(float));
    	data_transf[0] =  ema_receive.EMA_x;
    	data_transf[1] =  ema_receive.EMA_y;
    	data_transf[2] =  ema_receive.EMA_z;
    	data_transf[3] =  ema_receive.EMA_press;

        // float to 8 bits -> 4 bytes
    	memcpy(send_FRAM, &data_transf[0], sizeof(data_transf[0]));
    	memcpy(send_FRAM + sizeof(data_transf[0]), &data_transf[1], sizeof(data_transf[1]));
    	memcpy(send_FRAM + 2 * sizeof(data_transf[0]), &data_transf[2], sizeof(data_transf[2]));
    	memcpy(send_FRAM + 3 * sizeof(data_transf[0]), &data_transf[3], sizeof(data_transf[3]));

    	// Libera a memória alocada por send_FRAM
    	free(data_transf);

    	// Envia os dados para gravaçao na FRAM
    	if (FRAM_state == FRAM_IDLE){
    		FRAM_state = ENABLE_WRITE;
    	}
    }
}

/* TO DEBUG

void data_att(void){
	// UTILIZADO PARA VÔO:
	data_vehicle.acc_x = (acc_data.Ax2)/16384.0; 							 		// continua igual
	data_vehicle.acc_y = ((acc_data.Ay2) - ((acc_data.Az)*1/0.7071))/16384.0; 	// dado rotacionado - dado original (cos 45 graus = 1/sqrt(2)) do outro eixo
	data_vehicle.acc_z = ((acc_data.Az2) - ((acc_data.Ay)*1/0.7071))/16384.0; 	// dado rotacionado - dado original (cos 45 graus = 1/sqrt(2)) do outro eixo

	// dado final de voo -> (data_vehicle.acc_z/cos45)
}


void transf_BMP280(void){
	data_rec.temperature = (bmp_data.temperature)/100.0;
	data_rec.pressure =  (bmp_data.pressure)/256.0;
	data_rec.altitude = 44330.0*(1.0 - pow((data_rec.pressure/101325), 0.1903));
}

void transf_MPU6050_1(void){
	data_rec.acc_x1 = (acc_data.Ax)/16384.0;
	data_rec.acc_y1 = (acc_data.Ay)/16384.0;
	data_rec.acc_z1 = acc_data.Az/16384.0;

	data_rec.gy_x = (gy_data.Gx)/131.0;
	data_rec.gy_y = (gy_data.Gy)/131.0;
	data_rec.gy_z = (gy_data.Gz)/131.0;
}

void transf_MPU6050_2(void){
	data_rec.acc_x2 = (acc_data.Ax2)/16384.0;
	data_rec.acc_y2 = (acc_data.Ay2)/16384.0;
	data_rec.acc_z2 = (acc_data.Az2)/16384.0;

	data_rec.gy_x2 = (gy_data.Gx2)/131.0;
	data_rec.gy_y2 = (gy_data.Gy2)/131.0;
	data_rec.gy_z2 = (gy_data.Gz2)/131.0;
}
*/
