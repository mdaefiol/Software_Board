/*
 * fram.c
 *
 *  Created on: Apr 6, 2023
 *      Author: mariana.daefiol
 */

#include "global_include.h"

extern SPI_HandleTypeDef hspi1;
FRAM_STATES FRAM_state;

// Command
uint8_t WRSR  = 0x01;
uint8_t WRITE = 0x02;
uint8_t READ  = 0x03;
uint8_t WRDI  = 0x04;
uint8_t RDSR  = 0x05;
uint8_t WREN  = 0x06;
uint8_t RDIDI  = 0x9F;
uint8_t SLEEP = 0xB9;

uint8_t bytes_ID[4];
uint8_t msg[128];
int contador_FRAM = 0;

// READ DESABILITADO PARA VÔO
uint32_t receiv_fromFRAM[4];
uint8_t data_receive[16];
float transf_float[4];


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
        if (hspi->Instance->CR1 & SPI_CR1_SPE) {
 	  	  switch(FRAM_state)
 	  	{
 	  		case WAIT_ENABLE_WRITE:
 	  			FRAM_state = FRAM_WRITE;
 	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
 	  		break;
 	  		case WAIT_FRAM_WRITE:
 	  			FRAM_state = FRAM_IDLE;
 	  			contador_FRAM += 1;
 	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
 	  		break;
 	  		case WAIT_FRAM_READ_COMMAND:
 	  			FRAM_state = FRAM_READ;
 	  		break;
 	  		default:
 	  			break;
 	  	}
      }
   }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi1) {
        if (hspi->Instance->CR1 & SPI_CR1_SPE) {
        	switch(FRAM_state)
 	  	{
 	  		case WAIT_FRAM_READ_ID:
 	  			FRAM_state = FRAM_IDLE;
 	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
 	  		break;
 	  		case WAIT_FRAM_READ:
	  		    receiv_fromFRAM[0] = (data_receive[3] << 24) | (data_receive[2] << 16) | (data_receive[1] << 8) | data_receive[0];
	  		    transf_float[0] = *(float*)&receiv_fromFRAM[0];

	  		    receiv_fromFRAM[1] = (data_receive[7] << 24) | (data_receive[6] << 16) | (data_receive[5] << 8) | data_receive[4];
	  		    transf_float[1] = *(float*)&receiv_fromFRAM[1];

	  		    receiv_fromFRAM[2] = (data_receive[11] << 24) | (data_receive[10] << 16) | (data_receive[9] << 8) | data_receive[8];
	  		    transf_float[2] = *(float*)&receiv_fromFRAM[2];

	  		    receiv_fromFRAM[3] = (data_receive[15] << 24) | (data_receive[14] << 16) | (data_receive[13] << 8) | data_receive[12];
	  		    transf_float[3] = *(float*)&receiv_fromFRAM[3];
 	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
 	  			FRAM_state = FRAM_IDLE;
 	  		break;
 	  		default:
 	  			break;
      }
   }
}
}

void FRAMset_config(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	uint8_t cmd[2] = {0x06, 0x00};
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 1 ,1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	cmd[0] = 0x01;
	cmd[1] = 0x80;
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {

}

void FRAM_ID(void) {

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&RDIDI, 1);
    HAL_SPI_Receive_IT(&hspi1, bytes_ID, 4);
}


void FRAM_enablewrite(void){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&WREN, 1);
}

void FRAM_StatusRegister(void){

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&RDSR, 1);
}

void FRAM_resetWrite(void){

	uint8_t addr_high = 0x00;
    uint8_t addr_low = 0x00;

    msg[0] = WRDI;
    msg[1] = addr_high;
    msg[2] = addr_low;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, msg, 3);
}

void FRAM_Write(uint16_t address, uint8_t *data, uint16_t size){

	uint8_t addr_high = (address >> 8 ) & 0xFF;
    uint8_t addr_low = address & 0xFF;

    msg[0] = WRITE;
    msg[1] = addr_high;
    msg[2] = addr_low;

    for (int i = 0; i < size ; i++){
    	msg[i+3] = data[i];
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, msg, 3+size);
}

void FRAM_Read_Command(uint16_t address){

	uint8_t addr_high = (address >> 8 ) & 0xFF;
    uint8_t addr_low = address & 0xFF;

    msg[0] = READ;
    msg[1] = addr_high;
    msg[2] = addr_low;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&hspi1, msg, 3);
}

void FRAM_Read(uint8_t *data_receive, uint16_t size){

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Receive_IT(&hspi1, data_receive, size);
}
