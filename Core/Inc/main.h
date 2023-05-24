/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "global_include.h"
#include "fram.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


// Definindo os estados da máquina de estados
typedef enum {
  PAUSADO,
  AGUARDANDO_LANCAMENTO,
  LANCADO,
  VOANDO_ACELERADO,
  DETECCAO_APOGEU,
  VOANDO_RETARDADO,
  PARAQUEDAS_ACIONADO
} State_vehicle;

typedef enum {
    ESTADO_INICIAL,
    ESTADO_SUBIDA,
    ESTADO_DESCIDA,
    ESTADO_ESTACIONARIO
} estado_Altitude ;

typedef enum {
    ACCEL_NEAR_ZERO,
    ACCEL_NEAR_G,
    ACCEL_HIGH_POSITIVE,
    ACCEL_LOW_NEGATIVE,
    ACCEL_HIGH_NEGATIVE
} estado_Aceleracao;


// Definindo a estrutura para armazenar os dados do veículo
typedef struct {
  float accel;
  float accel_anterior;
  float accelTemp;
  float pressao;
  float pressao_anterior;
  float pressTemp;
  float pressMIN;
} Data_vehicle;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
