/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NUM_AMOSTRAS 15  // Número de amostras para verificar a tendência da altitude

// ESTADO DA FRAM E VEICULO, DADOS DO VEICULO: ACELERAÇÃO E PRESSÃO
extern FRAM_STATES FRAM_state;
Data_vehicle data_vehicle;
State_vehicle current_state;
estado_Altitude EstadoAltitude;
estado_Aceleracao EstadoAceleracao;

// MPU6050
extern AccelData acc_data;
extern GyroData gy_data;

// BMP280
extern BMP280_data bmp_data;

// MÉDIA MÓVEL EXPONENCIAL
extern Data_EMA ema_receive;
extern Data_transf data_rec;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // INICIALIZAÇÀO DOS PINOS DE ACIONAMENTO E MONITORAMENTO DE CARGA
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 			// ACIONA CARGA 1 = 0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 			// ACIONA CARGA 2 = 0
  GPIO_PinState pin_state1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9); 	// Lê o estado do pino GPIO14 para carga 1; RESET = 0V -> HA CARGA
  GPIO_PinState pin_state2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); 	// Lê o estado do pino GPIO15 para carga 2; RESET = 0V -> HA CARGA

  // INICIALIZAÇÃO DE SENSORES
  MPU6050_Config();
  MPU6050_2_Config();
  BMP280_Config(OSRS_2, OSRS_16, MODE_NORMAL, T_SB_0p5, IIR_16);

  int contSubida = 0;
  int contDescida = 0;
  float diffAceleracao;
  data_vehicle.pressMIN = 100000;
  extern uint8_t press_receiv ;

  // INICIALIZÇÃO DO ESTADO E INDICADORES DE PRÉ LANÇAMENTO
  current_state = PAUSADO;
  EstadoAltitude = ESTADO_INICIAL;

  GPIO_PinState pin_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); 		// BUTTON
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 	GPIO_PIN_RESET);  	// LED VERDE -> ESTADO LANÇADO
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 	GPIO_PIN_RESET); 	// LED VERMELHO -> PAUSADO
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	// LED AMARELO -> ESTADO AGUARDANDO LANÇAMENTO
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 	GPIO_PIN_RESET); 	// BUZER

  // INICIALIZAÇÃO DAS INTERRUPÇÕES
  HAL_TIM_Base_Start_IT(&htim4);

  // INICIALIZAÇÃO FRAM
  FRAM_state = FRAM_PAUSED;
  FRAMset_config();
  uint16_t FRAM_address = 0x0000;
  extern uint8_t send_FRAM[16];
  extern uint8_t bytes_ID[4];
  // CÓDIGOS PARA SOFTWARE IN THE LOOP:
  extern SIL_DATA simulated_data;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

 while (1)
 {
	 pin_state1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	 pin_state2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	 pin_button = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

     // Verifica a tendência da aceleraçao
	 // data_vehicle.accel = simulated_data.accel_z;
	 data_vehicle.accel = ema_receive.EMA_z;
	 //data_vehicle.accelTemp = data_vehicle.accel;
	 data_vehicle.accelTemp = ema_receive.EMA_z;
     diffAceleracao = data_vehicle.accel - data_vehicle.accel_anterior;

     if (data_vehicle.accel > -0.1 && data_vehicle.accel < 0.1) {
    	 EstadoAceleracao = ACCEL_NEAR_ZERO;
     } else if ( -1.5 < data_vehicle.accel && data_vehicle.accel < -1) {
    	 EstadoAceleracao = ACCEL_NEAR_G;
     } else if (data_vehicle.accel > 1) {
    	 EstadoAceleracao = ACCEL_HIGH_POSITIVE;
     } else if (diffAceleracao < -0.05) {
    	 EstadoAceleracao = ACCEL_HIGH_NEGATIVE;
     } else {
    	 EstadoAceleracao = ACCEL_LOW_NEGATIVE;
     }


     // Verifica a tendência da altitude
     if (press_receiv == 1){
    	 press_receiv = 0;
    	// data_vehicle.pressao = simulated_data.pressao;
    	 data_vehicle.pressao = ema_receive.EMA_press;
    	 data_vehicle.pressTemp = data_vehicle.pressao;

    	 if ( data_vehicle.pressao < data_vehicle.pressao_anterior){
    		 if (contSubida < 30)
    			 contSubida++;
    		 if (contDescida > 0)
    			 contDescida--;
    	 } else if ( data_vehicle.pressao > data_vehicle.pressao_anterior) { //5%
    		 if (contDescida < 30)
    			 contDescida++;
    		 if (contSubida > 0 )
    			 contSubida--;
    	 } else if (data_vehicle.pressao == data_vehicle.pressao_anterior){
    		 if (contDescida > 0)
    		     contDescida--;
    		 if (contSubida > 0 )
    			 contSubida--;
    	 }
    	 data_vehicle.pressao_anterior = data_vehicle.pressTemp;
     }


     // MAQUINA DE ESTADOS PARA VERIFICAÇAO DA ALTITUDE
     switch (EstadoAltitude) {
         case ESTADO_INICIAL:
             if (contSubida >= NUM_AMOSTRAS) {
            	 EstadoAltitude = ESTADO_SUBIDA;
             } else if (contDescida >= NUM_AMOSTRAS) {
            	 EstadoAltitude = ESTADO_DESCIDA;
             } else if (contSubida == 0 && contDescida == 0) {
            	 EstadoAltitude = ESTADO_ESTACIONARIO;
             }
             break;

         case ESTADO_SUBIDA:
             if (contDescida >= NUM_AMOSTRAS) {
            	 EstadoAltitude = ESTADO_DESCIDA;
             } else if (contSubida == 0 && contDescida == 0) {
            	 EstadoAltitude = ESTADO_ESTACIONARIO;
             }else
             break;

         case ESTADO_DESCIDA:
             if (contSubida >= NUM_AMOSTRAS) {
            	 EstadoAltitude = ESTADO_SUBIDA;
             } else if (contSubida == 0 && contDescida == 0) {
            	 EstadoAltitude = ESTADO_ESTACIONARIO;
             }
             break;

         case ESTADO_ESTACIONARIO:
             if (contSubida >= NUM_AMOSTRAS) {
            	 EstadoAltitude = ESTADO_SUBIDA;
             } else if (contDescida >= 7) {
            	 EstadoAltitude = ESTADO_DESCIDA;
             }
             break;
     }
     data_vehicle.accel_anterior = data_vehicle.accelTemp;


	// MAQUINA DE ESTADOS DO FOGUETE
	switch (current_state){
		case PAUSADO:
			if (pin_state1 == GPIO_PIN_RESET && pin_state2 == GPIO_PIN_RESET && pin_button == GPIO_PIN_SET ){
				current_state = AGUARDANDO_LANCAMENTO;
				FRAM_state = FRAM_IDLE;
				FRAM_ID();
	          															// ***** JA FOI INICIADA A AQUISIÇÃO DE DADOS *****
	        															// CARGAS ESTÃO CONECTADAS, ENTRE EM ESTADO DE AGUARDANDO LANÇAMENTO
			}
			else {
				current_state = PAUSADO;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); 	// LED VERMELHO para indicar PAUSA
			}
		break;
		case AGUARDANDO_LANCAMENTO:
			if (EstadoAceleracao == ACCEL_HIGH_POSITIVE){
				current_state = LANCADO;
			}
			else if (EstadoAceleracao == ACCEL_NEAR_ZERO && EstadoAltitude == ESTADO_ESTACIONARIO){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 	// DESLIGA LED VERMELHO
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 	// BUZZER INDICA AGUARDANDO LANÇAMENTO
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); 	// LED AMARELO INDICAÇÃO AGUARDANDO LANÇAMENTO
				current_state = AGUARDANDO_LANCAMENTO;
			}
		break;
		case LANCADO:
			if (EstadoAltitude == ESTADO_SUBIDA && EstadoAceleracao == ACCEL_HIGH_POSITIVE){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// DESLIGA BUZZER
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	// DESLIGA LED AMARELO
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); 		// LIGA LED VERDE DE LANÇADO
	          	current_state = LANCADO;
			}
			else if(EstadoAltitude == ESTADO_SUBIDA && EstadoAceleracao == ACCEL_LOW_NEGATIVE){
				current_state = DETECCAO_APOGEU ;
			}

			break;
	     case DETECCAO_APOGEU:
	          if (EstadoAceleracao == ACCEL_NEAR_G && EstadoAltitude == ESTADO_DESCIDA ){
	        	  if(data_vehicle.pressMIN > data_vehicle.pressao)
	        		  data_vehicle.pressMIN =  data_vehicle.pressao ;
	        	  current_state = DETECCAO_APOGEU;
	          }
	          else if ((EstadoAceleracao == ACCEL_LOW_NEGATIVE || EstadoAceleracao == ACCEL_HIGH_NEGATIVE) && EstadoAltitude ==  ESTADO_DESCIDA){
	        	  current_state = PARAQUEDAS_ACIONADO;
	        	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // ACIONA CARGA 1 = 1;
	        	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // ACIONA CARGA 2 = 1; paraquedas on
	        	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); // DESLIGA O LED VERDE
	          }
	         break;
	     case PARAQUEDAS_ACIONADO:

	    	 break;
	  	default:
	  		 break;
	  }


	// MAQUINA DE ESTADOS DA FRAM
	switch(FRAM_state){
		case FRAM_PAUSED:

		break;
	  	case FRAM_READ_ID:
	  		FRAM_state = WAIT_FRAM_READ_ID;
	  		FRAM_ID();
	  		break;
		case ENABLE_WRITE:
			FRAM_state = WAIT_ENABLE_WRITE;
			FRAM_enablewrite();
	  		break;
		case FRAM_WRITE:
			FRAM_state = WAIT_FRAM_WRITE;
			FRAM_Write(FRAM_address, send_FRAM, 16);
			FRAM_address += 0x10;					// Incrementa a cada 16 bytes
	  		break;
		case FRAM_IDLE:

			break;
/*
		case FRAM_READ:
			FRAM_state = WAIT_FRAM_READ;
			FRAM_Read(data_receive, 16);
	  		break;

		case FRAM_IDLE:
			FRAM_state = ENABLE_WRITE;

		break;
*/
		default:
			break;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB13 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
