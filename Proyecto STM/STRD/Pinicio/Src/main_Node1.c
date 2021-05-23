/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "dwt_stm32_delay.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan1;
SPI_HandleTypeDef hspi1;

osThreadId TareaVelocidadHandle;
osThreadId TareaDistanciaHandle;
osThreadId TareaFrenadoHandle;
osThreadId TareaLucesHandle;

SemaphoreHandle_t sem_freno;
int sem_freno_count = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);

/* Tareas perodicas */
void StartTarea_Velocidad(void const * argument);
void StartTarea_Distancia(void const * argument);
void StartTarea_Frenado(void const * argument);
void StartTarea_Luces(void const * argument);

/* Variables para depuracion */
int ContTarea_Velocidad = 0;
int ContTarea_Distancia = 0;
int ContTarea_Frenado = 0;
int Luz = 0;

/* Variable global para potenciometro */
int vActual = 0;
int intensidad_frenado = 0;
int distancia_seguridad = 0;
float currentDistance;

/*Prioridades de las Tareas Periodicas*/
#define PR_TAREA_VELOCIDAD 2
#define PR_TAREA_DISTANCIA 1
#define PR_TAREA_FRENADO 3
#define PR_TAREA_LUCES 1

/*Periodos de las tareas*/
#define T_TAREA_VELOCIDAD 250
#define T_TAREA_DISTANCIA 600
#define T_TAREA_FRENADO 100
#define T_TAREA_LUCES 600

#define TRUE 1
#define FALSE 0

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
	
	/* Initialize ultrasonidos */
	DWT_Delay_Init();
	
  /* Create the mutex(es) */
  /* definition and creation of mutex1 */
	sem_freno = xSemaphoreCreateMutex();

  /* Create the thread(s) */
  /* definition and creation of Tarea1 */
  osThreadDef(Tarea_Velocidad, StartTarea_Velocidad, PR_TAREA_VELOCIDAD, 0, 128);
  TareaVelocidadHandle = osThreadCreate(osThread(Tarea_Velocidad), NULL);
	
	/* Create the thread(s) */
  /* definition and creation of Tarea2 */
  osThreadDef(Tarea_Distancia, StartTarea_Distancia, PR_TAREA_DISTANCIA, 0, 128);
  TareaDistanciaHandle = osThreadCreate(osThread(Tarea_Distancia), NULL);
	
	osThreadDef(Tarea_Frenado, StartTarea_Frenado, PR_TAREA_FRENADO, 0, 128);
  TareaFrenadoHandle = osThreadCreate(osThread(Tarea_Frenado), NULL);
	
	osThreadDef(Tarea_Luces, StartTarea_Luces, PR_TAREA_LUCES, 0, 128);
  TareaLucesHandle = osThreadCreate(osThread(Tarea_Luces), NULL);

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


uint32_t readDistance(void)
{
	__IO uint8_t flag=0;
	__IO uint32_t disTime=0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
	while(flag == 0){
		while(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11) == GPIO_PIN_SET){
			disTime++;
			flag = 1;
		}
	}
	return disTime;
}


/**
  * @brief Ultrasonidos config
  * @retval None
  */
uint32_t DWT_Delay_Init(void) {
	
	/* Disable TRC */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	/* Enable TRC */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
	/* Disable clock cycle counter */
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	/* Enable clock cycle counter */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
	/* Reset the clock cycle counter value */
	DWT->CYCCNT = 0;
	/* 3 NO OPERATION instructions */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");
	/* Check if clock cycle counter has started */
	if(DWT->CYCCNT)
	{
		return 0; /*clock cycle counter started*/
	}
	else
	{
		return 1; /*clock cycle counter not started*/
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/*Configure GPIO pins : PB0 PB3: Interrupcion botones externos */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|
	                      GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}



/* USER CODE BEGIN Header_StartTarea1 */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTarea1 */
void StartTarea_Velocidad(void const * argument)
{	
	
	for(;;){
		
		sem_freno_count = (int)uxSemaphoreGetCount(sem_freno);
		if ( xSemaphoreTake(sem_freno, portMAX_DELAY) == pdTRUE){
			
			int actual = 0; //Valor actual
	
			ContTarea_Velocidad ++;
			
			/* Lectura del canal ADC0 */
			ADC_ChannelConfTypeDef sConfig = {1};
			sConfig.Channel = ADC_CHANNEL_1; // seleccionamos el canal 1
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1); // comenzamos la conversón AD
			
			if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
				actual = HAL_ADC_GetValue(&hadc1); // leemos el valor
				vActual = actual; // actualizamos una variable global
			}
			
			xSemaphoreGive(sem_freno);
			
		}
		
		osDelay(T_TAREA_VELOCIDAD);
	}
}

/* USER CODE BEGIN Header_StartTarea2 */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTarea2 */
void StartTarea_Distancia(void const * argument)
{
	
	TickType_t xLastWakeTime;
	for(;;)
	{
		if (xSemaphoreTake(sem_freno, portMAX_DELAY) == pdTRUE){
			
			xLastWakeTime = xTaskGetTickCount();
			currentDistance = (float)readDistance() * 0.00171821;
			if (currentDistance == 500000) currentDistance = 1;
			//vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 400 )); //Periodo de 400mS
			
			distancia_seguridad = (vActual/10)*(vActual/10);
			
			xSemaphoreGive(sem_freno);
			
		}
		
		osDelay(T_TAREA_DISTANCIA);
		
	}

}

/* USER CODE BEGIN Header_StartTarea3 */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTarea3 */
void StartTarea_Frenado(void const * argument)
{
	
  /* Infinite loop */
  for(;;)
  {
		
		enum estFrenado{ 
			luz0 = 0x00,
			luz1 = 0x01,
			luz2 = 0x02,
			luz3 = 0x03,
			luz4 = 0x04,
		} estFrenado;
		
		if (xSemaphoreTake(sem_freno, portMAX_DELAY) == pdTRUE){
			
			enum estFrenado estado;
			
			estado = luz0;
			
			float resta = currentDistance-(float)(distancia_seguridad);
			
			switch(estado){
				case luz0:
					if(resta <  -0.2*(float)(distancia_seguridad)){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz1;
					}
					break;
					
				case luz1:
					if(resta <  -0.4*(float)(distancia_seguridad)){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz2;
					}
					else if(resta>0){
						
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz0;
					}
						
					break;
					
				case luz2:
					if(resta <  -0.8*(float)(distancia_seguridad)){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz3;
					}
					else if(resta >  -0.2*(float)(distancia_seguridad)){
						
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz1;
					}
						
					break;
					
				case luz3:
					if(resta <  -0.8*(float)(distancia_seguridad)){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
						
						estado = luz4;
					}
					else if(resta >  -0.4*(float)(distancia_seguridad)){
						
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz2;
					}
						
					break;
					
				case luz4:
					if(resta >  -0.8*(float)(distancia_seguridad)){
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
						
						estado = luz3;
					}
						
					break;
					
				}
			
			xSemaphoreGive(sem_freno);
		
		}
		
    osDelay(T_TAREA_FRENADO);
  }
}

void StartTarea_Luces(void const * argument)
{	
	
	for(;;){
		
		int actual_luz = 0; //Valor actual
		
		/* Lectura del canal ADC0 */
		ADC_ChannelConfTypeDef sConfig = {0};
		
		sConfig.Channel = ADC_CHANNEL_0; // seleccionamos el canal 0
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
		
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_Start(&hadc1); // comenzamos la conversón AD
		
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
			actual_luz = HAL_ADC_GetValue(&hadc1); // leemos el valor
			Luz = actual_luz;
		}
		
		if(actual_luz>100){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); 
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); 
		}
		
		osDelay(T_TAREA_LUCES);	
	}
}

/* Funcion para el tratamiento de interrupciones */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  long yield = pdFALSE;
//  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  portYIELD_FROM_ISR(yield);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
