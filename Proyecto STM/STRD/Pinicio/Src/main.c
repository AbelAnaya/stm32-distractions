/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main2.c
  * @brief          : Main program body Node 2
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

osThreadId TareaAcelerometroHandle;
osThreadId TareaVolantazosHandle;
osThreadId TareaModoHandle;
osThreadId TareaRelaxHandle;
osThreadId TareaRiesgosHandle;

/* Semaforos FreeRTOS */
SemaphoreHandle_t xSemaphoreBinary;
SemaphoreHandle_t mutexSintomas1;
SemaphoreHandle_t mutexSintomas2;
SemaphoreHandle_t mutexModoFunc;

/* CAN Structures */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox; 
uint8_t ByteSent = 0; //declare byte to be transmitted //declare a receive byte
uint8_t ByteReceived, ByteReceived1, ByteReceived2 = 0; //declare a receive byte
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void InicializaTransmisionesCAN(void);

// Funciones para lecturas de los registros del acelerometro
uint8_t spiTxBuf[2], spiRxBuf[2];
uint8_t SPI_Read (uint8_t address);

// Variable global para giro de volante
int giro = 0;

// Variable global para velocidad y distancia recibidas.
int velocidad, distancia;

// Variables para calculo de inclinacion de cabeza. Se hacen globales para poder depurar
int Ix, Iy, Iz;
uint8_t Ix1, Ix2;
uint8_t Iy1, Iy2;
uint8_t Iz1, Iz2;
double X, Y, Z;
double rotX, rotY;

/* Estructura global para almacenar informacion de inclinacion y volantazos para gestion de riesgos */
typedef struct{
	double rotX_sintomas;
	double rotY_sintomas;
	int volantazos;
	
} Struct_SINTOMAS_1;

Struct_SINTOMAS_1 SINTOMAS_1;

// Funcion para inicializar el acelerometro
void Inicializa_Acelerometro ();

/* Tareas perodicas */
void StartTarea_Acelerometro(void const * argument);
void StartTarea_Volantazos(void const * argument);
void StartTarea_Modo (void const * argument);
void StartTarea_Relax (void const * argument);
void StartTarea_Riesgos (void const * argument);

/* Variables para depuracion */
int ContTarea_Acelerometro = 0;
int ContTarea_Volantazos = 0;
int ContTarea_Modo = 0;
int ContTarea_Relax = 0;
int ContTarea_Riesgos = 0;

/* Variables globales de recursos compartidos */
int SINTOMAS_2 = 0;
int sintomas_inclinacion, sintomas_volantazos = 0;
int Modo_funcionamiento = 1;

/* Variable global para potenciometro */

/*Prioridades de las Tareas Periodicas*/
#define PR_TAREA_ACELEROMETRO 1
#define PR_TAREA_VOLANTAZOS 2
#define PR_TAREA_MODO 4
#define PR_TAREA_RIESGOS 3
#define PR_TAREA_RELAX 10

/*Periodos de las tareas*/
#define T_TAREA_ACELEROMETRO 600
#define T_TAREA_VOLANTAZOS 400
#define T_TAREA_RELAX 500
#define T_TAREA_RIESGOS 300

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
	
	/* Initialize acelerometro */
	Inicializa_Acelerometro();
	
	/* Initialize CAN Bus*/
	InicializaTransmisionesCAN(); 
	
  /* Create the mutex(es) */
  /* creation and give of mutexSintomas1 */
	mutexSintomas1 = xSemaphoreCreateMutex();
	xSemaphoreGive(mutexSintomas1);
	
	/* Create the mutex(es) */
  /* creation and give mutexSintomas2 */
	mutexSintomas2 = xSemaphoreCreateMutex();
	xSemaphoreGive(mutexSintomas2);
	
	/* Create the mutex(es) */
  /* creation and give of mutexModoFunc */
	mutexModoFunc = xSemaphoreCreateMutex();
	xSemaphoreGive(mutexModoFunc);
	
	/* Create the binaries sempahore(es) */
  /* definition and creation of binarie semaphore1*/
	xSemaphoreBinary = xSemaphoreCreateBinary();

  /* Create the thread(s) */
  /* definition and creation of TareaAcelerometro */
  osThreadDef(Tarea_Acelerometro, StartTarea_Acelerometro, PR_TAREA_ACELEROMETRO, 0, 128);
  TareaAcelerometroHandle = osThreadCreate(osThread(Tarea_Acelerometro), NULL);
	
	/* Create the thread(s) */
  /* definition and creation of TareaVolantazos*/
  osThreadDef(Tarea_Volantazos, StartTarea_Volantazos, PR_TAREA_VOLANTAZOS, 0, 128);
  TareaVolantazosHandle = osThreadCreate(osThread(Tarea_Volantazos), NULL);
	
	/* Create the thread(s) */
  /* definition and creation of TareaVolantazos*/
  osThreadDef(Tarea_Modo, StartTarea_Modo, PR_TAREA_MODO, 0, 128);
  TareaModoHandle = osThreadCreate(osThread(Tarea_Modo), NULL);
	
	/* Create the thread(s) */
  /* definition and creation of TareaRelax*/
  osThreadDef(Tarea_Relax, StartTarea_Relax, PR_TAREA_RELAX, 0, 128);
  TareaRelaxHandle = osThreadCreate(osThread(Tarea_Relax), NULL);
	
	/* Create the thread(s) */
  /* definition and creation of TareaRiesgos*/
  osThreadDef(Tarea_Riesgos, StartTarea_Riesgos, PR_TAREA_RIESGOS, 0, 128);
  TareaRiesgosHandle = osThreadCreate(osThread(Tarea_Riesgos), NULL);

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

/**
  * @brief Init CAN BUS
  * @retval None
  */

void InicializaTransmisionesCAN() {

	pHeader.DLC=4; //give message size of 1 byte
	pHeader.IDE=CAN_ID_STD; //set identifier to standard
	pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
	pHeader.StdId=0x2F4; //define a standard identifier, used for message identification by filters (##switch this for the other microcontroller##)
	
	//filter one (stack light blink)
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x2FF<<5; //the ID that the filter looks for (##switch this for the other microcontroller##)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0x3F0<<5; //Mask: 11 1111 0000
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK; //set identifier mask mode
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&hcan1); //start CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
};

/**
  * @brief READ SPI
  * @retval None
  */

uint8_t SPI_Read (uint8_t address)
{
	
	// 1.Bring slave select low
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	// 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
	spiTxBuf[0] = address|0x80; //Register
	HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
	// 3.Receive data
	HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
	// 4.Bring slave select high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	return spiRxBuf[0];
}

/**
  * @brief Inicializa Acelerometro
  * @retval None
  */
void Inicializa_Acelerometro ()
{
 /*To transmit data in SPI follow the next steps: */
 // 1. Bring slave select to low
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2. Transmit register + data
 spiTxBuf[0] = 0x20; // control Register
 spiTxBuf[1] = 0x17; //Data  Enable X Y Z Rate 3.125 Hz --- Valor original = 0x11
 //								size, timeout
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 2, 50);
 // 3. Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

 /*To receive data in SPI follow the next steps: */
 // 1.Bring slave select low
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
 // 2.Transmit register + 0x80 (To set MSB high) Most Significant Bit(MSB) high = read mode
 spiTxBuf[0] = 0x20|0x80; //Register
 HAL_SPI_Transmit(&hspi1, spiTxBuf, 1, 50);
 // 3.Receive data
 HAL_SPI_Receive(&hspi1, spiRxBuf, 1, 50);
 // 4.Bring slave select high
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
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

/**
  * @brief  Function tratamiento interrupciones
  * @param  argument: Not used 
  * @retval None
  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	long yield = pdFALSE;
	// Prevent unused argument(s) compilation warning
	UNUSED(GPIO_Pin);
	
	// Unlock task
	xSemaphoreGiveFromISR(xSemaphoreBinary, &yield);
	
	// yield
	portYIELD_FROM_ISR(yield);
}

/* USER CODE BEGIN Header_StartTareaAcelerometro */
/**
  * @brief  Function implementing the TareaAcelerometro thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTareaAcelerometro */
void StartTarea_Acelerometro(void const * argument)
{
	int sintomas1_y, n_inclinacion_y, sintomas1_x = 0;
	
	for(;;){
		
		if( xSemaphoreTake(mutexSintomas1, portMAX_DELAY) == pdTRUE){
			
			ContTarea_Acelerometro++;
		
			Ix1 = SPI_Read (0x28);
			Ix2 = SPI_Read (0x29);
			Ix = (Ix2 << 8) + Ix1;
			if (Ix >= 0x8000) Ix = -(65536 - Ix);
			X = Ix/16384.0;
			
			Iy1 = SPI_Read (0x2A);
			Iy2 = SPI_Read (0x2B);
			Iy = (Iy2 << 8) + Iy1;
			if (Iy >= 0x8000) Iy = -(65536 - Iy);
			Y = Iy/16384.0;
			
			Iz1 = SPI_Read (0x2C);
			Iz2 = SPI_Read (0x2D);
			Iz = (Iz2 << 8) + Iz1;
			if (Iz >= 0x8000) Iz = -(65536 - Iz);
			Z = Iz/16384.0;
			
			rotX = atan2(Y, sqrt(X*X+Z*Z)) * 180.0/3.1416;
			rotY = - atan2(X, sqrt(Y*Y+Z*Z)) * 180.0/3.1416;
			
			// Deteccion inclinacion cabeza vertical
			if( rotY>20 || rotY<-20 ){
				
				n_inclinacion_y++;
				
				if(n_inclinacion_y == 2){
					SINTOMAS_1.rotY_sintomas = rotY;
				}
			}
			else SINTOMAS_1.rotY_sintomas = 0; n_inclinacion_y = 0;
			
			// Deteccion inclinacion cabeza lateral
			if( ((rotX<-20) || (rotX>20)) && ( (giro > 512) || (giro < 512)) ){
				SINTOMAS_1.rotX_sintomas = rotX;
			}
			else SINTOMAS_1.rotX_sintomas = 0;
			
			xSemaphoreGive(mutexSintomas1);
			
		}
		
		osDelay(T_TAREA_ACELEROMETRO);
	}
}

/* USER CODE BEGIN Header_StartTareaVolantazos */
/**
  * @brief  Function implementing the TareaVolantazos thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTareaVolantazos */
void StartTarea_Volantazos(void const * argument)
{
	
	int giroAnterior = 0;
	
	for(;;)
	{
		
		if( xSemaphoreTake(mutexSintomas1, portMAX_DELAY) == pdTRUE){
			
			ContTarea_Volantazos++;
			
			/* Lectura del canal ADC0 */
			ADC_ChannelConfTypeDef sConfig = {1};
			sConfig.Channel = ADC_CHANNEL_1; // seleccionamos el canal 1
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1); // comenzamos la conversón AD
				
			if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
				giroAnterior = giro;
				giro = HAL_ADC_GetValue(&hadc1); // leemos el valor
			}
			
			if(((giroAnterior - giro) >= 170) || ((giroAnterior - giro) <= -170)){
				SINTOMAS_1.volantazos = TRUE;
			}
			else SINTOMAS_1.volantazos = FALSE;
			
			xSemaphoreGive(mutexSintomas1);
			
		}
		
		osDelay(T_TAREA_VOLANTAZOS);
		
	}

}

/* USER CODE BEGIN Header_StartTareaModo */
/**
  * @brief  Function implementing the TareaModo thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTareaModo */
void StartTarea_Modo(void const * argument)
{
	
	for(;;)
	{
		
		if( xSemaphoreTake(xSemaphoreBinary, portMAX_DELAY) == pdTRUE){
			
			if(xSemaphoreTake(mutexModoFunc, portMAX_DELAY) == pdTRUE){
				
				ContTarea_Modo++;
			
				// Cambio en el modo de funcionamiento
				switch(Modo_funcionamiento){
					case 1:
						Modo_funcionamiento = 2;
						break;
					case 2:
						Modo_funcionamiento = 3;
						break;
					case 3:
						Modo_funcionamiento = 1;
						break;
				}
				xSemaphoreGive(mutexModoFunc);
			}
		}
	}
}

/* USER CODE BEGIN Header_StartTareaRelax*/
/**
  * @brief  Function implementing the TareaRelax thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTareaRelax */
void StartTarea_Relax (void const * argument)
{
	
	int relaxCounter = 0;
	
	for(;;)
	{
		
		if( xSemaphoreTake(mutexSintomas2, portMAX_DELAY) == pdTRUE){
		 
			ContTarea_Relax++;
			
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_RESET){
				
				if(relaxCounter < 2){
					relaxCounter++;
				}
				else if (relaxCounter == 2){
					relaxCounter++;
					SINTOMAS_2 = TRUE;
				}
				else if(relaxCounter == -1){
					relaxCounter = 3;
				}
			}
			else{
				
				if(SINTOMAS_2 == TRUE){
					
					relaxCounter = -1;
					if(relaxCounter == -1){
						SINTOMAS_2 = FALSE;
					}
				}
				else relaxCounter = 0;
			}
			xSemaphoreGive(mutexSintomas2);
		}
		osDelay(T_TAREA_RELAX);
	}
}

/* USER CODE BEGIN Header_StartTareaRiesgos*/
/**
  * @brief  Function implementing the TareaRiesgos thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTareaRiesgos */
void StartTarea_Riesgos (void const * argument)
{
	int somnolencia, no_atiende, movil = FALSE;
	
	
	for(;;)
	{
		 ContTarea_Riesgos++;
		
		velocidad = ByteReceived1;
		distancia = ByteReceived2;
		
		// MOVIL
		if( ((SINTOMAS_1.rotX_sintomas > 20) || (SINTOMAS_1.rotX_sintomas < -20)) && ((SINTOMAS_1.rotY_sintomas > 20) || (SINTOMAS_1.rotY_sintomas < -20)) && (SINTOMAS_2 == TRUE)){
			
			movil = TRUE;
		
			/* Encender Luz Amarilla y pitido nivel 1 */
		
		} else movil = FALSE; /* Apagar luz amarilla y pitido de nivel 1 */
		
		// NO ATIENDE. PENDIENTE AÑADIR COMPROBACION DE VELOCIDAD > 70 Km/h
		if( (((SINTOMAS_1.rotX_sintomas > 20) || (SINTOMAS_1.rotX_sintomas < -20)) || ((SINTOMAS_1.rotY_sintomas > 20) || (SINTOMAS_1.rotY_sintomas < -20))) && (SINTOMAS_2 == FALSE) && (velocidad >= 70)){
				
			no_atiende = TRUE;
			
			/* Encender luz amarilla */
		
		} else no_atiende = FALSE; /* Apagar luz amarilla */
		
		// SOMNOLENCIA
		if( ((SINTOMAS_1.rotX_sintomas > 30) || (SINTOMAS_1.rotX_sintomas < -30)) && (SINTOMAS_1.volantazos == FALSE) ){
			
			somnolencia = TRUE;
			
			/* Encender luz amarilla y pitido nivel 2 */
			
		} else somnolencia = FALSE; /*Desactivar luz amarilla y pitido de nivel 2 */
		
		// Deteccion riesgos nivel 2
		if( (movil+somnolencia+no_atiende) >= 2){
			
			/* Encender luz roja y pitido de nivel 2 */
			
		} else /* Apagar luz roja y pitido de nivel 2 */
		
		/* Deteccion situacion de emergencia. Necesaria distancia de seguridad correcta y distancia actual */
		
		osDelay(T_TAREA_RIESGOS);
	}
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
