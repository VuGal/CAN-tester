/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

// Engine Control Unit structures
CAN_TxHeaderTypeDef TxHeaderEngineCU = {
	.StdId = 0x1,
	.ExtId = 0x1,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef EngineCUFilterConfig = {
	.FilterIdHigh = 0x1 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 0,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t EngineCUTxData[8] = {0x00, 0x00, 0x01, 0x15, 0x01, 0x68, 0x05, 0x24};

// ----------------------------------------------

// Transmission Control Unit structures
CAN_TxHeaderTypeDef TxHeaderTransmissionCU = {
	.StdId = 0x25,
	.ExtId = 0x25,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef TransmissionCUFilterConfig = {
	.FilterIdHigh = 0x25 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 1,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t TransmissionCUTxData[14] = {0x00, 0x00, 0x07, 0x31, 0x07, 0x32, 0x07, 0x33, 0x07, 0x34, 0x07, 0x35, 0x07, 0x36};

// ----------------------------------------------

// Speed Control Unit structures
CAN_TxHeaderTypeDef TxHeaderSpeedCU = {
	.StdId = 0x10,
	.ExtId = 0x10,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef SpeedCUFilterConfig = {
	.FilterIdHigh = 0x10 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 2,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t SpeedCUTxData[8] = {0x00, 0x00, 0x05, 0x65, 0x05, 0x66, 0x05, 0x74};

// ----------------------------------------------

// Brake Control Unit structures
CAN_TxHeaderTypeDef TxHeaderBrakeCU = {
	.StdId = 0x5,
	.ExtId = 0x5,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef BrakeCUFilterConfig = {
	.FilterIdHigh = 0x5 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 3,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t BrakeCUTxData[6] = {0x00, 0x00, 0x05, 0x57, 0x05, 0x58};

// ----------------------------------------------

// Door Control Unit structures
CAN_TxHeaderTypeDef TxHeaderDoorCU = {
	.StdId = 0x134,
	.ExtId = 0x134,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef DoorCUFilterConfig = {
	.FilterIdHigh = 0x134 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 4,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t DoorCUTxData[14] = {0x00, 0x00, 0x02, 0x06, 0x02, 0x07, 0x02, 0x15, 0x02, 0x16, 0x02, 0x22, 0x02, 0x23};

// ----------------------------------------------

// Seat Control Unit structures
CAN_TxHeaderTypeDef TxHeaderSeatCU = {
	.StdId = 0x176,
	.ExtId = 0x176,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef SeatCUFilterConfig = {
	.FilterIdHigh = 0x176 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 5,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t SeatCUTxData[6] = {0x00, 0x00, 0x02, 0x08, 0x02, 0x09};


// ----------------------------------------------

// Human-Machine Interface structures
CAN_TxHeaderTypeDef TxHeaderHMI = {
	.StdId = 0x200,
	.ExtId = 0x200,
	.RTR = CAN_RTR_DATA,
	.IDE = CAN_ID_STD,
	.DLC = 2,
	.TransmitGlobalTime = DISABLE
};
CAN_FilterTypeDef HMIFilterConfig = {
	.FilterIdHigh = 0x200 << 5,
	.FilterIdLow = 0x0000,
	.FilterMaskIdHigh = 0xFFE0,
	.FilterMaskIdLow = 0x0000,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 6,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 0
};

uint8_t HMICUTxData[8] = {0x00, 0x00, 0x01, 0x96, 0x01, 0x58, 0x01, 0x62};

// ----------------------------------------------

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[4] = {0, 0, 0, 0};

uint32_t TxMailbox;

uint8_t CurrentEngineCUPacket = 0;
uint8_t CurrentTransmissionCUPacket = 0;
uint8_t CurrentSpeedCUPacket = 0;
uint8_t CurrentBrakeCUPacket = 0;
uint8_t CurrentDoorCUPacket = 0;
uint8_t CurrentSeatCUPacket = 0;
uint8_t CurrentHMIPacket = 0;

uint8_t DoorCURequestFlag = 0;
uint8_t SeatCURequestFlag = 0;
uint8_t HMIRequestFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

	// Timer start
	HAL_TIM_Base_Start(&htim1);

	// "SNIFFER" MODE TEST

	// apply CAN filter configuration
	if ((HAL_CAN_ConfigFilter(&hcan, &EngineCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &TransmissionCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &SpeedCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &BrakeCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &DoorCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &SeatCUFilterConfig) != HAL_OK) ||
		(HAL_CAN_ConfigFilter(&hcan, &HMIFilterConfig) != HAL_OK))
	{
	  /* Filter configuration Error */
	  Error_Handler();
	}

	// start CAN bus operation
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
	  /* Start Error */
	  Error_Handler();
	}

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	++CurrentEngineCUPacket;
	++CurrentTransmissionCUPacket;
	++CurrentSpeedCUPacket;
	++CurrentBrakeCUPacket;
	++CurrentDoorCUPacket;
	++CurrentSeatCUPacket;
	++CurrentHMIPacket;

	if ( CurrentEngineCUPacket % ( sizeof(EngineCUTxData)/(2*sizeof(EngineCUTxData[0])) ) == 0 ) {
		CurrentEngineCUPacket = 0;
	}

	if ( CurrentTransmissionCUPacket % ( sizeof(TransmissionCUTxData)/(2*sizeof(TransmissionCUTxData[0])) ) == 0 ) {
		CurrentTransmissionCUPacket = 0;
	}

	if ( CurrentSpeedCUPacket % ( sizeof(SpeedCUTxData)/(2*sizeof(SpeedCUTxData[0])) ) == 0 ) {
		CurrentSpeedCUPacket = 0;
	}

	if ( CurrentBrakeCUPacket % ( sizeof(BrakeCUTxData)/(2*sizeof(BrakeCUTxData[0])) ) == 0 ) {
		CurrentBrakeCUPacket = 0;
	}

	if ( CurrentDoorCUPacket % ( sizeof(DoorCUTxData)/(2*sizeof(DoorCUTxData[0])) ) == 0 ) {
		CurrentDoorCUPacket = 0;
	}

	if ( CurrentSeatCUPacket % ( sizeof(SeatCUTxData)/(2*sizeof(SeatCUTxData[0])) ) == 0 ) {
		CurrentSeatCUPacket = 0;
	}

	if ( CurrentHMIPacket % ( sizeof(HMICUTxData)/(2*sizeof(HMICUTxData[0])) ) == 0 ) {
		CurrentHMIPacket = 0;
	}

	// "SNIFFER" MODE TEST
    //
    // These ECUs broadcast their status regularly. Raspberry Pi will sniff these frames from the bus.

	HAL_CAN_AddTxMessage(&hcan, &TxHeaderEngineCU, EngineCUTxData + (CurrentEngineCUPacket * 2), &TxMailbox);
	HAL_Delay(100);
	HAL_CAN_AddTxMessage(&hcan, &TxHeaderTransmissionCU, TransmissionCUTxData + (CurrentTransmissionCUPacket * 2), &TxMailbox);
	HAL_Delay(100);
	HAL_CAN_AddTxMessage(&hcan, &TxHeaderSpeedCU, SpeedCUTxData + (CurrentSpeedCUPacket * 2), &TxMailbox);
	HAL_Delay(100);
	HAL_CAN_AddTxMessage(&hcan, &TxHeaderBrakeCU, BrakeCUTxData + (CurrentBrakeCUPacket * 2), &TxMailbox);
	HAL_Delay(100);


	// "NODE" MODE TEST
    //
	// These ECUs require a request from the other node to transmit their status.
	// It's achieved by sending a RTR frame with a proper ID from Raspberry Pi.

	if (DoorCURequestFlag == 1) {
		HAL_CAN_AddTxMessage(&hcan, &TxHeaderDoorCU, DoorCUTxData + (CurrentDoorCUPacket * 2), &TxMailbox);
		DoorCURequestFlag = 0;
		HAL_Delay(100);
	}
	if (SeatCURequestFlag == 1) {
		HAL_CAN_AddTxMessage(&hcan, &TxHeaderSeatCU, SeatCUTxData + (CurrentSeatCUPacket * 2), &TxMailbox);
		SeatCURequestFlag = 0;
		HAL_Delay(100);
	}
	if (HMIRequestFlag == 1) {
		HAL_CAN_AddTxMessage(&hcan, &TxHeaderHMI, HMICUTxData + (CurrentHMIPacket * 2), &TxMailbox);
		HMIRequestFlag = 0;
		HAL_Delay(100);
	}

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = ENABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_LED_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if (RxHeader.StdId == 0x134 && RxHeader.RTR == CAN_RTR_REMOTE) {
		DoorCURequestFlag = 1;
	}

	if (RxHeader.StdId == 0x176 && RxHeader.RTR == CAN_RTR_REMOTE) {
		SeatCURequestFlag = 1;
	}

	if (RxHeader.StdId == 0x200 && RxHeader.RTR == CAN_RTR_REMOTE) {
		HMIRequestFlag = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	HAL_NVIC_SystemReset();
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

