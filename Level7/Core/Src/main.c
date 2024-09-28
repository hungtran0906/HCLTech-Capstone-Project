/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef  p1header;
CAN_RxHeaderTypeDef  p2header;
uint32_t pTxMailbox = CAN_TX_MAILBOX0, RxFifo = CAN_RX_FIFO0;
CAN_FilterTypeDef  sFilterConfig;
uint8_t aData[8], received_data[8];
uint8_t speed = 0, count = 0, obstacle_status = 0, engine_status = 0, SW1;
uint8_t can_data, Can_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void lcd_init();
void cmd(unsigned char C);
void data(unsigned char D);
void display(unsigned char *ptr);
void numeric(unsigned char Value);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  	lcd_init();
	HAL_Delay(500);

	// Set Output pin STB1, STB2 as LOW
	HAL_GPIO_WritePin(GPIOC, STB1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, STB2_Pin, GPIO_PIN_RESET);

	// CAN Tx message header configuration
	p1header.DLC = 8;
	p1header.IDE = CAN_ID_STD;
	p1header.StdId = 0x001;
	p1header.RTR = CAN_RTR_DATA;

	// CAN Filter configuration
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = 0x00C;
	sFilterConfig.FilterIdLow = 0x00A;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);

	// Start CAN transmission
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	SW1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
	obstacle_status = HAL_GPIO_ReadPin(IR_SEN_GPIO_Port, IR_SEN_Pin);


	if ((!SW1) && (engine_status == 0))
	{
		engine_status = 1;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	}
	else if ((!SW1) && (engine_status == 1))
	{
		engine_status = 0;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin ,1);
	}

	// ADC start and speed calculation
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	count = HAL_ADC_GetValue(&hadc1);
	speed = ((100*count)/4095);
	HAL_ADC_Stop(&hadc1);

	// Data configuration base on engine status
	if (engine_status == 1)
	{
		aData[0] = engine_status;
		aData[1] = speed;
		aData[2] = obtacle_status;
		aData[3] = 0;
		aData[4] = 0;
		aData[5] = 0;
		aData[6] = 0;
		aData[7] = 0;
	}
	else
	{
		aData[0] = engine_status;
		aData[1] = 00;
		aData[2] = 00;
		aData[3] = 0;
		aData[4] = 0;
		aData[5] = 0;
		aData[6] = 0;
		aData[7] = 0;
	}

	// CAN1 send message CAN2 receive
	HAL_CAN_AddTxMessage(&hcan1, &p1header, aData, &pTxMailbox);
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &p2header, received_data);
	Can_id = p1header.StdId;

	// LCD display base on received data
	if (received_data[0] == 1)
	{
		cmd(0x80);
		display("   ENGINE: ON   ");
		cmd(0xC0);
		display("SPEED");
		cmd(0xC6);
		numeric(received_data[1]);
		cmd(0xC9);
		display("OBC");
		cmd(0xCE);
		if (received_data[2] == 1)
		{
			display("YES");
		}
		else
		{
			display("NO");
		}

	}
	else if (received_data[0] == 0)
	{
		cmd(0x80);
		display("   ENGINE: OFF   ");
		cmd(0xC0);
		display("SPEED");
		cmd(0xC6);
		numeric(received_data[1]);
		cmd(0xC9);
		display("OBC");
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 8;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STB1_Pin|STB2_Pin|D0_Pin|D1_Pin
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D2_Pin|D3_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STB1_Pin STB2_Pin D0_Pin D1_Pin
                           LED1_Pin */
  GPIO_InitStruct.Pin = STB1_Pin|STB2_Pin|D0_Pin|D1_Pin
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin EN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_SEN_Pin */
  GPIO_InitStruct.Pin = IR_SEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_SEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void lcd_init()
{
	HAL_Delay(10);
	cmd(0x01);
	cmd(0x38);
	cmd(0x06);
	cmd(0x0C);
	HAL_Delay(10);
}

void cmd(unsigned char C)
{
	HAL_GPIO_WritePin(GPIOA, RS_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, D0_Pin,(C&0X01));
	HAL_GPIO_WritePin(GPIOC, D1_Pin,(C&0X02));
	HAL_GPIO_WritePin(GPIOB, D2_Pin,(C&0X04));
	HAL_GPIO_WritePin(GPIOB, D3_Pin,(C&0X08));
	HAL_GPIO_WritePin(GPIOB, D4_Pin,(C&0X10));
	HAL_GPIO_WritePin(GPIOB, D5_Pin,(C&0X20));
	HAL_GPIO_WritePin(GPIOB, D6_Pin,(C&0X40));
	HAL_GPIO_WritePin(GPIOB, D7_Pin,(C&0X80));
	HAL_GPIO_WritePin(GPIOA, EN_Pin, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA, EN_Pin, 0);
	HAL_Delay(5);
}

void data (unsigned char D)
{
	HAL_GPIO_WritePin(GPIOA, RS_Pin, 1);
	HAL_GPIO_WritePin(GPIOC, D0_Pin,(D&0X01));
	HAL_GPIO_WritePin(GPIOC, D1_Pin,(D&0X02));
	HAL_GPIO_WritePin(GPIOB, D2_Pin,(D&0X04));
	HAL_GPIO_WritePin(GPIOB, D3_Pin,(D&0X08));
	HAL_GPIO_WritePin(GPIOB, D4_Pin,(D&0X10));
	HAL_GPIO_WritePin(GPIOB, D5_Pin,(D&0X20));
	HAL_GPIO_WritePin(GPIOB, D6_Pin,(D&0X40));
	HAL_GPIO_WritePin(GPIOB, D7_Pin,(D&0X80));
	HAL_GPIO_WritePin(GPIOA, EN_Pin, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA, EN_Pin, 0);
	HAL_Delay(5);
}

void display(unsigned char *ptr)
{
	while(*ptr!='\0')
	{
	data(*ptr++);
	HAL_Delay(2);
	}
}

void numeric(unsigned char Value)
{
	data(((Value%1000)/100)+0x30);
	data(((Value%100)/10)+0X30);
	data((Value%10)+0X30);
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
