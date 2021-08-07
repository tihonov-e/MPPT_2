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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MASTER_REQ_READ    0x12
#define MASTER_REQ_WRITE   0x34
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;

COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp5;
COMP_HandleTypeDef hcomp6;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DAC_HandleTypeDef hdac3;
DAC_HandleTypeDef hdac4;

I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_test_communication_message**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO uint16_t hTxNumData = 0;
__IO uint16_t hRxNumData = 0;
uint8_t bTransferRequest = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_ADC5_Init(void);
static void MX_COMP3_Init(void);
static void MX_COMP5_Init(void);
static void MX_COMP6_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_DAC3_Init(void);
static void MX_DAC4_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */

static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2,
		uint16_t BufferLength);
static void Flush_Buffer(uint8_t *pBuffer, uint16_t BufferLength);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;

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
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_ADC4_Init();
	MX_ADC5_Init();
	MX_COMP3_Init();
	MX_COMP5_Init();
	MX_COMP6_Init();
	MX_DAC1_Init();
	MX_DAC2_Init();
	MX_DAC3_Init();
	MX_DAC4_Init();
	MX_I2C4_Init();
	MX_USART1_UART_Init();
	MX_USB_PCD_Init();

	/* USER CODE BEGIN 2 */

	/*Turn on LED_Status*/
	HAL_GPIO_WritePin  ( GPIOB, LED_STATUS_Pin,  GPIO_PIN_SET  );


	/* Wait for User push-button press before starting the Communication */
	while (HAL_GPIO_ReadPin(GPIOB, BTN_USER_Pin) == GPIO_PIN_RESET) {
	}

	/* Wait for User push-button release before starting the Communication */
	while (HAL_GPIO_ReadPin(GPIOB, BTN_USER_Pin) == GPIO_PIN_SET) {
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		hTxNumData = TXBUFFERSIZE;
		hRxNumData = RXBUFFERSIZE;

		/* Update bTransferRequest to send buffer write request for Slave */
		bTransferRequest = MASTER_REQ_WRITE;

		/*##-2- Master sends write request for slave #############################*/
		do {
			if (HAL_I2C_Master_Transmit_IT(&hi2c4, (uint16_t) I2C_ADDRESS,
					(uint8_t*) &bTransferRequest, 1) != HAL_OK) {
				/* Error_Handler() function is called when error occurs. */
				Error_Handler();
			}

			/*  Before starting a new communication transfer, you need to check the current
			 state of the peripheral; if it’s busy you need to wait for the end of current
			 transfer before starting a new one.
			 For simplicity reasons, this example is just waiting till the end of the
			 transfer, but application may perform other tasks while transfer operation
			 is ongoing. */
			while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {}

			/* When Acknowledge failure occurs (Slave don't acknowledge it's address)
			 Master restarts communication */
		} while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

		/*##-3- Master sends number of data to be written ########################*/
		do {
			if (HAL_I2C_Master_Transmit_IT(&hi2c4, (uint16_t) I2C_ADDRESS,
					(uint8_t*) &hTxNumData, 2) != HAL_OK) {
				/* Error_Handler() function is called when error occurs. */
				Error_Handler();
			}

			/*  Before starting a new communication transfer, you need to check the current
			 state of the peripheral; if it’s busy you need to wait for the end of current
			 transfer before starting a new one.
			 For simplicity reasons, this example is just waiting till the end of the
			 transfer, but application may perform other tasks while transfer operation
			 is ongoing. */
			while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) {
			}

			/* When Acknowledge failure occurs (Slave don't acknowledge it's address)
			 Master restarts communication */
		} while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

		/*##-4- Master sends aTxBuffer to slave ##################################*/
	    do
	    {
	      if (HAL_I2C_Master_Transmit_IT(&hi2c4, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
	      {
	        /* Error_Handler() function is called when error occurs. */
	        Error_Handler();
	      }

	      /*  Before starting a new communication transfer, you need to check the current
	          state of the peripheral; if it’s busy you need to wait for the end of current
	          transfer before starting a new one.
	          For simplicity reasons, this example is just waiting till the end of the
	          transfer, but application may perform other tasks while transfer operation
	          is ongoing. */
	      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY)
	      {
	      }

	      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
	         Master restarts communication */
	    }
	    while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

	    /* Update bTransferRequest to send buffer read request for Slave */
	    bTransferRequest = MASTER_REQ_READ;

	    /*##-5- Master sends read request for slave ##############################*/
	    do
	    {
	      if (HAL_I2C_Master_Transmit_IT(&hi2c4, (uint16_t)I2C_ADDRESS, (uint8_t *)&bTransferRequest, 1) != HAL_OK)
	      {
	        /* Error_Handler() function is called when error occurs. */
	        Error_Handler();
	      }

	      /*  Before starting a new communication transfer, you need to check the current
	          state of the peripheral; if it’s busy you need to wait for the end of current
	          transfer before starting a new one.
	          For simplicity reasons, this example is just waiting till the end of the
	          transfer, but application may perform other tasks while transfer operation
	          is ongoing. */
	      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY)
	      {
	      }

	      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
	         Master restarts communication */
	    }
	    while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

	    /*##-6- Master sends number of data to be read ###########################*/
	    do
	    {
	      if (HAL_I2C_Master_Transmit_IT(&hi2c4, (uint16_t)I2C_ADDRESS, (uint8_t *)&hRxNumData, 2) != HAL_OK)
	      {
	        /* Error_Handler() function is called when error occurs. */
	        Error_Handler();
	      }

	      /*  Before starting a new communication transfer, you need to check the current
	          state of the peripheral; if it’s busy you need to wait for the end of current
	          transfer before starting a new one.
	          For simplicity reasons, this example is just waiting till the end of the
	          transfer, but application may perform other tasks while transfer operation
	          is ongoing. */
	      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY)
	      {
	      }

	      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
	         Master restarts communication */
	    }
	    while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

	    /*##-7- Master receives aRxBuffer from slave #############################*/
	    do
	    {
	      if (HAL_I2C_Master_Receive_IT(&hi2c4, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	      {
	        /* Error_Handler() function is called when error occurs. */
	        Error_Handler();
	      }

	      /*  Before starting a new communication transfer, you need to check the current
	          state of the peripheral; if it’s busy you need to wait for the end of current
	          transfer before starting a new one.
	          For simplicity reasons, this example is just waiting till the end of the
	          transfer, but application may perform other tasks while transfer operation
	          is ongoing. */
	      while (HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY)
	      {
	      }

	      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
	         Master restarts communication */
	    }
	    while (HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF);

	    /* Check correctness of received buffer ##################################*/
	    if (Buffercmp((uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, hRxNumData))
	    {
	      /* Processing Error */
	      Error_Handler();
	    }

	    /* Flush Rx buffers */
	    Flush_Buffer((uint8_t *)aRxBuffer, RXBUFFERSIZE);

	    /* Toggle LED2 */
	    HAL_GPIO_TogglePin(GPIOB, LED_STATUS_Pin);

	    /* This delay permits to see LED2 toggling */
	    HAL_Delay(25);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC12
			| RCC_PERIPHCLK_ADC345;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
	PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.GainCompensation = 0;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.GainCompensation = 0;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc3.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief ADC4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC4_Init(void) {

	/* USER CODE BEGIN ADC4_Init 0 */

	/* USER CODE END ADC4_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC4_Init 1 */

	/* USER CODE END ADC4_Init 1 */
	/** Common config
	 */
	hadc4.Instance = ADC4;
	hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc4.Init.Resolution = ADC_RESOLUTION_12B;
	hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc4.Init.GainCompensation = 0;
	hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc4.Init.LowPowerAutoWait = DISABLE;
	hadc4.Init.ContinuousConvMode = DISABLE;
	hadc4.Init.NbrOfConversion = 1;
	hadc4.Init.DiscontinuousConvMode = DISABLE;
	hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc4.Init.DMAContinuousRequests = DISABLE;
	hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc4.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc4) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC4_Init 2 */

	/* USER CODE END ADC4_Init 2 */

}

/**
 * @brief ADC5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC5_Init(void) {

	/* USER CODE BEGIN ADC5_Init 0 */

	/* USER CODE END ADC5_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC5_Init 1 */

	/* USER CODE END ADC5_Init 1 */
	/** Common config
	 */
	hadc5.Instance = ADC5;
	hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc5.Init.Resolution = ADC_RESOLUTION_12B;
	hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc5.Init.GainCompensation = 0;
	hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc5.Init.LowPowerAutoWait = DISABLE;
	hadc5.Init.ContinuousConvMode = DISABLE;
	hadc5.Init.NbrOfConversion = 1;
	hadc5.Init.DiscontinuousConvMode = DISABLE;
	hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc5.Init.DMAContinuousRequests = DISABLE;
	hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc5.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc5) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC5_Init 2 */

	/* USER CODE END ADC5_Init 2 */

}

/**
 * @brief COMP3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP3_Init(void) {

	/* USER CODE BEGIN COMP3_Init 0 */

	/* USER CODE END COMP3_Init 0 */

	/* USER CODE BEGIN COMP3_Init 1 */

	/* USER CODE END COMP3_Init 1 */
	hcomp3.Instance = COMP3;
	hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO2;
	hcomp3.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
	hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP3_Init 2 */

	/* USER CODE END COMP3_Init 2 */

}

/**
 * @brief COMP5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP5_Init(void) {

	/* USER CODE BEGIN COMP5_Init 0 */

	/* USER CODE END COMP5_Init 0 */

	/* USER CODE BEGIN COMP5_Init 1 */

	/* USER CODE END COMP5_Init 1 */
	hcomp5.Instance = COMP5;
	hcomp5.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp5.Init.InputMinus = COMP_INPUT_MINUS_DAC4_CH1;
	hcomp5.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp5.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp5.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp5.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP5_Init 2 */

	/* USER CODE END COMP5_Init 2 */

}

/**
 * @brief COMP6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP6_Init(void) {

	/* USER CODE BEGIN COMP6_Init 0 */

	/* USER CODE END COMP6_Init 0 */

	/* USER CODE BEGIN COMP6_Init 1 */

	/* USER CODE END COMP6_Init 1 */
	hcomp6.Instance = COMP6;
	hcomp6.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp6.Init.InputMinus = COMP_INPUT_MINUS_DAC2_CH1;
	hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp6.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP6_Init 2 */

	/* USER CODE END COMP6_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */
	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief DAC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC2_Init(void) {

	/* USER CODE BEGIN DAC2_Init 0 */

	/* USER CODE END DAC2_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC2_Init 1 */

	/* USER CODE END DAC2_Init 1 */
	/** DAC Initialization
	 */
	hdac2.Instance = DAC2;
	if (HAL_DAC_Init(&hdac2) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC2_Init 2 */

	/* USER CODE END DAC2_Init 2 */

}

/**
 * @brief DAC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC3_Init(void) {

	/* USER CODE BEGIN DAC3_Init 0 */

	/* USER CODE END DAC3_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC3_Init 1 */

	/* USER CODE END DAC3_Init 1 */
	/** DAC Initialization
	 */
	hdac3.Instance = DAC3;
	if (HAL_DAC_Init(&hdac3) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC3_Init 2 */

	/* USER CODE END DAC3_Init 2 */

}

/**
 * @brief DAC4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC4_Init(void) {

	/* USER CODE BEGIN DAC4_Init 0 */

	/* USER CODE END DAC4_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC4_Init 1 */

	/* USER CODE END DAC4_Init 1 */
	/** DAC Initialization
	 */
	hdac4.Instance = DAC4;
	if (HAL_DAC_Init(&hdac4) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac4, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	if (HAL_DAC_ConfigChannel(&hdac4, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC4_Init 2 */

	/* USER CODE END DAC4_Init 2 */

}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void) {

	/* USER CODE BEGIN I2C4_Init 0 */

	/* USER CODE END I2C4_Init 0 */

	/* USER CODE BEGIN I2C4_Init 1 */

	/* USER CODE END I2C4_Init 1 */
	hi2c4.Instance = I2C4;
	hi2c4.Init.Timing = 0x00506682; //100 kHz
	hi2c4.Init.OwnAddress1 = 0;
	hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.OwnAddress2 = 0;
	hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C4_Init 2 */

	/* USER CODE END I2C4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			nHOLD_FLASH_Pin | RS_TFT_Pin | FAN_Pin | HINPWM_BUCK_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LINPWM_BOOST_Pin | SPI_CS_FLASH_Pin | LED_STATUS_Pin | USER_LED_Pin
					| CAN_TX_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, HINPWM_BOOST_Pin | LINPWM_BUCK_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : NC_Pin NCC0_Pin NCC2_Pin NCC3_Pin
	 NCC9_Pin NCC11_Pin NCC12_Pin */
	GPIO_InitStruct.Pin = NC_Pin | NCC0_Pin | NCC2_Pin | NCC3_Pin | NCC9_Pin
			| NCC11_Pin | NCC12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : nHOLD_FLASH_Pin RS_TFT_Pin FAN_Pin HINPWM_BUCK_Pin */
	GPIO_InitStruct.Pin = nHOLD_FLASH_Pin | RS_TFT_Pin | FAN_Pin
			| HINPWM_BUCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : NCA1_Pin TE_TFT_Pin NCA4_Pin NCA5_Pin
	 NCA6_Pin NCA7_Pin */
	GPIO_InitStruct.Pin = NCA1_Pin | TE_TFT_Pin | NCA4_Pin | NCA5_Pin | NCA6_Pin
			| NCA7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LINPWM_BOOST_Pin SPI_CS_FLASH_Pin LED_STATUS_Pin USER_LED_Pin
	 CAN_TX_Pin */
	GPIO_InitStruct.Pin = LINPWM_BOOST_Pin | SPI_CS_FLASH_Pin | LED_STATUS_Pin
			| USER_LED_Pin | CAN_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : USER_LED_Pin   */
	GPIO_InitStruct.Pin = USER_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : NCB10_Pin NCB4_Pin BTN_USER_Pin BOOT0_Pin */
	GPIO_InitStruct.Pin = NCB10_Pin | NCB4_Pin | BTN_USER_Pin | BOOT0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : HINPWM_BOOST_Pin LINPWM_BUCK_Pin */
	GPIO_InitStruct.Pin = HINPWM_BOOST_Pin | LINPWM_BUCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : NCD2_Pin */
	GPIO_InitStruct.Pin = NCD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(NCD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

/**
  * @brief  Flushes the buffer
  * @param  pBuffer: buffers to be flushed.
  * @param  BufferLength: buffer's length
  * @retval None
  */
static void Flush_Buffer(uint8_t *pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;

    pBuffer++;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* Error if LED_STATUS is slowly blinking (1 sec. period) */
	while (1) {
		HAL_GPIO_TogglePin(GPIOB,LED_STATUS_Pin);
		HAL_Delay(1000);
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
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
