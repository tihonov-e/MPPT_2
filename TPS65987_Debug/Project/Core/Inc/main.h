/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NC_Pin GPIO_PIN_13
#define NC_GPIO_Port GPIOC
#define NCC0_Pin GPIO_PIN_0
#define NCC0_GPIO_Port GPIOC
#define I_INDUCTOR_Pin GPIO_PIN_1
#define I_INDUCTOR_GPIO_Port GPIOC
#define NCC2_Pin GPIO_PIN_2
#define NCC2_GPIO_Port GPIOC
#define NCC3_Pin GPIO_PIN_3
#define NCC3_GPIO_Port GPIOC
#define nHOLD_FLASH_Pin GPIO_PIN_0
#define nHOLD_FLASH_GPIO_Port GPIOA
#define NCA1_Pin GPIO_PIN_1
#define NCA1_GPIO_Port GPIOA
#define RS_TFT_Pin GPIO_PIN_2
#define RS_TFT_GPIO_Port GPIOA
#define TE_TFT_Pin GPIO_PIN_3
#define TE_TFT_GPIO_Port GPIOA
#define NCA4_Pin GPIO_PIN_4
#define NCA4_GPIO_Port GPIOA
#define NCA5_Pin GPIO_PIN_5
#define NCA5_GPIO_Port GPIOA
#define NCA6_Pin GPIO_PIN_6
#define NCA6_GPIO_Port GPIOA
#define NCA7_Pin GPIO_PIN_7
#define NCA7_GPIO_Port GPIOA
#define PCB_TEMP_Pin GPIO_PIN_0
#define PCB_TEMP_GPIO_Port GPIOB
#define LINPWM_BOOST_Pin GPIO_PIN_1
#define LINPWM_BOOST_GPIO_Port GPIOB
#define SPI_CS_FLASH_Pin GPIO_PIN_2
#define SPI_CS_FLASH_GPIO_Port GPIOB
#define NCB10_Pin GPIO_PIN_10
#define NCB10_GPIO_Port GPIOB
#define I_INPUT_Pin GPIO_PIN_11
#define I_INPUT_GPIO_Port GPIOB
#define I_INPUTB12_Pin GPIO_PIN_12
#define I_INPUTB12_GPIO_Port GPIOB
#define I_OUTPUT_Pin GPIO_PIN_13
#define I_OUTPUT_GPIO_Port GPIOB
#define I_OUTPUTB14_Pin GPIO_PIN_14
#define I_OUTPUTB14_GPIO_Port GPIOB
#define I_EXT_Pin GPIO_PIN_15
#define I_EXT_GPIO_Port GPIOB
#define HINPWM_BOOST_Pin GPIO_PIN_8
#define HINPWM_BOOST_GPIO_Port GPIOC
#define NCC9_Pin GPIO_PIN_9
#define NCC9_GPIO_Port GPIOC
#define V_OUTPUT_Pin GPIO_PIN_8
#define V_OUTPUT_GPIO_Port GPIOA
#define V_INPUT_Pin GPIO_PIN_9
#define V_INPUT_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_10
#define FAN_GPIO_Port GPIOA
#define HINPWM_BUCK_Pin GPIO_PIN_15
#define HINPWM_BUCK_GPIO_Port GPIOA
#define LINPWM_BUCK_Pin GPIO_PIN_10
#define LINPWM_BUCK_GPIO_Port GPIOC
#define NCC11_Pin GPIO_PIN_11
#define NCC11_GPIO_Port GPIOC
#define NCC12_Pin GPIO_PIN_12
#define NCC12_GPIO_Port GPIOC
#define NCD2_Pin GPIO_PIN_2
#define NCD2_GPIO_Port GPIOD
#define NCB4_Pin GPIO_PIN_4
#define NCB4_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_5
#define USER_LED_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_6
#define LED_STATUS_GPIO_Port GPIOB
#define BTN_USER_Pin GPIO_PIN_7
#define BTN_USER_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define I2C_ADDRESS 0x00

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
