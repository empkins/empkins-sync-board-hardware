/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t saveReceivedData[50]; // buffer USB serial communication 6 Bytes
extern uint8_t dataReceived;
extern uint8_t button1_event;
extern uint8_t button2_event;
extern uint8_t md6_event;
extern uint8_t md5_event;

int Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
int ReadData(uint8_t buf[]);
void LightLED(uint8_t color);
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len);
void PrintError(uint8_t errorMessage);
void SendEventMessage(uint8_t message);
void SendMessage(uint8_t feature, uint8_t message, uint8_t length);
void FreeAllMalloc();
void InitSignalformAll();
void SetStartSource(uint8_t newStartSource, uint16_t delay, uint8_t signalform);
void SetStopSource(uint8_t newStopSource, uint16_t delay, uint8_t signalform);
void SetEvent(uint8_t eventSource, uint8_t active, uint8_t signalform);
void SetOutput(uint8_t device, uint8_t active, uint16_t delay, uint8_t signalform, uint16_t puls_length, uint16_t frequency, uint8_t stopTrigger, uint8_t degree);
void StartMeasurement(uint8_t usedSource);
void StopMeasurement(uint8_t usedSource);
void UpdateStartTrigger();
void UpdateStopTrigger();
void WhoAmI();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Button1_Pin GPIO_PIN_7
#define Button1_GPIO_Port GPIOA
#define Button1_EXTI_IRQn EXTI9_5_IRQn
#define Button2_Pin GPIO_PIN_0
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI0_IRQn
#define MD1_O_Pin GPIO_PIN_1
#define MD1_O_GPIO_Port GPIOB
#define MD7_O_Pin GPIO_PIN_2
#define MD7_O_GPIO_Port GPIOB
#define MD6_O_Pin GPIO_PIN_12
#define MD6_O_GPIO_Port GPIOB
#define MD6_I_Pin GPIO_PIN_13
#define MD6_I_GPIO_Port GPIOB
#define MD6_I_EXTI_IRQn EXTI15_10_IRQn
#define MD5_I_Pin GPIO_PIN_9
#define MD5_I_GPIO_Port GPIOA
#define MD5_I_EXTI_IRQn EXTI9_5_IRQn
#define MD5_O_Pin GPIO_PIN_10
#define MD5_O_GPIO_Port GPIOA
#define MD4_O_Pin GPIO_PIN_15
#define MD4_O_GPIO_Port GPIOA
#define MD3_O_Pin GPIO_PIN_3
#define MD3_O_GPIO_Port GPIOB
#define LED_blue_Pin GPIO_PIN_6
#define LED_blue_GPIO_Port GPIOB
#define LED_green_Pin GPIO_PIN_7
#define LED_green_GPIO_Port GPIOB
#define LED_red_Pin GPIO_PIN_8
#define LED_red_GPIO_Port GPIOB
#define MD2_O_Pin GPIO_PIN_9
#define MD2_O_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
