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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */




void InicializaVariable(void);
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Dis_eco_Pin GPIO_PIN_3
#define Dis_eco_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define User_button_Pin GPIO_PIN_4
#define User_button_GPIO_Port GPIOC
#define Enable_R_Pin GPIO_PIN_1
#define Enable_R_GPIO_Port GPIOB
#define Enable_L_Pin GPIO_PIN_2
#define Enable_L_GPIO_Port GPIOB
#define Trigger_Pin GPIO_PIN_14
#define Trigger_GPIO_Port GPIOB
#define Brake_L_Pin GPIO_PIN_8
#define Brake_L_GPIO_Port GPIOC
#define Brake_R_Pin GPIO_PIN_9
#define Brake_R_GPIO_Port GPIOC
#define Led_amarillo_Pin GPIO_PIN_8
#define Led_amarillo_GPIO_Port GPIOA
#define Z_F_R_Pin GPIO_PIN_11
#define Z_F_R_GPIO_Port GPIOA
#define Z_F_L_Pin GPIO_PIN_12
#define Z_F_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BT_State_Pin GPIO_PIN_3
#define BT_State_GPIO_Port GPIOB
#define Led_verde_Pin GPIO_PIN_4
#define Led_verde_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
