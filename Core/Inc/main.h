/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOC
#define NC2_Pin GPIO_PIN_14
#define NC2_GPIO_Port GPIOC
#define NC3_Pin GPIO_PIN_15
#define NC3_GPIO_Port GPIOC
#define NC4_Pin GPIO_PIN_0
#define NC4_GPIO_Port GPIOF
#define NC5_Pin GPIO_PIN_1
#define NC5_GPIO_Port GPIOF
#define POWER_EN_Pin GPIO_PIN_0
#define POWER_EN_GPIO_Port GPIOA
#define SDR_IN_1_Pin GPIO_PIN_1
#define SDR_IN_1_GPIO_Port GPIOA
#define BATTMON_Pin GPIO_PIN_2
#define BATTMON_GPIO_Port GPIOA
#define SDR_IN_2_Pin GPIO_PIN_3
#define SDR_IN_2_GPIO_Port GPIOA
#define SPEAKER2_Pin GPIO_PIN_4
#define SPEAKER2_GPIO_Port GPIOA
#define SDR_CNT1_Pin GPIO_PIN_5
#define SDR_CNT1_GPIO_Port GPIOA
#define SDR_CNT2_Pin GPIO_PIN_6
#define SDR_CNT2_GPIO_Port GPIOA
#define PA7_SDR_EN_Pin GPIO_PIN_7
#define PA7_SDR_EN_GPIO_Port GPIOA
#define DISPLAY_CS_Pin GPIO_PIN_4
#define DISPLAY_CS_GPIO_Port GPIOC
#define DISPLAY_CS_EXTI_IRQn EXTI4_IRQn
#define IMON_Pin GPIO_PIN_0
#define IMON_GPIO_Port GPIOB
#define VUSB_Pin GPIO_PIN_1
#define VUSB_GPIO_Port GPIOB
#define PB2_PERIPH_Pin GPIO_PIN_2
#define PB2_PERIPH_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_10
#define OLED_RES_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOB
#define NC6_Pin GPIO_PIN_14
#define NC6_GPIO_Port GPIOB
#define NC1_Pin GPIO_PIN_6
#define NC1_GPIO_Port GPIOC
#define SDR_EN_Pin GPIO_PIN_8
#define SDR_EN_GPIO_Port GPIOA
#define CHARGE_NEN_Pin GPIO_PIN_9
#define CHARGE_NEN_GPIO_Port GPIOA
#define CHARGE_STATE_Pin GPIO_PIN_10
#define CHARGE_STATE_GPIO_Port GPIOA
#define USB2_Pin GPIO_PIN_11
#define USB2_GPIO_Port GPIOA
#define USB1_Pin GPIO_PIN_12
#define USB1_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_10
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOC
#define D_SCK_Pin GPIO_PIN_3
#define D_SCK_GPIO_Port GPIOB
#define D_MISO_Pin GPIO_PIN_4
#define D_MISO_GPIO_Port GPIOB
#define D_MOSI_Pin GPIO_PIN_5
#define D_MOSI_GPIO_Port GPIOB
#define PB6_DISP_DI_Pin GPIO_PIN_6
#define PB6_DISP_DI_GPIO_Port GPIOB
#define PB6_DISP_DI_EXTI_IRQn EXTI9_5_IRQn
#define BACK_BTN_Pin GPIO_PIN_8
#define BACK_BTN_GPIO_Port GPIOB
#define PB9_DISP_RST_Pin GPIO_PIN_9
#define PB9_DISP_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define COMPANION_SPI SPI1
#define OLED_SPI SPI2

#define LP5562_CHANNEL_RED_CURRENT_REGISTER   0x07
#define LP5562_CHANNEL_GREEN_CURRENT_REGISTER 0x06
#define LP5562_CHANNEL_BLUE_CURRENT_REGISTER  0x05
#define LP5562_CHANNEL_WHITE_CURRENT_REGISTER 0x0F

#define LP5562_CHANNEL_RED_VALUE_REGISTER   0x04
#define LP5562_CHANNEL_GREEN_VALUE_REGISTER 0x03
#define LP5562_CHANNEL_BLUE_VALUE_REGISTER  0x02
#define LP5562_CHANNEL_WHITE_VALUE_REGISTER 0x0E
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
