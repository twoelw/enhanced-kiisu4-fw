/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g431xx.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_spi.h"
#include "rw_i2c_emu.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t i2c_address = 0;
uint8_t i2c_direction = 0;
uint8_t i2c_pointer = 0;
uint8_t i2c_register = 0;


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
volatile bool companion_dc_state;
volatile bool companion_cs_state;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(DISPLAY_CS_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PB6_DISP_DI_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
  
  if (LL_I2C_IsActiveFlag_ADDR(I2C1))
  {
    i2c_address = LL_I2C_GetAddressMatchCode(I2C1)/2;
    i2c_direction = LL_I2C_GetTransferDirection(I2C1);
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_ClearFlag_TXE(I2C1);
  }

  if (LL_I2C_IsActiveFlag_NACK(I2C1))
  {
    LL_I2C_ClearFlag_NACK(I2C1);
  }

  while (LL_I2C_IsActiveFlag_RXNE(I2C1))
  {
    uint8_t data = LL_I2C_ReceiveData8(I2C1);
    if (i2c_direction == LL_I2C_DIRECTION_WRITE)
    {
      if (i2c_pointer == 0)
      {
        i2c_register = data;
      }
      else
      {
        rw_i2c_reg_written(i2c_address, i2c_register++, data);     
      }
    }
    else
    {
      
    }
    i2c_pointer++;
  }
  if (LL_I2C_IsActiveFlag_TXE(I2C1))
  {
    // Implement data sending logic here if needed
    //LL_I2C_GenerateStopCondition(I2C1);
    LL_I2C_ClearFlag_TXE(I2C1);
  }
  
  
  if (LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
      LL_I2C_TransmitData8(I2C1, rw_i2c_get_reg(i2c_address,i2c_register++));
    // Implement data sending logic here if needed
  }

  if (LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    LL_I2C_ClearFlag_STOP(I2C1);
    i2c_pointer = 0;
  }
  /* USER CODE END I2C1_EV_IRQn 0 */
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
  while (LL_SPI_IsActiveFlag_RXNE(COMPANION_SPI))
  {
    // if (HAL_SPI_Receive(&hspi1, buf, 1, 1000) == HAL_OK)
    // {
    // reinterpret_cast<uint8_t *>(&tmp)[1] =
    // LL_GPIO_IsInputPinSet(INPUT_DISPLAY_DI_PORT, INPUT_DISPLAY_DI_PIN);
    // reinterpret_cast<uint8_t *>(&tmp)[0] = LL_SPI_ReceiveData8(spi);
    uint8_t data = LL_SPI_ReceiveData8(COMPANION_SPI);
    // data = buf[0];
    if (companion_dc_state)
    {
      LL_GPIO_SetOutputPin(OLED_DC_GPIO_Port, OLED_DC_Pin);
      LL_SPI_TransmitData8(OLED_SPI, data);
    }
    else
    {
      if (data == 0x00 || data == 0x10) // || ((data | 0x0f) == 0xbf)
      {
        LL_GPIO_ResetOutputPin(OLED_DC_GPIO_Port, OLED_DC_Pin);
        LL_SPI_TransmitData8(OLED_SPI, data);
      }
    }
    // hspi2.Instance->DR = buf[0];
  }
 /* if (companion_cs_state)
  {
    LL_SPI_Disable(COMPANION_SPI);
  }*/
  if (LL_SPI_IsActiveFlag_OVR(COMPANION_SPI))
  {
    LL_SPI_ClearFlag_OVR(COMPANION_SPI);
  }
  /* USER CODE END SPI1_IRQn 0 */
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* EXTI line interrupt detected */
  if (GPIO_Pin == PB6_DISP_DI_Pin)
  {
    companion_dc_state =
      HAL_GPIO_ReadPin(PB6_DISP_DI_GPIO_Port, PB6_DISP_DI_Pin) == GPIO_PIN_SET;
  }

  if (GPIO_Pin == DISPLAY_CS_Pin)
  {
    SET_BIT(SPI1->CR1, SPI_CR1_SSM);

    if (HAL_GPIO_ReadPin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin) == GPIO_PIN_SET)
    {
      SET_BIT(SPI1->CR1, SPI_CR1_SSI);
    }
    else
    {
      CLEAR_BIT(SPI1->CR1, SPI_CR1_SSI);
    }
  } 


}

/* USER CODE END 1 */

/**
  * @brief This function handles TIM6 global interrupt and DAC underrun errors.
  */
void TIM6_DAC_IRQHandler(void)
{
  // Check update flag
  if (TIM6->SR & TIM_SR_UIF)
  {
    TIM6->SR &= ~TIM_SR_UIF; // clear
    extern void led_pwm_isr_step(void);
    led_pwm_isr_step(); // run at 10 kHz
  }
}
