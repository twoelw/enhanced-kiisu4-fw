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
#include "rw_hal.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_spi.h"
#include "rw_sh1106.h"
#include "rw_i2c_emu.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void handle_back_button(void);  // Function declaration
void handle_shutdown_request(void);  // Clean shutdown function declaration
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Add these variables for smooth breathing effect
uint32_t breath_cycle_time = 0;
uint8_t breath_phase = 0; // 0 = breathing in, 1 = breathing out, 2 = dark period
uint32_t breath_phase_start = 0;
uint8_t breath_brightness = 0;

// Variables for smooth PWM
uint32_t pwm_counter = 0;
uint32_t pwm_last_update = 0;

// OLED brightness control
uint8_t last_backlight_value = 255; // Initialize to max to ensure first update
uint8_t oled_drawing_in_progress = 0; // Flag to prevent brightness changes during drawing
uint32_t last_brightness_check = 0; // Timestamp for brightness checking

// Shutdown request queueing
uint8_t shutdown_request_queued = 0; // Flag to indicate a shutdown was requested while USB connected

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
  rw_i2c_emu_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  rw_powerswitch(1);
  HAL_Delay(200);
  rw_display_init();
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    handle_back_button();  // Check for shutdown sequence
    uint8_t charge_state = rw_chargestate();
    rw_read_adc();
    rw_led(0, 0, 0);
    if (adc_usb > 4400)
    {
      rw_chargeswitch(1);
      
      // OLED brightness control based on virtual backlight
      // Only check brightness every 50ms to reduce conflicts with drawing
      if (HAL_GetTick() - last_brightness_check >= 50) {
        uint8_t current_backlight = rw_i2c_get_backlight();
        if (current_backlight != last_backlight_value && !oled_drawing_in_progress) {
          if (current_backlight == 0) {
            // Backlight off - turn off OLED display completely
            rw_display_off();
          } else {
            // Backlight on - ensure display is on and set brightness
            rw_display_on();
            rw_display_set_brightness(current_backlight);
          }
          last_backlight_value = current_backlight;
        }
        last_brightness_check = HAL_GetTick();
      }
      
      if (charge_state == 0)
      {
        rw_led(0, 0, 1); // blue, not charging
        rw_i2c_set_battery(adc_vsys, adc_usb, 0, 0);
      }
      else if (charge_state == 1)
      {
        rw_led(1, 1, 0); // yellow, charginh
        rw_i2c_set_battery(3700, adc_usb, 20, 1);
      }
      else if (charge_state == 2)
      {
        rw_led(0, 1, 0); // green, charged
        rw_i2c_set_battery(4200, adc_usb, 0, 2);
      }
    }
    else // Not connected to USB
    {
      // Check for queued shutdown request when USB is unplugged
      if (shutdown_request_queued) {
        // Execute the queued shutdown immediately
        rw_led(0, 0, 0); // Turn off all LEDs
        rw_display_off(); // Turn off display
        rw_powerswitch(0); // Power off the device
        while (1); // Hang here as device will power off
      }
      
      rw_i2c_set_battery(adc_vsys, 0, -20, 0);
      rw_chargeswitch(0);
      
      // OLED brightness control based on virtual backlight
      // Only check brightness every 50ms to reduce conflicts with drawing
      if (HAL_GetTick() - last_brightness_check >= 50) {
        uint8_t current_backlight = rw_i2c_get_backlight();
        if (current_backlight != last_backlight_value && !oled_drawing_in_progress) {
          if (current_backlight == 0) {
            // Backlight off - turn off OLED display completely
            rw_display_off();
          } else {
            // Backlight on - ensure display is on and set brightness
            rw_display_on();
            rw_display_set_brightness(current_backlight);
          }
          last_backlight_value = current_backlight;
        }
        last_brightness_check = HAL_GetTick();
      }
      
      if (rw_i2c_get_backlight() == 0) // Backlight is off
      {
        // 3-second warning before entering idle
        uint32_t warning_start = HAL_GetTick();
        uint8_t warning_cancelled = 0;
        
        while (HAL_GetTick() - warning_start < 3000) // 3-second warning
        {
          // Rapid flashing red LED (100ms on, 100ms off)
          if ((HAL_GetTick() % 200) < 100) {
            rw_led(1, 0, 0); // Red on
          } else {
            rw_led(0, 0, 0); // Red off
          }
          
          // Check if backlight turned back on (cancel warning)
          if (rw_i2c_get_backlight() != 0) {
            warning_cancelled = 1;
            break;
          }
          
          HAL_Delay(10); // Small delay to reduce CPU load
        }
        
        // Turn off LED after warning
        rw_led(0, 0, 0);
        
        // If warning wasn't cancelled, enter idle mode
        if (!warning_cancelled) {
          rw_display_off(); // Turn off display
          
          uint32_t idle_start_time = HAL_GetTick();
          uint32_t auto_shutdown_time = 30 * 60 * 1000; // 30 minutes in ms
          
          // Variables for non-blocking PWM breathing effect
          uint32_t cycle_start_time = HAL_GetTick();
          uint32_t pwm_state_change_time = HAL_GetTick();
          uint8_t pwm_state = 0; // 0 = off, 1 = on
          uint32_t pwm_period_ms = 20; // 20ms period
          
          while (1) // Idle loop
          {
            // Check if backlight turned back on (exit idle)
            if (rw_i2c_get_backlight() != 0) {
              break;
            }
            
            // Check for 30-minute auto-shutdown
            if (HAL_GetTick() - idle_start_time >= auto_shutdown_time) {
              // 10-second shutdown warning
              uint32_t shutdown_warning_start = HAL_GetTick();
              uint8_t shutdown_cancelled = 0;
              
              // Turn off breathing effect during warning
              rw_led(0, 0, 0);
              
              while (HAL_GetTick() - shutdown_warning_start < 10000) // 10-second warning
              {
                // Rapid flashing red LED (100ms on, 100ms off)
                if ((HAL_GetTick() % 200) < 100) {
                  rw_led(1, 0, 0); // Red on
                } else {
                  rw_led(0, 0, 0); // Red off
                }
                
                // Check if backlight turned back on (cancel shutdown)
                if (rw_i2c_get_backlight() != 0) {
                  shutdown_cancelled = 1;
                  break;
                }
                
                HAL_Delay(10); // Small delay to reduce CPU load
              }
              
              // If shutdown wasn't cancelled, power off
              if (!shutdown_cancelled) {
                rw_powerswitch(0); // Power off the device
              } else {
                // Reset auto-shutdown timer if cancelled
                idle_start_time = HAL_GetTick();
                rw_led(0, 0, 0); // Turn off LED
                // Restore OLED brightness after cancelling shutdown
                uint8_t current_backlight = rw_i2c_get_backlight();
                if (current_backlight > 0) {
                  rw_display_on();
                  rw_display_set_brightness(current_backlight);
                }
                // Re-initialize breathing variables after cancelling shutdown
                cycle_start_time = HAL_GetTick();
                pwm_state_change_time = HAL_GetTick();
                pwm_state = 0;
              }
            }
            
            // Non-blocking PWM breathing effect for red LED
            uint32_t current_time = HAL_GetTick();
            uint32_t elapsed_in_cycle = (current_time - cycle_start_time) % 5000; // 5-second cycle
            
            // Calculate brightness based on time in cycle
            uint32_t brightness = 0;
            if (elapsed_in_cycle < 1500) {
              // Ramp up: 0 to 2000 over 1500ms
              brightness = (elapsed_in_cycle * 2000) / 1500;
            } else if (elapsed_in_cycle < 3000) {
              // Ramp down: 2000 to 0 over 1500ms
              brightness = 2000 - ((elapsed_in_cycle - 1500) * 2000) / 1500;
            }
            // Else: 2000ms to 5000ms is darkness, brightness remains 0
            
            // Set minimum brightness threshold to ensure LED goes completely off
            if (brightness < 50) { // Below 2.5% brightness, turn LED completely off
              rw_led(0, 0, 0); // LED off
            } else {
              // Calculate the on-time in the PWM period (in ms)
              uint32_t on_time_ms = (brightness * pwm_period_ms) / 2000;
              
              // Ensure minimum on-time for visible brightness
              if (on_time_ms < 1) {
                on_time_ms = 1; // Minimum 1ms on-time
              }
              
              // Check if we need to change the PWM state
              if (current_time - pwm_state_change_time >= (pwm_state ? on_time_ms : (pwm_period_ms - on_time_ms))) {
                // Toggle PWM state
                pwm_state = !pwm_state;
                pwm_state_change_time = current_time;
                
                // Set LED based on new state
                if (pwm_state) {
                  rw_led(1, 0, 0); // Red on
                } else {
                  rw_led(0, 0, 0); // Red off
                }
              }
            }
            
            HAL_Delay(1); // Small delay to reduce CPU load
          }
          
          // Exiting idle mode - turn display back on
          rw_display_on();
          // Restore brightness based on current backlight value
          uint8_t current_backlight = rw_i2c_get_backlight();
          if (current_backlight > 0) {
            rw_display_set_brightness(current_backlight);
          }
          rw_led(0, 0, 0); // Ensure LED is off
          last_backlight_value = current_backlight; // Update tracking variable
        }
      }
    }
    HAL_Delay(100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PA15   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0)); // Changed from 0 to 2 to avoid conflicts with OLED
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  
  /* Add Error Interrupt Priority */
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0)); // New addition to avoid conflicts with OLED
  NVIC_EnableIRQ(I2C1_ER_IRQn); // New addition to avoid conflicts with OLED

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00F12981;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 96;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_MASK07);
  LL_I2C_EnableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  LL_I2C_EnableIT_ADDR(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
  LL_I2C_EnableIT_NACK(I2C1);
  LL_I2C_EnableIT_RX(I2C1);
  LL_I2C_EnableIT_TC(I2C1);
  LL_I2C_EnableIT_TX(I2C1);
  LL_I2C_EnableIT_STOP(I2C1);
  LL_I2C_Enable(I2C1);

  /* USER CODE END I2C1_Init 2 */

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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PB3   ------> SPI1_SCK
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_SIMPLEX_RX;
  SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */
  LL_SPI_Enable(COMPANION_SPI);
  LL_SPI_SetRxFIFOThreshold(COMPANION_SPI, LL_SPI_RX_FIFO_TH_QUARTER);

  LL_SPI_EnableIT_ERR(COMPANION_SPI);
  LL_SPI_EnableIT_RXNE(COMPANION_SPI);

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB12   ------> SPI2_NSS
  PB13   ------> SPI2_SCK
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */
  LL_SPI_Enable(OLED_SPI);

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_B_Pin|LED_R_Pin|LED_G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, POWER_EN_Pin|PA7_SDR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_RES_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHARGE_NEN_GPIO_Port, CHARGE_NEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_B_Pin LED_R_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_R_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NC2_Pin NC3_Pin NC1_Pin */
  GPIO_InitStruct.Pin = NC2_Pin|NC3_Pin|NC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NC4_Pin NC5_Pin */
  GPIO_InitStruct.Pin = NC4_Pin|NC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : POWER_EN_Pin PA7_SDR_EN_Pin */
  GPIO_InitStruct.Pin = POWER_EN_Pin|PA7_SDR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDR_IN_1_Pin SDR_IN_2_Pin SDR_CNT1_Pin SDR_CNT2_Pin
                           SDR_EN_Pin CHARGE_STATE_Pin USB2_Pin USB1_Pin */
  GPIO_InitStruct.Pin = SDR_IN_1_Pin|SDR_IN_2_Pin|SDR_CNT1_Pin|SDR_CNT2_Pin
                          |SDR_EN_Pin|CHARGE_STATE_Pin|USB2_Pin|USB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPEAKER2_Pin */
  GPIO_InitStruct.Pin = SPEAKER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPEAKER2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISPLAY_CS_Pin */
  GPIO_InitStruct.Pin = DISPLAY_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISPLAY_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_PERIPH_Pin D_MISO_Pin BACK_BTN_Pin */
  GPIO_InitStruct.Pin = PB2_PERIPH_Pin|D_MISO_Pin|BACK_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_RES_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_RES_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NC6_Pin */
  GPIO_InitStruct.Pin = NC6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(NC6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGE_NEN_Pin */
  GPIO_InitStruct.Pin = CHARGE_NEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CHARGE_NEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6_DISP_DI_Pin */
  GPIO_InitStruct.Pin = PB6_DISP_DI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB6_DISP_DI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9_DISP_RST_Pin */
  GPIO_InitStruct.Pin = PB9_DISP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PB9_DISP_RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void handle_shutdown_request(void) {
  // Check if USB is connected (using same logic as main loop)
  rw_read_adc(); // Update ADC readings
  
  if (adc_usb > 4400) {
    // USB is connected - queue the shutdown request instead of executing immediately
    shutdown_request_queued = 1;
    return; // Don't shutdown now, wait for USB to be unplugged
  }
  
  // USB not connected - proceed with immediate shutdown
  // Clean shutdown without LED warnings - similar to battery disconnect
  // Turn off all LEDs immediately
  rw_led(0, 0, 0);
  
  // Turn off display
  rw_display_off();
  
  // Power off the device
  rw_powerswitch(0);
  
  // Hang here as device will power off
  while (1);
}

void handle_back_button(void) {
  static uint8_t press_count = 0;
  static uint32_t last_press_time = 0;
  static uint32_t press_start_time = 0;
  static uint8_t button_state = 0; // 0 = released, 1 = pressed
  uint8_t current_button_state = HAL_GPIO_ReadPin(BACK_BTN_GPIO_Port, BACK_BTN_Pin);
  
  // --- State change detection ---
  // RISING EDGE (Button was just pressed)
  if (current_button_state && !button_state) {
    button_state = 1;
    press_start_time = HAL_GetTick();
    // Reset press count if there's a >1s gap between presses
    if (HAL_GetTick() - last_press_time > 1000) {
      press_count = 1;
    } else {
      press_count++;
    }
    last_press_time = HAL_GetTick();
  }
  // FALLING EDGE (Button was just released)
  else if (!current_button_state && button_state) {
    button_state = 0;
  }
  
  // --- Action logic based on state ---
  // Check for 4-press-and-hold shutdown sequence
  if (press_count >= 4) {
    if (button_state) { // Button is currently held down
      if (HAL_GetTick() - press_start_time >= 3000) { // Held for 3 seconds
        // Flash the red led for 2 seconds and shut down.
        uint32_t shutdown_warning_start_time = HAL_GetTick();
        while (HAL_GetTick() - shutdown_warning_start_time < 2000) {
          rw_led(1, 0, 0); HAL_Delay(100);
          rw_led(0, 0, 0); HAL_Delay(100);
        }
        rw_powerswitch(0);
        // The device will now power off, so we hang here.
        while (1);
      }
    } else { // Button was released
      // This is a failed attempt (4 presses but no hold), so reset the counter.
      press_count = 0;
    }
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
