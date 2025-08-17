#include "main.h"
#include "rw_hal.h"
#include "main.h"
#include "rw_sh1106.h"
#include "stdbool.h"
#include "stm32g4xx_ll_spi.h"
extern ADC_HandleTypeDef hadc1;
uint32_t adc_vsys=0;
uint32_t adc_usb=0;
uint32_t adc_current=0;
void rw_led(char r, char g, char b)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, !r);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, !g);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, !b);
}

void rw_read_adc()
{
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  adc_vsys = HAL_ADC_GetValue(&hadc1)/ 1.77;
  HAL_ADC_PollForConversion(&hadc1, 1);
  adc_usb = HAL_ADC_GetValue(&hadc1) / 3.276;
  HAL_ADC_PollForConversion(&hadc1, 1);
  //not working in 4a and 4b
  adc_current = HAL_ADC_GetValue(&hadc1);
}

uint8_t rw_chargestate(void) // 0 - not charging, 1 - charging, 2 - charged
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = CHARGE_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CHARGE_STATE_GPIO_Port, &GPIO_InitStruct);
  HAL_Delay(1);
  uint16_t pullup = HAL_GPIO_ReadPin(CHARGE_STATE_GPIO_Port, CHARGE_STATE_Pin);
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CHARGE_STATE_GPIO_Port, &GPIO_InitStruct);
  HAL_Delay(1);
  uint16_t pulldown = HAL_GPIO_ReadPin(CHARGE_STATE_GPIO_Port, CHARGE_STATE_Pin);
  if (pulldown == pullup)
  {
    if (pullup)
    {
      return 2;
    }
    else
    {
      return 1;
    }
  }
  else
  {
    return 0;
  }
}

void rw_chargeswitch(char state) 
  {
  if (state)
  {
    HAL_GPIO_WritePin(CHARGE_NEN_GPIO_Port, CHARGE_NEN_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(CHARGE_NEN_GPIO_Port, CHARGE_NEN_Pin, GPIO_PIN_SET);
  }
}

void rw_powerswitch(char state) 
{
#ifdef KIISU_VERSION_4A
  if (state)
  {
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_RESET);
  }
#else
  if (state)
  {
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(POWER_EN_GPIO_Port, POWER_EN_Pin, GPIO_PIN_SET);
  }  
#endif
}

void rw_sh1106_spi_write(uint8_t* buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    LL_SPI_TransmitData8(OLED_SPI, buf[i]);
    while (LL_I2S_IsActiveFlag_BSY(OLED_SPI)){};
  }
}

void rw_sh1106_pins_set(bool cs, bool dc, bool rst)
{
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, dc);
  HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, rst);
}

void rw_display_init(void)
{
  rw_sh1106_init();
  rw_display_drawing_start();
  rw_sh1106_fill(0);
  rw_sh1106_setposition(0, 0);
  rw_sh1106_print("Kiisu is starting...");
  rw_display_drawing_end();
}

void display_test(void)
{
  rw_display_drawing_start();
  rw_sh1106_fill(0);
  for (int i = 0; i < 255; i++)
  {
    rw_sh1106_print("B");
    HAL_Delay(100);
  }
  rw_display_drawing_end();

  rw_sh1106_pins_set(0, 0, 0);
  HAL_Delay(100);
  rw_sh1106_pins_set(0, 0, 1);
  HAL_Delay(1000);
}