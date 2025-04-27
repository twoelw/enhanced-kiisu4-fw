#include "main.h"
#include "rw_hal.h"
#include "main.h"
#include "rw_sh1106.h"
#include "stdbool.h"
#include "stm32g4xx_ll_spi.h"
extern ADC_HandleTypeDef hadc1;
uint32_t adc3=0;
uint32_t adc_usb_raw=0;
uint32_t adc15=0;
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
  adc3 = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  adc_usb_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  adc15 = HAL_ADC_GetValue(&hadc1);
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
  rw_sh1106_fill(0);
  rw_sh1106_setposition(0, 0);
  rw_sh1106_print("Kiisu is starting...");
}

void display_test(void)
{
  rw_sh1106_fill(0);
  for (int i = 0; i < 255; i++)
  {
    rw_sh1106_print("B");
    HAL_Delay(100);
  }

  rw_sh1106_pins_set(0, 0, 0);
  HAL_Delay(100);
  rw_sh1106_pins_set(0, 0, 1);
  HAL_Delay(1000);
}