#pragma once

#include <stdint.h>

// Tiny non-blocking LED PWM helper.
// Contract:
//  - Period is fixed to 20 ms.
//  - Duty range 0..2000 maps to 0..100%.
//  - Use led_pwm_tick(now_ms) every loop; no blocking allowed.

void led_pwm_init(void);
void led_pwm_set_color(uint8_t r, uint8_t g, uint8_t b);
void led_pwm_set_duty_0_2000(uint16_t duty);
void led_pwm_disable(void);
void led_pwm_tick(uint32_t now_ms);

// Optional ISR-driven mode
void led_pwm_isr_step(void); // call from a periodic timer ISR (e.g., 2 kHz)
void led_pwm_enable_timer_mode(void); // prefer ISR mode over main-loop tick
