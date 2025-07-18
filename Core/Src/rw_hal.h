#pragma once
#include "stdint.h"
#include "rw_display.h"

void rw_read_adc(void);
void rw_led(char r, char g, char b);
void rw_display_init(void);
void rw_chargeswitch(char state);
void rw_powerswitch(char state);
uint8_t rw_chargestate(void); // 0 - not charging, 1 - charging, 2 - charged

extern uint32_t adc_vsys;
extern uint32_t adc_usb;
extern uint32_t adc_current;