#pragma once
#include "stdint.h"
#include "rw_display.h"

void rw_read_adc(void);
void rw_led(char r, char g, char b);
void rw_display_init(void);
void rw_chargeswitch(char state);
void rw_powerswitch(char state);

extern uint32_t adc3;
extern uint32_t adc_usb_raw;
extern uint32_t adc15;