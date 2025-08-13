#pragma once
#include "stdint.h"
#include "stdbool.h"

void rw_sh1106_pins_set(bool cs, bool dc, bool rst);
void rw_sh1106_spi_write(uint8_t *buf, uint32_t len);
void rw_sh1106_init();
void rw_sh1106_display(uint8_t buffer);

void rw_sh1106_fill(uint8_t pattern);
void rw_sh1106_setposition(uint8_t column, uint8_t row);

void rw_sh1106_print(char *str);

void rw_display_off(void);
void rw_display_on(void);