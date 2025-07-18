#pragma once
#include "stdbool.h"
#include "stdint.h"
uint8_t rw_i2c_get_reg(uint8_t address, uint8_t reg);
void rw_i2c_reg_written(uint8_t address, uint8_t reg, uint8_t value);
void rw_i2c_emu_init();
uint8_t rw_i2c_get_backlight();
