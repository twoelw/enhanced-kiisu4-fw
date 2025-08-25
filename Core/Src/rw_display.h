#pragma once
#include <stdint.h>

// High-level display control
void rw_display_init(void);
void rw_display_off(void);
void rw_display_on(void);
void rw_display_set_brightness(uint8_t brightness);

// Drawing protection API
void rw_display_drawing_start(void);
void rw_display_drawing_end(void);
void rw_display_spi_critical_start(void);
void rw_display_spi_critical_end(void);

// Shared arbitration flag (defined in main.c)
extern volatile uint8_t spi_display_critical_section;
