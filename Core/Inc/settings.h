#pragma once
#include <stdint.h>

// Settings protocol values (shared with I2C register interface in rw_i2c_emu.c)

// Auto power-off options while in sleep/idle
// Index mapping per spec: 0=Off, 1=15s, 2=30s, 3=1min, 4=5min, 5=10min, 6=30min, 7=60min
typedef enum {
  APOFF_OFF = 0,
  APOFF_15S = 1,
  APOFF_30S = 2,
  APOFF_1M  = 3,
  APOFF_5M  = 4,
  APOFF_10M = 5,
  APOFF_30M = 6,
  APOFF_60M = 7,
} kiisu_apoff_t;

// Startup LED color options
// 0=Purple (default), 1=Off, 2=Red, 3=Green, 4=Blue, 5=Yellow, 6=Cyan, 7=Magenta, 8=White
typedef enum {
  SC_PURPLE = 0,
  SC_OFF    = 1,
  SC_RED    = 2,
  SC_GREEN  = 3,
  SC_BLUE   = 4,
  SC_YELLOW = 5,
  SC_CYAN   = 6,
  SC_MAGENTA= 7,
  SC_WHITE  = 8,
} kiisu_start_color_t;

// Initialize defaults
void settings_init(void);

// Setters (invoked by I2C emu on register writes)
void settings_set_led_brightness_percent(uint8_t pct_or_0_auto, uint32_t now_ms);
void settings_set_auto_poweroff(kiisu_apoff_t v);
void settings_set_startup_color(kiisu_start_color_t v, uint32_t now_ms);
void settings_set_charge_rainbow(uint8_t on, uint32_t now_ms);

// Getters for application logic
uint8_t settings_is_led_brightness_auto(void);
// Return 0..255 backlight-equivalent for LED PWM; if auto, returns provided backlight
uint8_t settings_led_brightness_base255(uint8_t current_backlight);
// Map APOFF index to milliseconds (0 means disabled)
uint32_t settings_get_auto_poweroff_ms(void);
// Decode startup color to R,G,B enable flags (active-low LED pins are handled elsewhere)
void settings_get_startup_color_bits(uint8_t* r, uint8_t* g, uint8_t* b);
uint8_t settings_get_charge_rainbow_enabled(void);

// Preview helpers (one-shot windows set by setters). The main loop may use these
// to temporarily drive LEDs when otherwise off. These do NOT perform IO.
// Returns 1 and outputs duty 0..2000 when a brightness preview window is active.
uint8_t settings_get_brightness_preview(uint32_t now_ms, uint16_t* out_duty_0_2000);
// Returns 1 and outputs color bits when a startup color preview window is active.
uint8_t settings_get_startup_preview(uint32_t now_ms, uint8_t* r, uint8_t* g, uint8_t* b);
// Returns 1 if a rainbow-on preview should be started; 2 if rainbow-off showcase is requested; 0 otherwise.
// This is an edge-triggered fetch; it clears the pending request.
uint8_t settings_fetch_rainbow_preview_request(void);

// Raw getters for mirroring current values into I2C register snapshots (optional)
uint8_t settings_get_led_brightness_pct_raw(void);
kiisu_apoff_t settings_get_auto_poweroff_raw(void);
kiisu_start_color_t settings_get_startup_color_raw(void);
uint8_t settings_get_charge_rainbow_raw(void);

// Process any pending save to flash; call from main loop
void settings_process(uint32_t now_ms);
