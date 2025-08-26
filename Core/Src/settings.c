#include "settings.h"
#include "stm32g4xx_hal.h"
#include "main.h" // for OLED_SPI, DISPLAY_CS_* macros
#include <string.h>

// Persist settings in the last flash page. Simpler, single-blob storage.
#define FLASH_START_ADDR   0x08000000u
#define FLASH_TOTAL_SIZE   (112u*1024u)
#define SETTINGS_PAGE_BASE (FLASH_START_ADDR + FLASH_TOTAL_SIZE - FLASH_PAGE_SIZE)

// Minimal fixed blob, doubleword aligned; no CRC/journal.
typedef struct __attribute__((packed)) {
  uint32_t magic;     // 'KSB1' 0x3142534Bu
  uint16_t ver;       // 1
  uint16_t rsvd;
  uint8_t led_brightness_pct; // 0 or 5..100
  uint8_t auto_off;           // kiisu_apoff_t
  uint8_t start_color;        // kiisu_start_color_t
  uint8_t charge_rainbow_on;  // 0/1
  uint32_t pad[5];            // keep 32 bytes and DW alignment
} settings_blob_t;

#define KSB_MAGIC 0x3142534Bu
#define KSB_VER   1u

// Forward decl
static uint8_t clamp_pct_step5(uint8_t v);

static volatile uint8_t s_pending_save = 0;
static uint32_t s_save_deadline_ms = 0;
static uint32_t s_pending_since_ms = 0;
// Track quiet windows for SPI/OLED and Companion CS to avoid interference during erase/program
static uint32_t s_last_spi2_idle_ms = 0;
static uint32_t s_last_comp_cs_high_ms = 0;
// From main.c; set when OLED SPI is in a critical section
extern volatile uint8_t spi_display_critical_section;
extern uint8_t oled_drawing_in_progress;

typedef struct {
  // LED brightness percent: 0=Auto, else 5..100 step 5
  uint8_t led_brightness_pct; // 0 or 5..100
  // Auto power off index
  kiisu_apoff_t auto_off;
  // Startup color
  kiisu_start_color_t start_color;
  // Charge rainbow flag
  uint8_t charge_rainbow_on; // 1=on, 0=off

  // Preview windows
  uint32_t brightness_preview_until_ms; // 0 if none
  uint8_t  brightness_preview_pct; // 5..100

  uint32_t startup_preview_until_ms; // 0 if none
  kiisu_start_color_t startup_preview_color;

  uint8_t rainbow_preview_req; // 0 none, 1 show on, 2 show off
} settings_t;

static settings_t S;

void settings_init(void) {
  // Defaults
  S.led_brightness_pct = 0; // Auto
  S.auto_off = APOFF_15S;
  S.start_color = SC_PURPLE;
  S.charge_rainbow_on = 1;
  S.brightness_preview_until_ms = 0;
  S.startup_preview_until_ms = 0;
  S.rainbow_preview_req = 0;
  s_pending_save = 0;
  s_pending_since_ms = 0;

  // Load blob from flash
  const settings_blob_t* B = (const settings_blob_t*)SETTINGS_PAGE_BASE;
  uint8_t need_save = 0;
  if (B->magic == KSB_MAGIC && B->ver == KSB_VER) {
    // Copy and validate each field, fix invalids to defaults
    uint8_t v;
    v = B->led_brightness_pct; v = clamp_pct_step5(v); S.led_brightness_pct = v;
    v = B->auto_off; if (v > APOFF_60M) { v = APOFF_15S; need_save = 1; } S.auto_off = (kiisu_apoff_t)v;
    v = B->start_color; if (v > SC_WHITE) { v = SC_PURPLE; need_save = 1; } S.start_color = (kiisu_start_color_t)v;
    v = B->charge_rainbow_on; if (v > 1) { v = 1; need_save = 1; } S.charge_rainbow_on = v;
  } else {
    // Not initialized; keep defaults and save soon
    need_save = 1;
  }
  if (need_save) { s_pending_save = 1; s_save_deadline_ms = HAL_GetTick() + 1000; s_pending_since_ms = HAL_GetTick(); }
}

static uint8_t clamp_pct_step5(uint8_t v) {
  if (v == 0) return 0; // auto
  if (v < 5) v = 5;
  if (v > 100) v = 100;
  // round to nearest multiple of 5
  uint8_t rem = v % 5; if (rem) v = (uint8_t)(v - rem + (rem >= 3 ? 5 : 0));
  if (v == 0) v = 5;
  return v;
}

void settings_set_led_brightness_percent(uint8_t pct_or_0_auto, uint32_t now_ms) {
  pct_or_0_auto = clamp_pct_step5(pct_or_0_auto);
  S.led_brightness_pct = pct_or_0_auto;
  if (pct_or_0_auto != 0) {
    S.brightness_preview_pct = pct_or_0_auto;
    S.brightness_preview_until_ms = now_ms + 1200; // 1.2s showcase
  } else {
    S.brightness_preview_until_ms = 0;
  }
  s_pending_save = 1; s_save_deadline_ms = now_ms + 200; if (s_pending_since_ms == 0) s_pending_since_ms = now_ms; // debounce saves
}

void settings_set_auto_poweroff(kiisu_apoff_t v) {
  if (v > APOFF_60M) v = APOFF_60M;
  S.auto_off = v;
  s_pending_save = 1; s_save_deadline_ms = HAL_GetTick() + 200; if (s_pending_since_ms == 0) s_pending_since_ms = HAL_GetTick();
}

void settings_set_startup_color(kiisu_start_color_t v, uint32_t now_ms) {
  if (v > SC_WHITE) v = SC_WHITE;
  S.start_color = v;
  // Trigger a 1s showcase
  S.startup_preview_color = v;
  S.startup_preview_until_ms = now_ms + 1000;
  s_pending_save = 1; s_save_deadline_ms = now_ms + 200; if (s_pending_since_ms == 0) s_pending_since_ms = now_ms;
}

void settings_set_charge_rainbow(uint8_t on, uint32_t now_ms) {
  S.charge_rainbow_on = on ? 1 : 0;
  S.rainbow_preview_req = on ? 1 : 2; // request preview action by main
  (void)now_ms;
  s_pending_save = 1; s_save_deadline_ms = now_ms + 200; if (s_pending_since_ms == 0) s_pending_since_ms = now_ms;
}

uint8_t settings_is_led_brightness_auto(void) { return S.led_brightness_pct == 0; }

uint8_t settings_led_brightness_base255(uint8_t current_backlight) {
  if (S.led_brightness_pct == 0) return current_backlight; // auto
  // Convert percent (5..100) to 0..255
  uint16_t v = (uint16_t)S.led_brightness_pct * 255 / 100;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

static uint32_t idx_to_ms(kiisu_apoff_t v) {
  switch (v) {
    case APOFF_OFF: return 0;
    case APOFF_15S: return 15000;
    case APOFF_30S: return 30000;
    case APOFF_1M:  return 60000;
    case APOFF_5M:  return 300000;
    case APOFF_10M: return 600000;
    case APOFF_30M: return 1800000;
    case APOFF_60M: return 3600000;
    default: return 15000;
  }
}

uint32_t settings_get_auto_poweroff_ms(void) { return idx_to_ms(S.auto_off); }

void settings_get_startup_color_bits(uint8_t* r, uint8_t* g, uint8_t* b) {
  uint8_t R=0,G=0,B=0;
  switch (S.start_color) {
    case SC_PURPLE: R=1;G=0;B=1; break;
    case SC_OFF:    R=0;G=0;B=0; break;
    case SC_RED:    R=1;G=0;B=0; break;
    case SC_GREEN:  R=0;G=1;B=0; break;
    case SC_BLUE:   R=0;G=0;B=1; break;
    case SC_YELLOW: R=1;G=1;B=0; break;
    case SC_CYAN:   R=0;G=1;B=1; break;
    case SC_MAGENTA:R=1;G=0;B=1; break;
    case SC_WHITE:  R=1;G=1;B=1; break;
    default:        R=1;G=0;B=1; break;
  }
  if (r) *r = R; if (g) *g = G; if (b) *b = B;
}

uint8_t settings_get_charge_rainbow_enabled(void) { return S.charge_rainbow_on; }

uint8_t settings_get_brightness_preview(uint32_t now_ms, uint16_t* out_duty) {
  if (S.brightness_preview_until_ms && now_ms < S.brightness_preview_until_ms) {
    // Convert percent to duty 0..2000 (linear)
    uint16_t duty = (uint16_t)((uint32_t)S.brightness_preview_pct * 2000 / 100);
    if (out_duty) *out_duty = duty;
    return 1;
  }
  return 0;
}

uint8_t settings_get_startup_preview(uint32_t now_ms, uint8_t* r, uint8_t* g, uint8_t* b) {
  if (S.startup_preview_until_ms && now_ms < S.startup_preview_until_ms) {
    // Decode preview color bits
    uint8_t R,G,B; kiisu_start_color_t sv = S.startup_preview_color;
    switch (sv) {
      case SC_PURPLE: R=1;G=0;B=1; break;
      case SC_OFF:    R=0;G=0;B=0; break;
      case SC_RED:    R=1;G=0;B=0; break;
      case SC_GREEN:  R=0;G=1;B=0; break;
      case SC_BLUE:   R=0;G=0;B=1; break;
      case SC_YELLOW: R=1;G=1;B=0; break;
      case SC_CYAN:   R=0;G=1;B=1; break;
      case SC_MAGENTA:R=1;G=0;B=1; break;
      case SC_WHITE:  R=1;G=1;B=1; break;
      default:        R=1;G=0;B=1; break;
    }
    if (r) *r = R; if (g) *g = G; if (b) *b = B;
    return 1;
  }
  return 0;
}

uint8_t settings_fetch_rainbow_preview_request(void) {
  uint8_t v = S.rainbow_preview_req;
  S.rainbow_preview_req = 0;
  return v;
}

// Raw getters (for I2C mirror)
uint8_t settings_get_led_brightness_pct_raw(void) { return S.led_brightness_pct; }
kiisu_apoff_t settings_get_auto_poweroff_raw(void) { return S.auto_off; }
kiisu_start_color_t settings_get_startup_color_raw(void) { return S.start_color; }
uint8_t settings_get_charge_rainbow_raw(void) { return S.charge_rainbow_on; }

// Save processing: erase page and write a single blob in main loop context
static int write_blob_to_flash(const settings_blob_t* blob) {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR | FLASH_FLAG_OPTVERR);

  // Always erase page, then write the single blob at base
  FLASH_EraseInitTypeDef erase = {0}; uint32_t err = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = (SETTINGS_PAGE_BASE - FLASH_START_ADDR) / FLASH_PAGE_SIZE;
  erase.NbPages = 1;
  if (HAL_FLASHEx_Erase(&erase, &err) != HAL_OK) { HAL_FLASH_Lock(); return -1; }

  // Program by doublewords
  const uint8_t* b = (const uint8_t*)blob;
  for (uint32_t off = 0; off < sizeof(settings_blob_t); off += 8) {
    uint64_t qw = 0xFFFFFFFFFFFFFFFFULL;
    for (uint8_t i=0;i<8;i++) qw = (qw & ~(0xFFULL << (8*i))) | ((uint64_t)b[off+i] << (8*i));
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_PAGE_BASE + off, qw) != HAL_OK) {
      HAL_FLASH_Lock(); return -2;
    }
  }
  HAL_FLASH_Lock();
  return 0;
}

void settings_process(uint32_t now_ms) {
  (void)now_ms;
  if (!s_pending_save) return;
  if (now_ms && now_ms < s_save_deadline_ms) return; // debounce window
  // Track quiet windows for SPI2 and companion CS
  if (LL_SPI_IsActiveFlag_BSY(OLED_SPI) == 0) { s_last_spi2_idle_ms = now_ms; }
  if (HAL_GPIO_ReadPin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin) == GPIO_PIN_SET) { s_last_comp_cs_high_ms = now_ms; }
  // Require a brief quiet gap before erasing/programming to avoid interference
  const uint32_t QUIET_GAP_MS = 10; // small idle window
  uint8_t quiet_ok = 1;
  if (spi_display_critical_section || oled_drawing_in_progress) quiet_ok = 0;
  if (LL_SPI_IsActiveFlag_BSY(OLED_SPI)) quiet_ok = 0;
  if (HAL_GPIO_ReadPin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin) == GPIO_PIN_RESET) quiet_ok = 0;
  if ((now_ms - s_last_spi2_idle_ms) < QUIET_GAP_MS) quiet_ok = 0;
  if ((now_ms - s_last_comp_cs_high_ms) < QUIET_GAP_MS) quiet_ok = 0;
  // Fallback: if we've been pending for a long time, relax the quiet-gap requirement
  if (s_pending_since_ms && (now_ms - s_pending_since_ms) > 3000) {
    // after 3s, only require not-busy and CS high without the time gap
    quiet_ok = (!spi_display_critical_section && !oled_drawing_in_progress && (LL_SPI_IsActiveFlag_BSY(OLED_SPI) == 0) && (HAL_GPIO_ReadPin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin) == GPIO_PIN_SET));
  }
  if (!quiet_ok) return;
  s_pending_save = 0;
  s_pending_since_ms = 0;

  settings_blob_t blob = {0};
  blob.magic = KSB_MAGIC; blob.ver = KSB_VER; blob.rsvd = 0;
  blob.led_brightness_pct = S.led_brightness_pct;
  blob.auto_off = (uint8_t)S.auto_off;
  blob.start_color = (uint8_t)S.start_color;
  blob.charge_rainbow_on = S.charge_rainbow_on ? 1 : 0;
  (void)write_blob_to_flash(&blob);
}
