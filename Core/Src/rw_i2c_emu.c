
#include "rw_i2c_emu.h"
#include "stm32g4xx_hal.h"
#include <string.h>

// External variables from main.c
extern uint32_t adc_usb; // USB voltage reading
// Ask main to handle bootloader jump safely in thread context
extern void queue_bootloader_request(void);

uint8_t i2c_55[256];
uint8_t i2c_6b[256];
uint8_t i2c_30[256];

// ===== Firmware update state (addr 0x30) =====
// Register map constants
#define REG_PROTO_VER      0xF0
#define REG_CMD            0xF1
#define REG_STATUS         0xF2
#define REG_ERROR          0xF3
#define REG_ROLL_CRC32_0   0xF4
#define REG_ROLL_CRC32_3   0xF7
#define REG_TOTAL_LEN_0    0xF8
#define REG_TOTAL_LEN_3    0xFB
#define REG_RX_LEN_0       0xFC
#define REG_RX_LEN_3       0xFF
// Host can write REG_GO (0xF9) = 0x01 to request immediate jump to new app
#define REG_GO             0xF9
#define REG_CHUNK_OFF_0    0xE0
#define REG_CHUNK_OFF_3    0xE3
#define REG_CHUNK_LEN      0xE4
#define REG_CHUNK_CRC16_0  0xE6
#define REG_DATA_START     0xC0
#define REG_DATA_END       0xDF // inclusive, 32 bytes window
// Optional debug exposure: computed CRC16 and echoed CRC16 written by host
#define REG_DBG_CRC16_CALC_0 0xE8
#define REG_DBG_CRC16_CALC_1 0xE9
#define REG_DBG_CRC16_HOST_0 0xEA
#define REG_DBG_CRC16_HOST_1 0xEB
// Debug: expose staged vector reset the aux sees during FINALIZE
#define REG_DBG_VECT_RESET_0  0xEC
// Event/flags exposed to host (bit0 = DONE)
#define REG_EVENT_FLAGS      0xED
#define EVT_DONE_MASK        0x01

// Commands
#define CMD_ENTER_READY    0xA0
#define CMD_COMMIT_CHUNK   0xB1
#define CMD_FINALIZE       0xA2
// Optional: host-requested shutdown
#define CMD_SHUTDOWN       0xA3

// Status codes
#define ST_IDLE            0x00
#define ST_READY           0x10
#define ST_BUSY            0x11
#define ST_ERROR           0x40

// Error codes
#define ERR_NONE           0x00
#define ERR_BAD_PARAM      0x01
#define ERR_CRC_MISMATCH   0x02
#define ERR_FLASH_ERASE    0x03
#define ERR_FLASH_PROG     0x04

// Flash layout
#define FLASH_START_ADDR   0x08000000UL
#define FLASH_END_ADDR     (FLASH_START_ADDR + 112*1024UL)
#define PAGE_SIZE          2048UL // STM32G431 page 2KB
// New application staging base (keep current app intact until jump)
// Use 0x08010000 (64KB offset) to avoid overlapping the current app (~36KB now).
// Note: The staged image must be built/linked for this base to be directly runnable.
#define NEW_APP_BASE       (FLASH_START_ADDR + 0x10000UL) // 64KB offset

static uint8_t update_ready = 0;
static volatile uint8_t update_quiet_request = 0;
static uint32_t rolling_crc32 = 0;
static uint32_t total_len_expect = 0;
static uint32_t received_len = 0;
static uint8_t flash_region_erased = 0;

// RAM-resident routines to erase [dst..dst+len) and program from src.
// Must reside in RAM to be able to erase/program flash safely.
static void __attribute__((section(".ramfunc"), noinline)) ram_copy_down(uint32_t src, uint32_t dst, uint32_t len)
{
  __disable_irq();
  // Unlock if locked
  if (FLASH->CR & FLASH_CR_LOCK) {
    FLASH->KEYR = 0x45670123U;
    FLASH->KEYR = 0xCDEF89ABU;
  }
  // Clear sticky error flags
  FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR |
               FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR |
               FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR);

  // Erase necessary pages
  for (uint32_t a = dst; a < (dst + len); a += PAGE_SIZE) {
    uint32_t page = (a - FLASH_START_ADDR) / PAGE_SIZE;
    // Set page erase with page number
    FLASH->CR &= ~FLASH_CR_PNB_Msk;
    FLASH->CR |= FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY) { /* wait */ }
    FLASH->CR &= ~FLASH_CR_PER;
  }

  // Program doublewords
  FLASH->CR |= FLASH_CR_PG;
  for (uint32_t off = 0; off < len; off += 8) {
    uint8_t* s = (uint8_t*)(src + off);
    uint8_t* d = (uint8_t*)(dst + off);
    uint64_t val = 0xFFFFFFFFFFFFFFFFULL;
    for (uint8_t b = 0; b < 8; ++b) {
      uint8_t vb = 0xFF;
      if (off + b < len) vb = s[b];
      val &= ~((uint64_t)0xFF << (8*b));
      val |= ((uint64_t)vb) << (8*b);
    }
    *((volatile uint32_t*)d) = (uint32_t)(val & 0xFFFFFFFFu);
    *((volatile uint32_t*)(d + 4)) = (uint32_t)(val >> 32);
    while (FLASH->SR & FLASH_SR_BSY) { /* wait */ }
  }
  FLASH->CR &= ~FLASH_CR_PG;
  __enable_irq();
}

// CRC16/X25: poly 0x1021 (reflected 0x8408), init 0xFFFF, refin=true, refout=true, xorout=0xFFFF
static uint16_t crc16_x25(const uint8_t* data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0x8408; else crc >>= 1;
    }
  }
  return (uint16_t)~crc;
}

// CRC32 (IEEE) for rolling
static uint32_t crc32_step(uint32_t crc, const uint8_t* data, uint32_t len)
{
  crc = ~crc;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b=0;b<8;b++)
      crc = (crc >> 1) ^ (0xEDB88320U & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

static inline uint32_t u32_from_regs(uint8_t* base, uint8_t idx0)
{
  return (uint32_t)base[idx0] | ((uint32_t)base[idx0+1] << 8) | ((uint32_t)base[idx0+2] << 16) | ((uint32_t)base[idx0+3] << 24);
}
static inline void u32_to_regs(uint8_t* base, uint8_t idx0, uint32_t v)
{
  base[idx0] = (uint8_t)(v & 0xFF);
  base[idx0+1] = (uint8_t)((v >> 8) & 0xFF);
  base[idx0+2] = (uint8_t)((v >> 16) & 0xFF);
  base[idx0+3] = (uint8_t)((v >> 24) & 0xFF);
}

/*
 write to 0x55 ack data: 0x3E 0x9B 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0x31

write to 0x55 ack data: 0x3E 0x9D 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x08 0x34
write to 0x55 ack data: 0x3E 0x9F 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x08 0x34
write to 0x55 ack data: 0x3E 0xA3 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0E 0x5F
write to 0x55 ack data: 0x3E 0xA9 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x01 0xAE
write to 0x55 ack data: 0x3E 0xAB 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x01 0x4E
write to 0x55 ack data: 0x3E 0xAD 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x12 0x12
write to 0x55 ack data: 0x3E 0xAF 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x01 0x98
write to 0x55 ack data: 0x3E 0xB1 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0B
write to 0x55 ack data: 0x3E 0xB2 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x00
write to 0x55 ack data: 0x3E 0xBD 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0F 0xCC
write to 0x55 ack data: 0x3E 0xBF 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0F 0x41
write to 0x55 ack data: 0x3E 0xC1 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0E 0xDF
write to 0x55 ack data: 0x3E 0xC3 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0E 0x86
write to 0x55 ack data: 0x3E 0xC5 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0E 0x3A
write to 0x55 ack data: 0x3E 0xC7 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0E 0x01
write to 0x55 ack data: 0x3E 0xC9 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0xDA
write to 0x55 ack data: 0x3E 0xCB 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0xBA
write to 0x55 ack data: 0x3E 0xCD 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0x95
write to 0x55 ack data: 0x3E 0xCF 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0x53
write to 0x55 ack data: 0x3E 0xD1 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0C 0xE3
write to 0x55 ack data: 0x3E 0xB4 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0C 0xE4
write to 0x55 ack data: 0x3E 0xB7 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0C 0xF9
write to 0x55 ack data: 0x3E 0xBA 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x0D 0x1B
write to 0x55 ack data: 0x3E 0xDE 0x91
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x01

write to 0x55 ack data: 0x3E 0x17 0x92
write to 0x55 ack data: 0x40
read to 0x55 ack data: 0x00 0x01

write to 0x55 ack data: 0x3A
read to 0x55 ack data: 0x24 0x00
//sealed
write to 0x55 ack data: 0x00 0x30 0x00
write to 0x55 ack data: 0x3A
read to 0x55 ack data: 0x26 0x00
*/

// ..\Core\Src\rw_i2c_emu.c
void addr_55_written(uint8_t reg, uint8_t value)
{
  if (reg == 0x00)
  {
    switch (value)
    {
      case 0x30:
        i2c_55[0x3A] = 0b01101110;
        break;
      case 0x01:
        i2c_55[0x40] = 0x20;
        i2c_55[0x41] = 0x02;
        break;
    }
  }

  if (reg == 0x3E)
  {
    switch (value)
    {
      case 0x9B:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0x31;
        break;
      case 0x9D:
        i2c_55[0x40] = 0x08;
        i2c_55[0x41] = 0x34;
        break;
      case 0x9F:
        i2c_55[0x40] = 0x08;
        i2c_55[0x41] = 0x34;
        break;
      case 0xA3:
        i2c_55[0x40] = 0x0E;
        i2c_55[0x41] = 0x5F;
        break;
      case 0xA9:
        i2c_55[0x40] = 0x01;
        i2c_55[0x41] = 0xAE;
        break;
      case 0xAB:
        i2c_55[0x40] = 0x01;
        i2c_55[0x41] = 0x4E;
        break;
      case 0xAD:
        i2c_55[0x40] = 0x12;
        i2c_55[0x41] = 0x12;
        break;
      case 0xAF:
        i2c_55[0x40] = 0x01;
        i2c_55[0x41] = 0x98;
        break;
      case 0xB1:
        i2c_55[0x40] = 0x0B;
        i2c_55[0x41] = 0x00;
        break;
      case 0xB2:
        i2c_55[0x40] = 0x00;
        i2c_55[0x41] = 0x00;
        break;
      case 0xBD:
        i2c_55[0x40] = 0x0F;
        i2c_55[0x41] = 0xCC;
        break;
      case 0xBF:
        i2c_55[0x40] = 0x0F;
        i2c_55[0x41] = 0x41;
        break;
      case 0xC1:
        i2c_55[0x40] = 0x0E;
        i2c_55[0x41] = 0xDF;
        break;
      case 0xC3:
        i2c_55[0x40] = 0x0E;
        i2c_55[0x41] = 0x86;
        break;
      case 0xC5:
        i2c_55[0x40] = 0x0E;
        i2c_55[0x41] = 0x3A;
        break;
      case 0xC7:
        i2c_55[0x40] = 0x0E;
        i2c_55[0x41] = 0x01;
        break;
      case 0xC9:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0xDA;
        break;
      case 0xCB:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0xBA;
        break;
      case 0xCD:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0x95;
        break;
      case 0xCF:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0x53;
        break;
      case 0xD1:
        i2c_55[0x40] = 0x0C;
        i2c_55[0x41] = 0xE3;
        break;
      case 0xB4:
        i2c_55[0x40] = 0x0C;
        i2c_55[0x41] = 0xE4;
        break;
      case 0xB7:
        i2c_55[0x40] = 0x0C;
        i2c_55[0x41] = 0xF9;
        break;
      case 0xBA:
        i2c_55[0x40] = 0x0D;
        i2c_55[0x41] = 0x1B;
        break;
      case 0xDE:
        i2c_55[0x40] = 0x01;
        i2c_55[0x41] = 0x00;
        break;
      case 0x17:
        i2c_55[0x40] = 0x00;
        i2c_55[0x41] = 0x01;
        break;
      default:
        break;
    }
  }
}

void int_to_array(uint8_t arr[], uint8_t index, int16_t value) 
{
  arr[index+1] = (value >> 8) & 0xFF; // High byte
  arr[index] = value & 0xFF;    // Low byte
}

void rw_i2c_emu_init()
{
  int_to_array(i2c_55, 0x06, (273+20)*10);//temperature
  int_to_array(i2c_55, 0x08, 3400);       // voltage
  int_to_array(i2c_55, 0x0C, 20);     // current
  int_to_array(i2c_55, 0x2C, 80);      // charge percentage
  int_to_array(i2c_55, 0x2E, 100);       // health
  i2c_55[0x0A] = 0b00111011;           // 1 bit - set when discarging
  i2c_55[0x0B] = 0b01000000;           // 2 bit - full charge
  i2c_55[0x3A] = 0b01101100;

  i2c_6b[0x0B] = 0b00000010; // not charging
  i2c_6b[0x0E] = 100;        // batv
  i2c_6b[0x0F] = 100;        // sysv
  i2c_6b[0x11] = 4000; // vbus
  
  //i2c_6b[0x0B] = 0b00110110; // charging
  //i2c_6b[0x0B] = 0b00111110; // charged

  // Initialize control page for boot capability on both 0x6B and 0x30 (mirror)
  // Capability: bit0 -> supports system memory boot
  i2c_6b[0xF0] = 0x01;
  i2c_30[0xF0] = 0x01;
  // Status: 0 = idle
  i2c_6b[0xF2] = 0x00;
  i2c_30[0xF2] = 0x00;
  // Signature 'B','L', version 0x01
  i2c_6b[0xF3] = 'B'; i2c_30[0xF3] = 'B';
  i2c_6b[0xF4] = 'L'; i2c_30[0xF4] = 'L';
  i2c_6b[0xF5] = 0x01; i2c_30[0xF5] = 0x01;

  // Firmware update defaults on 0x30
  i2c_30[REG_PROTO_VER] = 0x01;
  i2c_30[REG_STATUS] = ST_IDLE;
  i2c_30[REG_ERROR] = ERR_NONE;
  u32_to_regs(i2c_30, REG_ROLL_CRC32_0, 0);
  u32_to_regs(i2c_30, REG_TOTAL_LEN_0, 0);
  u32_to_regs(i2c_30, REG_RX_LEN_0, 0);
  i2c_30[REG_EVENT_FLAGS] = 0;
}

void rw_i2c_set_battery(int16_t vbatt,int16_t vusb,int16_t current, uint8_t charge_state) 
{
  if (charge_state == 0)
  {
    i2c_55[0x0A] = 0b00111011; // 1 bit - set when discarging
    i2c_6b[0x0B] = 0b00000010; // not charging
  }
  else if (charge_state == 1)
  {
    i2c_55[0x0A] = 0b00111010; 
    i2c_6b[0x0B] = 0b00110110; // charging
  }
  else if (charge_state == 2)
  {
    i2c_55[0x0A] = 0b00111110; // 
    i2c_6b[0x0B] = 0b00111110; // charged
  }
  int_to_array(i2c_55, 0x08, vbatt); // voltage
  int_to_array(i2c_55, 0x0C, current);   // current
  int_to_array(i2c_55, 0x2C, (vbatt-3200)*100/(4200-3200));   // charge percentage 
}

void rw_i2c_reg_written(uint8_t address, uint8_t reg, uint8_t value)
{
  if (address == 0x6b)
  {
    i2c_6b[reg] = value;
    
    // Check for shutdown signal (similar to BQ25896 BATFET_DIS)
    // Register 0x09 with bit 5 set indicates battery disconnect/shutdown request
    if (reg == 0x09 && (value & 0x20)) { // Check bit 5 (BATFET_DIS equivalent)
      // Queue shutdown to be handled in main loop context (avoid heavy work in ISR)
      extern void queue_shutdown_request(void);
      queue_shutdown_request();
    }
    // Boot command handled at 0x6B:F1 too
    if (reg == 0xF1) {
      if (value == 0xA5) {
        i2c_6b[0xF2] = 0x01; // queued
        i2c_30[0xF2] = 0x01; // mirror
        queue_bootloader_request();
      } else if (value == 0x00) {
        i2c_6b[0xF2] = 0x00; i2c_30[0xF2] = 0x00; // clear to idle
      }
    }
  }
  else if (address == 0x30)
  {
    i2c_30[reg] = value;
    // Firmware update protocol handling under 0x30
    if (reg == REG_CMD) {
      switch (value) {
        case CMD_ENTER_READY:
          update_ready = 1;
          update_quiet_request = 1; // ask main to quiet non-essential activity
          i2c_30[REG_STATUS] = ST_READY;
          i2c_30[REG_ERROR] = ERR_NONE;
          rolling_crc32 = 0; received_len = 0;
          flash_region_erased = 0;
          u32_to_regs(i2c_30, REG_ROLL_CRC32_0, rolling_crc32);
          u32_to_regs(i2c_30, REG_RX_LEN_0, received_len);
          break;
        case CMD_COMMIT_CHUNK: {
          if (!update_ready) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; break; }
          uint32_t off = u32_from_regs(i2c_30, REG_CHUNK_OFF_0);
          uint8_t len = i2c_30[REG_CHUNK_LEN];
          uint16_t crc_in = (uint16_t)i2c_30[REG_CHUNK_CRC16_0] | ((uint16_t)i2c_30[REG_CHUNK_CRC16_0+1] << 8);
          if (len == 0 || len > 32 || (off % 4) != 0) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; break; }
          // Verify CRC16
          uint16_t crc_calc = crc16_x25(&i2c_30[REG_DATA_START], len);
          // Store debug CRCs for host inspection on mismatch
          i2c_30[REG_DBG_CRC16_CALC_0] = (uint8_t)(crc_calc & 0xFF);
          i2c_30[REG_DBG_CRC16_CALC_1] = (uint8_t)((crc_calc >> 8) & 0xFF);
          i2c_30[REG_DBG_CRC16_HOST_0] = (uint8_t)(crc_in & 0xFF);
          i2c_30[REG_DBG_CRC16_HOST_1] = (uint8_t)((crc_in >> 8) & 0xFF);
          if (crc_calc != crc_in) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_CRC_MISMATCH; break; }
          // From this point, the operation is accepted; report BUSY to the host immediately
          // so it doesn't see READY while we perform a potentially long flash erase.
          i2c_30[REG_STATUS] = ST_BUSY;
          i2c_30[REG_ERROR] = ERR_NONE;

          // Do not erase/program flash in ISR; rw_update_process() will handle
          // erase on first chunk and programming in main loop context.
          // Stage accepted; rw_update_process() will program the chunk and then set READY
          // Nothing else here; actual flash happens in rw_update_process
          break; }
        case CMD_FINALIZE:
          if (!update_ready) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; }
          else { i2c_30[REG_STATUS] = ST_BUSY; }
          break;
        case CMD_SHUTDOWN:
          // Host may ask to power off after update completes
          {
            extern void queue_shutdown_request(void);
            queue_shutdown_request();
            i2c_30[REG_STATUS] = ST_READY; i2c_30[REG_ERROR] = ERR_NONE;
          }
          break;
        default:
          break;
      }
    }
    // Allow host to set expected total length
    if (reg >= REG_TOTAL_LEN_0 && reg <= REG_TOTAL_LEN_3) {
      total_len_expect = u32_from_regs(i2c_30, REG_TOTAL_LEN_0);
    }
    // Host request to immediately jump to new application after finalize
    if (reg == REG_GO) {
      if (i2c_30[REG_GO] == 0x01) {
        // If finalize already processed and received_len matches expected, request jump
        if (received_len && total_len_expect && received_len == total_len_expect) {
          extern void request_jump_to_application(uint32_t app_base);
          request_jump_to_application(NEW_APP_BASE);
          i2c_30[REG_STATUS] = ST_READY;
          i2c_30[REG_ERROR] = ERR_NONE;
        }
      }
    }
  }
  else if (address == 0x55)
  {
    //i2c_55[reg] = value
    addr_55_written(reg,value);
  }
}

uint8_t rw_i2c_get_reg(uint8_t address, uint8_t reg)
{
  if (address == 0x6b)
  {
  return i2c_6b[reg];
  }
  else if (address == 0x30)
  {
  return i2c_30[reg];
  }
  else if (address == 0x55)
  {
	return i2c_55[reg];
  }
  return 0;
}

uint8_t rw_i2c_get_backlight()
{
 return i2c_30[14];
}

void rw_i2c_set_boot_status(uint8_t status)
{
  i2c_6b[0xF2] = status;
  i2c_30[0xF2] = status; // mirror
}

// Exposed for main: process pending update tasks
void rw_update_process(void)
{
  // Only act if in READY/BUSY state
  uint8_t st = i2c_30[REG_STATUS];
  if (!update_ready || (st != ST_BUSY && st != ST_READY)) {
    return;
  }

  // If BUSY due to COMMIT_CHUNK, program chunk now
  if (st == ST_BUSY && i2c_30[REG_CMD] == CMD_COMMIT_CHUNK) {
    uint32_t off = u32_from_regs(i2c_30, REG_CHUNK_OFF_0);
    uint8_t len = i2c_30[REG_CHUNK_LEN];
    if (len == 0 || len > 32 || (off % 4)) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return; }
    uint32_t dst = NEW_APP_BASE + off;
    if (dst < FLASH_START_ADDR || dst + len > FLASH_END_ADDR) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return; }

    // Perform one-time erase of the destination region before first program
    if (!flash_region_erased) {
      if (total_len_expect == 0) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return; }
      uint32_t last_addr = NEW_APP_BASE + total_len_expect - 1;
      if (last_addr >= FLASH_END_ADDR) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return; }

      HAL_FLASH_Unlock();
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR | FLASH_FLAG_OPTVERR);
      uint32_t first_page = (NEW_APP_BASE - FLASH_START_ADDR) / PAGE_SIZE;
      uint32_t last_page = (last_addr - FLASH_START_ADDR) / PAGE_SIZE;
      FLASH_EraseInitTypeDef erase = {0};
      erase.TypeErase = FLASH_TYPEERASE_PAGES;
      erase.Page = first_page;
      erase.NbPages = (last_page - first_page + 1);
      uint32_t err_page = 0;
      if (HAL_FLASHEx_Erase(&erase, &err_page) != HAL_OK) { i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_FLASH_ERASE; HAL_FLASH_Lock(); return; }
      HAL_FLASH_Lock();
      flash_region_erased = 1;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR | FLASH_FLAG_OPTVERR);

    // Program 64-bit doublewords; pad to 8 bytes
    uint8_t buf[40];
    for (uint8_t i=0;i<len;i++) buf[i] = i2c_30[REG_DATA_START+i];
    while (len % 8) { buf[len++] = 0xFF; }
    for (uint8_t i=0; i<len; i+=8) {
      uint64_t qw = 0;
      for (uint8_t b=0;b<8;b++) { qw |= ((uint64_t)buf[i+b]) << (8*b); }
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst + i, qw) != HAL_OK) {
        i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_FLASH_PROG; HAL_FLASH_Lock(); return; }
    }
    HAL_FLASH_Lock();

    // Update rolling CRC32 and received length
    uint8_t orig_len = i2c_30[REG_CHUNK_LEN];
    rolling_crc32 = crc32_step(rolling_crc32, &i2c_30[REG_DATA_START], orig_len);
    received_len += orig_len;
    u32_to_regs(i2c_30, REG_ROLL_CRC32_0, rolling_crc32);
    u32_to_regs(i2c_30, REG_RX_LEN_0, received_len);

    // Done this chunk
    i2c_30[REG_STATUS] = ST_READY;
    i2c_30[REG_ERROR] = ERR_NONE;
  }

  // Finalize: if image is linked for staged base, program vector to jump to stage.
  // If image is linked for 0x08000000, copy staged image down to 0x08000000 safely.
  if (st == ST_BUSY && i2c_30[REG_CMD] == CMD_FINALIZE) {
    // Require complete payload
    if (total_len_expect == 0 || received_len != total_len_expect) {
      i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return;
    }

    uint32_t staged_sp = *(uint32_t*)(NEW_APP_BASE + 0);
    uint32_t staged_reset = *(uint32_t*)(NEW_APP_BASE + 4);
    u32_to_regs(i2c_30, REG_DBG_VECT_RESET_0, staged_reset);

    // Sanity for SP
    if ((staged_sp & 0xFF000000u) != 0x20000000u) {
      i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return;
    }

    // Decide path
    uint8_t is_stage_linked = (staged_reset >= NEW_APP_BASE) && (staged_reset < NEW_APP_BASE + total_len_expect);
    uint8_t is_base_linked  = (staged_reset >= FLASH_START_ADDR) && (staged_reset < FLASH_START_ADDR + total_len_expect);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_PGSERR | FLASH_FLAG_MISERR | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR | FLASH_FLAG_OPTVERR);

    if (is_stage_linked) {
      // Signal DONE to host and wait a short window for it to react (beep/ack)
      i2c_30[REG_EVENT_FLAGS] |= EVT_DONE_MASK;
      // Program only primary vector to point to staged image (preserve image in place)
      // Program only primary vector to point to staged image (preserve image in place)
      // Erase first page then program [SP,Reset]
      FLASH_EraseInitTypeDef erase0 = {0};
      erase0.TypeErase = FLASH_TYPEERASE_PAGES; erase0.Page = 0; erase0.NbPages = 1; uint32_t err_page = 0;
      if (HAL_FLASHEx_Erase(&erase0, &err_page) != HAL_OK) {
        HAL_FLASH_Lock(); i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_FLASH_ERASE; return; }
      uint64_t qw = ((uint64_t)staged_reset << 32) | (uint64_t)staged_sp;
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_START_ADDR, qw) != HAL_OK) {
        HAL_FLASH_Lock(); i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_FLASH_PROG; return; }
      HAL_FLASH_Lock();

      // Verify
      if (*(uint32_t*)FLASH_START_ADDR != staged_sp || *(uint32_t*)(FLASH_START_ADDR+4) != staged_reset) {
        i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_FLASH_PROG; return; }

      // Do not jump/reset; schedule auto-shutdown after 2s so next boot runs new app
      i2c_30[REG_STATUS] = ST_READY; i2c_30[REG_ERROR] = ERR_NONE; update_quiet_request = 0;
      extern void queue_shutdown_after(uint32_t delay_ms);
      queue_shutdown_after(2000);
      return;
    }

    if (is_base_linked) {
  // Let host know finalize succeeded
  i2c_30[REG_EVENT_FLAGS] |= EVT_DONE_MASK;
  // Ensure copy routine is present in RAM (.ramfunc) by copying the load image if needed
      extern uint8_t __ramfunc_start__, __ramfunc_end__, __ramfunc_load_start__;
      uint32_t ramfunc_size = (uint32_t)(&__ramfunc_end__) - (uint32_t)(&__ramfunc_start__);
      if (ramfunc_size) {
        memcpy(&__ramfunc_start__, &__ramfunc_load_start__, ramfunc_size);
      }
  // Execute copy-down entirely from RAM and return; we'll power off instead of jumping
  ram_copy_down(NEW_APP_BASE, FLASH_START_ADDR, total_len_expect);
  // Mark status as READY so the host can read it
  i2c_30[REG_STATUS] = ST_READY; i2c_30[REG_ERROR] = ERR_NONE; update_quiet_request = 0;
  // Schedule auto-shutdown in 2 seconds; upon next boot, base-linked app runs from 0x08000000
  extern void queue_shutdown_after(uint32_t delay_ms);
  queue_shutdown_after(2000);
  return;
    }

    // If neither base nor stage range, treat as bad param
    HAL_FLASH_Lock(); i2c_30[REG_STATUS] = ST_ERROR; i2c_30[REG_ERROR] = ERR_BAD_PARAM; return;
  }
}

uint8_t rw_update_quiet_requested(void) { return update_quiet_request ? 1 : 0; }
void rw_update_clear_quiet_request(void) { update_quiet_request = 0; }