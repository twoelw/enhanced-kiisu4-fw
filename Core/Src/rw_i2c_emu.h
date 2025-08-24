#pragma once
#include "stdbool.h"
#include "stdint.h"
uint8_t rw_i2c_get_reg(uint8_t address, uint8_t reg);
void rw_i2c_reg_written(uint8_t address, uint8_t reg, uint8_t value);
void rw_i2c_emu_init();
uint8_t rw_i2c_get_backlight();
void rw_i2c_set_battery(int16_t vbatt, int16_t vusb, int16_t current,
                        uint8_t charge_state);

// Bootloader control/status on existing addresses (preferred: 0x6B; mirrored on 0x30)
// Registers (under 0x6B and 0x30):
//  F0: CAPS bit0=1 -> supports boot-to-system-memory
//  F1: CMD  write 0xA5 -> request boot
//  F2: STATUS 0=idle, 1=queued, 2=waiting-safe, 3=jumping, 0xFF=denied/error
//  F3..F5: Signature 'B','L', version byte
void rw_i2c_set_boot_status(uint8_t status);

// Firmware update protocol (I2C addr 0x30)
// Host workflow:
//  - Write 0xA0 to 0xF1 (CMD) to enter update READY; read 0xF2 STATUS=0x10
//  - For each chunk (<=32 bytes, 8-byte aligned offset and length):
//    - Write 0xE0..0xE3 CHUNK_OFFSET (u32), 0xE4 CHUNK_LEN (<=32), 0xE6..0xE7 CHUNK_CRC16
//    - Write data to 0xC0..(0xC0+len-1)
//    - Write 0xB1 to 0xF1 (COMMIT_CHUNK); poll 0xF2 for 0x10; on error 0x40 check 0xF3 and retry
//  - Write total length to 0xF8..0xFB optionally; track 0xFC..0xFF RECEIVED_LEN and 0xF4..0xF7 rolling CRC32
//  - Write 0xA2 to 0xF1 to finalize; device will program vector page and reset on success
// Registers under 0x30:
//  F0 PROTO_VER=1; F1 CMD; F2 STATUS; F3 ERROR; F4..F7 CRC32; F8..FB TOTAL_LEN; FC..FF RECEIVED_LEN
//  E0..E3 CHUNK_OFFSET; E4 CHUNK_LEN; E6..E7 CHUNK_CRC16; C0..DF DATA window (32 bytes)

// Called from main loop to process any pending update work (program chunks, finalize)
void rw_update_process(void);
// Main can check if update flow asked to quiet peripherals
uint8_t rw_update_quiet_requested(void);
void rw_update_clear_quiet_request(void);
