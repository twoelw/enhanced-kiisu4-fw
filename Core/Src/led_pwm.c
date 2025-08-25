#include <stdint.h>
#include "main.h"

// Static state for non-blocking software PWM (20 ms period)
static uint8_t s_color_r = 0;
static uint8_t s_color_g = 0;
static uint8_t s_color_b = 0;
static uint32_t s_mask_c = 0; // combined mask for GPIOC pins
static uint32_t s_mask_active = 0; // what pins are currently driven ON (low) when s_is_on=1
#define LED_ALL_MASK (LED_R_Pin | LED_G_Pin | LED_B_Pin)
static uint16_t s_duty_0_2000 = 0; // 0..2000
static uint8_t s_use_rgb = 0; // 1 when per-channel mode active
static uint16_t s_r_0_2000 = 0, s_g_0_2000 = 0, s_b_0_2000 = 0;
static int32_t s_sd_r = 0, s_sd_g = 0, s_sd_b = 0; // accumulators for rgb
static uint32_t s_last_toggle_ms = 0;
static uint8_t s_is_on = 0;
static uint8_t s_enabled = 0;
// 1 kHz sigma-delta (first-order) using 1 ms ticks.
static uint8_t s_use_isr = 0; // 0 = main loop 1kHz, 1 = ISR 2kHz
static uint32_t s_last_decide_ms = 0;
static int32_t s_sd_err = 0; // error accumulator in 0..2000 units
static int32_t s_sd_err_isr = 0;
static uint8_t s_isr_div = 0; // divide 10kHz TIM6 to ~2kHz decisions

 

void led_pwm_init(void) {
    s_color_r = s_color_g = s_color_b = 0;
    s_duty_0_2000 = 0;
    s_is_on = 0;
    s_last_toggle_ms = 0;
    s_enabled = 0;
    s_last_decide_ms = 0;
    s_sd_err = 0;
    s_sd_err_isr = 0;
    s_use_isr = 0;
    s_mask_c = 0;
    s_mask_active = 0;
    s_isr_div = 0;
    s_use_rgb = 0;
    s_r_0_2000 = s_g_0_2000 = s_b_0_2000 = 0;
    s_sd_r = s_sd_g = s_sd_b = 0;
}

void led_pwm_set_color(uint8_t r, uint8_t g, uint8_t b) {
    s_color_r = r ? 1 : 0;
    s_color_g = g ? 1 : 0;
    s_color_b = b ? 1 : 0;
    // Keep enabled state unchanged; color only affects what turns on during ON window
    s_mask_c = 0;
    if (s_color_r) s_mask_c |= LED_R_Pin;
    if (s_color_g) s_mask_c |= LED_G_Pin;
    if (s_color_b) s_mask_c |= LED_B_Pin;
    // If currently ON, immediately reflect new color: others OFF (high), selected ON (low)
    if (s_enabled && s_is_on) {
        // Turn OFF all channels first
        GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
        // Then turn ON selected channels
        if (s_mask_c) GPIOC->BSRR = (s_mask_c << 16);
        s_mask_active = s_mask_c;
    }
}

void led_pwm_set_duty_0_2000(uint16_t duty) {
    if (duty > 2000) duty = 2000;
    s_duty_0_2000 = duty;
    s_use_rgb = 0; // revert to single-duty mode
    s_enabled = (duty > 0);
    if (!s_enabled) {
        // Ensure LED off if disabled
    s_is_on = 0;
    // Turn OFF all LED channels
    GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
    }
}

void led_pwm_disable(void) {
    s_enabled = 0;
    s_duty_0_2000 = 0;
    s_is_on = 0;
    // Ensure LED is off so direct rw_led() can take over
    // Active-low: off means drive pins HIGH
    if (s_mask_c) GPIOC->BSRR = s_mask_c; // set bits high
}

void led_pwm_enable_timer_mode(void) {
    s_use_isr = 1;
}

void led_pwm_set_rgb_duty_0_2000(uint16_t r, uint16_t g, uint16_t b) {
    if (r > 2000) r = 2000;
    if (g > 2000) g = 2000;
    if (b > 2000) b = 2000;
    s_r_0_2000 = r; s_g_0_2000 = g; s_b_0_2000 = b;
    s_use_rgb = 1;
    s_enabled = (r | g | b) != 0;
    if (!s_enabled) {
        // OFF all LED channels
        GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
        s_mask_active = 0;
        s_is_on = 0;
    }
}

void led_pwm_isr_step(void) {
    if (!s_enabled) return;
    // Run decision ~1kHz by dividing TIM6 10kHz
    if (++s_isr_div < 10) return;
    s_isr_div = 0;
        if (!s_use_rgb) {
            // Fast paths for extremes
            if (s_duty_0_2000 == 0) {
        if (s_is_on) {
            s_is_on = 0;
            // OFF all channels
            GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
            s_mask_active = 0;
        }
        return;
            }
            if (s_duty_0_2000 >= 2000) {
        if (!s_is_on) {
            s_is_on = 1;
            // Ensure only selected channels ON
            GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
            if (s_mask_c) GPIOC->BSRR = (s_mask_c << 16);
            s_mask_active = s_mask_c;
        }
        return;
            }

            // 2 kHz sigma-delta for lower apparent flicker
            int32_t acc = s_sd_err_isr + (int32_t)s_duty_0_2000;
            uint8_t turn_on = (acc >= 2000);
            s_sd_err_isr = turn_on ? (acc - 2000) : acc;
            if (turn_on != s_is_on) {
                    s_is_on = turn_on;
                    if (s_is_on) {
                            // Only desired color ON; others OFF
                            GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
                            if (s_mask_c) GPIOC->BSRR = (s_mask_c << 16);
                            s_mask_active = s_mask_c;
                    } else {
                            GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
                            s_mask_active = 0;
                    }
            }
        } else {
            // Per-channel mode: compute on/off per channel using sigma-delta
            // Determine which channels should be ON this tick
            uint32_t on_mask = 0;
            // R
            int32_t acc_r = s_sd_r + (int32_t)s_r_0_2000;
            uint8_t on_r = (acc_r >= 2000);
            s_sd_r = on_r ? (acc_r - 2000) : acc_r;
            if (on_r && (s_mask_c & LED_R_Pin)) on_mask |= LED_R_Pin;
            // G
            int32_t acc_g = s_sd_g + (int32_t)s_g_0_2000;
            uint8_t on_g = (acc_g >= 2000);
            s_sd_g = on_g ? (acc_g - 2000) : acc_g;
            if (on_g && (s_mask_c & LED_G_Pin)) on_mask |= LED_G_Pin;
            // B
            int32_t acc_b = s_sd_b + (int32_t)s_b_0_2000;
            uint8_t on_b = (acc_b >= 2000);
            s_sd_b = on_b ? (acc_b - 2000) : acc_b;
            if (on_b && (s_mask_c & LED_B_Pin)) on_mask |= LED_B_Pin;

            // Update GPIOs: first turn everything off, then turn on desired channels
            GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu);
            if (on_mask) GPIOC->BSRR = (on_mask << 16);
            s_mask_active = on_mask;
            s_is_on = (on_mask != 0);
        }
}

void led_pwm_tick(uint32_t now_ms) {
    if (s_use_isr) {
        return; // ISR mode active; no work in main loop
    }
    if (!s_enabled) {
        return; // nothing to do
    }
    // Decide at most once per ms
    if (s_last_decide_ms == now_ms) return;
    s_last_decide_ms = now_ms;

    // Fast paths for extremes
    if (s_duty_0_2000 == 0) {
        if (s_is_on) { s_is_on = 0; GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu); s_mask_active = 0; }
        return;
    }
    if (s_duty_0_2000 >= 2000) {
        if (!s_is_on) { s_is_on = 1; GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu); if (s_mask_c) GPIOC->BSRR = (s_mask_c << 16); s_mask_active = s_mask_c; }
        return;
    }

    // First-order sigma-delta: evenly spaces 1ms ON pulses to reduce flicker
    int32_t acc = s_sd_err + (int32_t)s_duty_0_2000;
    uint8_t turn_on = (acc >= 2000);
    s_sd_err = turn_on ? (acc - 2000) : acc; // keep error bounded
    if (turn_on != s_is_on) {
        s_is_on = turn_on;
        if (s_is_on) { GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu); if (s_mask_c) GPIOC->BSRR = (s_mask_c << 16); s_mask_active = s_mask_c; }
        else { GPIOC->BSRR = (LED_ALL_MASK & 0xFFFFu); s_mask_active = 0; }
    }
}
