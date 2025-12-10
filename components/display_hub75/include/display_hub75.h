#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DISPLAY_OK = 0,
    DISPLAY_ERR_INIT = -1,
    DISPLAY_ERR_NO_POWER = -2
} display_err_t;

// Start the display task and initialize hardware. Returns ESP_OK on success.
esp_err_t display_hub75_start(void);

// Set panel power (MOSFET enable). true = power on, false = power off.
void panel_power_set(bool on);

// Set brightness 0..255 (0 = off, 255 = max). This is a software brightness applied to driver.
void display_set_brightness(uint8_t level);

// Show a single solid color immediately (RGB888).
void display_show_color(uint8_t r, uint8_t g, uint8_t b);

// Show a test pattern sequence (non-blocking).
void display_start_test_patterns(void);

// Stop test patterns and blank display.
void display_stop_patterns(void);

#ifdef __cplusplus
}
#endif
