#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize display and driver. Must be called after SD and other subsystems are ready.
esp_err_t display_hub75_init(void);

// Start GIF playback from SD path (non-blocking). Returns ESP_OK if playback started.
esp_err_t display_hub75_play_gif(const char *sd_path);

// Stop GIF playback.
void display_hub75_stop_gif(void);

// Set brightness 0..255
void display_hub75_set_brightness(uint8_t level);

// Show a single color immediately (RGB888)
void display_hub75_show_color(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
