#pragma once
#include "esp_err.h"

typedef struct {
    char mount_point[16];       // e.g., "/sdcard"
    int spi_clock_hz;           // e.g., 20000000
    int max_transfer_sz;        // DMA transfer size
    int mount_format_if_failed; // boolean
} sd_config_t;

const sd_config_t& app_sd_config();
esp_err_t app_sd_config_overrides(const sd_config_t* cfg);
