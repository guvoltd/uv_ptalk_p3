#include "app_config.h"
#include <string.h>

static sd_config_t g_sd_cfg = {
    .mount_point = "/sdcard",
    .spi_clock_hz = 20000000,       // start at 20 MHz, tune as needed
    .max_transfer_sz = 4096,
    .mount_format_if_failed = 0
};

const sd_config_t& app_sd_config() {
    return g_sd_cfg;
}

esp_err_t app_sd_config_overrides(const sd_config_t* cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    memcpy(&g_sd_cfg, cfg, sizeof(sd_config_t));
    return ESP_OK;
}
