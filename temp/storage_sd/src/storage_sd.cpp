#include "storage_sd.h"
#include "app_config.h"
#include "pins.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "ff.h"                 // FatFs directory operations
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>

static const char *TAG = "storage_sd";

// Global configs kept local to this TU
static sdspi_device_config_t slot_config;
static esp_vfs_fat_mount_config_t mount_config;

// esp_err_t storage_sd_init(sd_state_t *state) {
//     if (!state) return ESP_ERR_INVALID_ARG;
//     const sd_config_t &cfg = app_sd_config();

//     // Initialize SPI3 bus for SD over SPI
//     spi_bus_config_t bus_cfg = {};
//     bus_cfg.mosi_io_num = SD_MOSI;
//     bus_cfg.miso_io_num = SD_MISO;
//     bus_cfg.sclk_io_num = SD_SCK;
//     bus_cfg.quadwp_io_num = -1;
//     bus_cfg.quadhd_io_num = -1;
//     bus_cfg.max_transfer_sz = cfg.max_transfer_sz;

//     esp_err_t err = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "spi_bus_initialize(SPI3) failed: %s", esp_err_to_name(err));
//         return err;
//     }

//     // Host/slot configuration for SDSPI
//     sdmmc_host_t host_cfg = SDSPI_HOST_DEFAULT();
//     // host_cfg.slot = SPI3_HOST;  // Bind to the bus we initialized

//     slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//     slot_config.gpio_cs = SD_CS;
//     slot_config.host_id = (spi_host_device_t)host_cfg.slot;

//     // VFS mount configuration
//     mount_config.format_if_mount_failed = cfg.mount_format_if_failed;
//     mount_config.max_files = 8;
//     mount_config.allocation_unit_size = 16 * 1024;

//     sdmmc_card_t *card = nullptr;
//     err = esp_vfs_fat_sdspi_mount(cfg.mount_point, &host_cfg, &slot_config, &mount_config, &card);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "esp_vfs_fat_sdspi_mount failed: %s", esp_err_to_name(err));
//         spi_bus_free(SPI3_HOST);   // free the bus we initialized
//         return err;
//     }

//     sdmmc_card_print_info(stdout, card);
//     state->mounted = true;
//     state->card = card;
//     ESP_LOGI(TAG, "SD mounted at %s", cfg.mount_point);
//     return ESP_OK;
// }

esp_err_t storage_sd_init(sd_state_t *state) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // Correct host config
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;  // tell it which bus we initialized

    // Correct slot config
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = SD_CS;
    slot.host_id = (spi_host_device_t)host.slot;

    // Mount config
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Mount failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    state->mounted = true;
    state->card = card;
    return ESP_OK;
}

esp_err_t storage_sd_deinit(sd_state_t *state) {
    if (!state) return ESP_ERR_INVALID_ARG;
    const sd_config_t &cfg = app_sd_config();

    if (state->mounted) {
        esp_err_t err = esp_vfs_fat_sdcard_unmount(cfg.mount_point, state->card);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Unmount returned %s", esp_err_to_name(err));
        }
        state->mounted = false;
        state->card = nullptr;
    }
    spi_bus_free(SPI3_HOST);
    return ESP_OK;
}

esp_err_t storage_sd_write_text(const char *path, const char *text) {
    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGE(TAG, "fopen(%s) failed, errno=%d", path, errno);
        return ESP_FAIL;
    }
    size_t n = fwrite(text, 1, strlen(text), f);
    fclose(f);
    ESP_LOGI(TAG, "Wrote %u bytes to %s", (unsigned)n, path);
    return n ? ESP_OK : ESP_FAIL;
}

esp_err_t storage_sd_read_text(const char *path, char *buf, size_t buf_sz, size_t *out_len) {
    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "fopen(%s) failed, errno=%d", path, errno);
        return ESP_FAIL;
    }
    size_t n = fread(buf, 1, buf_sz - 1, f);
    buf[n] = '\0';
    if (out_len) *out_len = n;
    fclose(f);
    ESP_LOGI(TAG, "Read %u bytes from %s", (unsigned)n, path);
    return ESP_OK;
}


esp_err_t storage_sd_list_dir(const char *path) {
    DIR *dir;
    FILINFO fno;
    FRESULT res = FR_OK;

    dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed", path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Listing %s", path);
    for (;;) {
        struct dirent *entry = readdir(dir);
        if (!entry) break;
        ESP_LOGI(TAG, " - %s", entry->d_name);
    }

    closedir(dir);
    return ESP_OK;
}