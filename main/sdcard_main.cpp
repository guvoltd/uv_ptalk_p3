#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "storage_sd.h"
#include "app_config.h"
#include "logging.h"
#include "pins.h"
#include <sys/stat.h>
#include <stdio.h>  

static const char *TAG = "app";

extern "C" void app_main(void) {
    // NVS init for IDF subsystems
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    logging_init();

    // Optional: override SD config (SPI clock, etc.)
    sd_config_t override = app_sd_config();
    override.spi_clock_hz = 20000000;      // start 20 MHz
    override.max_transfer_sz = 4096;
    app_sd_config_overrides(&override);

    sd_state_t sd = {};
    if (storage_sd_init(&sd) != ESP_OK) {
        ESP_LOGE(TAG, "SD init failed, halting");
        vTaskDelay(portMAX_DELAY);
        return;
    }

    // Create /sdcard/test directory if not exists
    const char *mount_point = app_sd_config().mount_point;
    char dir_path[64];
    snprintf(dir_path, sizeof(dir_path), "%s/%s", mount_point, "test");
    
    struct stat st = {};
    if (stat(dir_path, &st) != 0) {
        ESP_LOGI(TAG, "Creating directory: %s", dir_path);
        mkdir(dir_path, 0777);
    }

    // Write a test file
    char file_path[96];
    snprintf(file_path, sizeof(file_path), "%s/%s/%s", mount_point, "test", "hello.txt");
    const char *content = "Hello, SD card! This is Module 1 test.\nLine 2.\n";
    storage_sd_write_text(file_path, content);

    // Read it back
    char buf[256];
    size_t n = 0;
    storage_sd_read_text(file_path, buf, sizeof(buf), &n);
    ESP_LOGI(TAG, "File content (%u bytes):\n%s", (unsigned)n, buf);

    // List /sdcard and /sdcard/test
    storage_sd_list_dir(mount_point);
    storage_sd_list_dir(dir_path);

    ESP_LOGI(TAG, "Module 1 demo complete. SD is mounted and working.");

    // Keep running; in a fuller app we would perform more operations
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Note: We don't unmount in this demo loop; if you add exit logic:
    // storage_sd_deinit(&sd);
}
