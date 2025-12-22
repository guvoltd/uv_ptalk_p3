#include "logging.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"

void logging_init() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    esp_reset_reason_t reason = esp_reset_reason();
    uint32_t freq = 240; // typical target
    ESP_LOGI("boot", "ESP32-S3 cores=%d wifi=%d ble=%d rev=%d reset=%d",
             chip_info.cores, chip_info.features & CHIP_FEATURE_WIFI_BGN,
             chip_info.features & CHIP_FEATURE_BLE, chip_info.revision, reason);
    ESP_LOGI("boot", "CPU target freq ~%u MHz", freq);
}
