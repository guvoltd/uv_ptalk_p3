#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "display_hub75.h"
#include "pins.h"

static const char *TAG = "main_display_demo";

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Starting display module demo");

    // Start display
    if (display_hub75_start() != ESP_OK) {
        ESP_LOGE(TAG, "display_hub75_start failed");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Demo sequence
    ESP_LOGI(TAG, "Showing color sequence");
    display_show_color(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(1000));
    display_show_color(0, 255, 0); vTaskDelay(pdMS_TO_TICKS(1000));
    display_show_color(0, 0, 255); vTaskDelay(pdMS_TO_TICKS(1000));
    display_show_color(255, 255, 255); vTaskDelay(pdMS_TO_TICKS(1000));
    display_show_color(0, 0, 0); vTaskDelay(pdMS_TO_TICKS(500));

    // Start patterns
    ESP_LOGI(TAG, "Starting test patterns");
    display_start_test_patterns();

    // Ramp brightness up and down
    for (int b = 64; b <= 255; b += 32) {
        display_set_brightness((uint8_t)b);
        ESP_LOGI(TAG, "Brightness %d", b);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    for (int b = 255; b >= 64; b -= 32) {
        display_set_brightness((uint8_t)b);
        ESP_LOGI(TAG, "Brightness %d", b);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Keep patterns running; log heartbeat
    while (1) {
        ESP_LOGI(TAG, "Display demo running");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
