#include "display_hub75.h"
#include "pins.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "MatrixPanel_I2S_DMA.h" // from mrfaptastic driver
#include <string.h>

static const char *TAG = "display_hub75";

// MatrixPanel object pointer
static MatrixPanel_I2S_DMA *matrix = nullptr;
static TaskHandle_t s_display_task = nullptr;
static volatile bool s_patterns_running = false;
static volatile uint8_t s_brightness = 128; // 0..255

// Soft-start timing
static const TickType_t POWER_ON_DELAY_MS = pdMS_TO_TICKS(150);
static const TickType_t PATTERN_DELAY_MS = pdMS_TO_TICKS(800);

// Helper: configure panel power pin
static void panel_power_gpio_init() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << HUB75_POWER_EN);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    // default off
    gpio_set_level(HUB75_POWER_EN, 0);
}

// Helper: ensure OE disabled (outputs off) before power transitions
static void panel_disable_outputs() {
    // OE pin is controlled by the MatrixPanel driver, but we can force it low/high via gpio
    // Many panels: OE HIGH disables outputs. We will set OE HIGH to disable outputs.
    gpio_set_level(HUB75_OE, 1);
}

// Helper: enable outputs (OE low)
static void panel_enable_outputs() {
    gpio_set_level(HUB75_OE, 0);
}

// Initialize MatrixPanel with pins from pins.h
static bool init_matrix_driver() {
    if (matrix) return true;

    HUB75_I2S_CFG cfg(64, 64, 1); // width, height, panels chained
    cfg.gpio.r1 = HUB75_R1;
    cfg.gpio.g1 = HUB75_G1;
    cfg.gpio.b1 = HUB75_B1;
    cfg.gpio.r2 = HUB75_R2;
    cfg.gpio.g2 = HUB75_G2;
    cfg.gpio.b2 = HUB75_B2;
    cfg.gpio.a  = HUB75_A;
    cfg.gpio.b  = HUB75_B;
    cfg.gpio.c  = HUB75_C;
    cfg.gpio.d  = HUB75_D;
    cfg.gpio.e  = HUB75_E;
    cfg.gpio.clk = HUB75_CLK;
    cfg.gpio.lat = HUB75_LAT;
    cfg.gpio.oe  = HUB75_OE;

    // Driver options: use SHIFTREG or SHIFTREG_32BIT depending on library
    cfg.driver = HUB75_I2S_CFG::SHIFTREG;

    matrix = new MatrixPanel_I2S_DMA(cfg);
    if (!matrix) {
        ESP_LOGE(TAG, "Matrix allocation failed");
        return false;
    }
    matrix->begin();
    matrix->setBrightness(s_brightness); // 0..255
    matrix->clearScreen();
    matrix->showBuffer();
    return true;
}

// Display task: runs patterns when requested
static void display_task(void *arg) {
    ESP_LOGI(TAG, "Display task started");
    while (1) {
        if (s_patterns_running && matrix) {
            // Color bars
            matrix->fillScreenRGB888(255, 0, 0); matrix->showBuffer(); vTaskDelay(PATTERN_DELAY_MS);
            matrix->fillScreenRGB888(0, 255, 0); matrix->showBuffer(); vTaskDelay(PATTERN_DELAY_MS);
            matrix->fillScreenRGB888(0, 0, 255); matrix->showBuffer(); vTaskDelay(PATTERN_DELAY_MS);

            // Checkerboard
            for (int y = 0; y < 64; y++) {
                for (int x = 0; x < 64; x++) {
                    if (((x / 8) + (y / 8)) & 1) matrix->drawPixelRGB888(x, y, 255, 255, 255);
                    else matrix->drawPixelRGB888(x, y, 0, 0, 0);
                }
            }
            matrix->showBuffer(); vTaskDelay(PATTERN_DELAY_MS);

            // Scrolling text (simple)
            matrix->clearScreen();
            matrix->setTextSize(1);
            matrix->setTextColorRGB888(255, 255, 0);
            const char *msg = "ESP32-S3 P3 64x64";
            for (int offset = 64; offset > -((int)strlen(msg) * 6); offset--) {
                matrix->clearScreen();
                matrix->setCursor(offset, 28);
                matrix->print(msg);
                matrix->showBuffer();
                vTaskDelay(pdMS_TO_TICKS(40));
            }
        } else {
            // Idle: small delay
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// Public API implementations

esp_err_t display_hub75_start(void) {
    // Initialize power pin and OE pin as outputs
    panel_power_gpio_init();

    // Ensure OE pin is configured as output so we can force it during power transitions
    gpio_config_t oe_cfg = {};
    oe_cfg.pin_bit_mask = (1ULL << HUB75_OE);
    oe_cfg.mode = GPIO_MODE_OUTPUT;
    oe_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    oe_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    oe_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&oe_cfg);
    // Disable outputs initially
    panel_disable_outputs();

    // Power on sequence
    panel_power_set(true);
    vTaskDelay(POWER_ON_DELAY_MS);

    // Initialize matrix driver
    if (!init_matrix_driver()) {
        ESP_LOGE(TAG, "Matrix init failed");
        return ESP_FAIL;
    }

    // Start display task
    if (!s_display_task) {
        xTaskCreatePinnedToCore(display_task, "display_task", 4096, NULL, 6, &s_display_task, 0);
    }

    // Enable outputs after driver ready
    panel_enable_outputs();
    ESP_LOGI(TAG, "Display started");
    return ESP_OK;
}

void panel_power_set(bool on) {
    // If using a MOSFET that is active-high to enable 5V, set accordingly.
    // If your MOSFET is active-low, invert the logic here.
    gpio_set_level(HUB75_POWER_EN, on ? 1 : 0);
    if (!on) {
        // Immediately disable outputs to avoid ghosting
        panel_disable_outputs();
    }
}

void display_set_brightness(uint8_t level) {
    s_brightness = level;
    if (matrix) {
        matrix->setBrightness(level);
    }
}

void display_show_color(uint8_t r, uint8_t g, uint8_t b) {
    if (!matrix) return;
    matrix->fillScreenRGB888(r, g, b);
    matrix->showBuffer();
}

void display_start_test_patterns(void) {
    s_patterns_running = true;
}

void display_stop_patterns(void) {
    s_patterns_running = false;
    if (matrix) {
        matrix->clearScreen();
        matrix->showBuffer();
    }
}
