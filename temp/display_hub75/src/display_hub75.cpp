// display_hub75.cpp
#include "display_hub75.h"
#include "pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdio.h>

// Matrix driver header (the header you provided)
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"

// Animated GIF library (Arduino-style). Ensure this library is available in components/arduino/libraries or as a component.
#include "ff.h"           // FatFS types: FIL, f_open, f_read, f_lseek, f_close
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "AnimatedGIF.h"  // ensure AnimatedGIF library is present; we will use callback-style I/O

// Globals used by GIF task
static FIL gif_fil;                 // FatFS file handle for the opened GIF
static bool gif_fil_open = false;

static const char *TAG = "display_hub75";

static MatrixPanel_I2S_DMA *matrix = nullptr;
static TaskHandle_t s_gif_task = NULL;
static volatile bool s_gif_running = false;
static AnimatedGIF gif; // global decoder instance
static char s_gif_path[256] = {0};
static uint8_t s_brightness = 128;

// Forward declarations
static bool init_matrix_driver(void);
static void gif_task(void *arg);
static void gif_draw_callback(GIFDRAW *pDraw);
static void panel_power_gpio_init(void);

// Initialize power pin and OE pin (if you use them)
static void panel_power_gpio_init() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << HUB75_POWER_EN);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(HUB75_POWER_EN, 1); // enable power by default; change if your hardware differs
}

// Initialize matrix driver using HUB75_I2S_CFG and begin()
static bool init_matrix_driver(void) {
    if (matrix) return true;

    HUB75_I2S_CFG cfg(MATRIX_WIDTH, MATRIX_HEIGHT, 1);
    // map pins from pins.h (ensure pins.h defines these)
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
    cfg.gpio.lat = HUB75_LAT;
    cfg.gpio.oe  = HUB75_OE;
    cfg.gpio.clk = HUB75_CLK;

    // driver type and options (match your panel)
    cfg.driver = HUB75_I2S_CFG::SHIFTREG;
    cfg.double_buff = true;            // enable double buffering for smooth frames
    cfg.i2sspeed = HUB75_I2S_CFG::HZ_15M;
    cfg.latch_blanking = DEFAULT_LAT_BLANKING;
    cfg.min_refresh_rate = 60;

    matrix = new MatrixPanel_I2S_DMA(cfg);
    if (!matrix) {
        ESP_LOGE(TAG, "Matrix allocation failed");
        return false;
    }

    if (!matrix->begin()) {
        ESP_LOGE(TAG, "Matrix begin() failed");
        delete matrix;
        matrix = nullptr;
        return false;
    }

    matrix->setBrightness(s_brightness);
    matrix->clearScreen();
    // flip initial buffer so DMA sees a clean buffer
    matrix->flipDMABuffer();
    ESP_LOGI(TAG, "Matrix initialized");
    return true;
}

// --- FatFS read/seek wrappers for AnimatedGIF ---
// These functions are passed to the AnimatedGIF decoder as the I/O layer.
// Signature may vary between AnimatedGIF forks; adapt if your fork uses different names/params.

// Read 'len' bytes into buf. Return number of bytes read (or -1 on error).
static int gif_read(void *user, uint8_t *buf, int len) {
    (void)user; // user can be used to pass context if needed
    if (!gif_fil_open) return -1;
    UINT br = 0;
    FRESULT res = f_read(&gif_fil, buf, (UINT)len, &br);
    if (res != FR_OK) {
        ESP_LOGE("GIF_IO", "f_read failed: %d", res);
        return -1;
    }
    return (int)br;
}

// Seek to absolute position 'pos' in file. Return 0 on success, -1 on error.
static int gif_seek(void *user, long pos) {
    (void)user;
    if (!gif_fil_open) return -1;
    FRESULT res = f_lseek(&gif_fil, (FSIZE_t)pos);
    if (res != FR_OK) {
        ESP_LOGE("GIF_IO", "f_lseek failed: %d", res);
        return -1;
    }
    return 0;
}

// Optional: tell file size (some decoders ask). Return file length or -1.
static long gif_tell_size(void *user) {
    if (!gif_fil_open) return -1;
    return (long)f_size(&gif_fil);
}

// --- GIF draw callback (same as AnimatedGIFPanel_SD style) ---
// GIFDRAW *pDraw contains pPixels, pPalette, iWidth, iHeight, iX, iY, iBitDepth, etc.
// This callback must write the decoded block into the matrix DMA buffer.
static void gif_draw_callback(GIFDRAW *pDraw) {
    if (!matrix) return;

    // If decoder provides indexed pixels with palette:
    if (pDraw->pPalette) {
        const uint8_t *pixels = pDraw->pPixels;
        const uint8_t *palette = (uint8_t*)pDraw->pPalette; // palette is RGB triplets
        int w = pDraw->iWidth;
        int h = pDraw->iHeight;
        int x0 = pDraw->iX;
        int y0 = pDraw->iY;
        for (int yy = 0; yy < h; ++yy) {
            for (int xx = 0; xx < w; ++xx) {
                int idx = pixels[yy * w + xx];
                uint8_t r = palette[idx * 3 + 0];
                uint8_t g = palette[idx * 3 + 1];
                uint8_t b = palette[idx * 3 + 2];
                matrix->drawPixelRGB888(x0 + xx, y0 + yy, r, g, b);
            }
        }
    } else {
        // If decoder provides direct RGB888 pixels in pPixels (24-bit)
        const uint8_t *pixels = pDraw->pPixels;
        int w = pDraw->iWidth;
        int h = pDraw->iHeight;
        int x0 = pDraw->iX;
        int y0 = pDraw->iY;
        for (int yy = 0; yy < h; ++yy) {
            for (int xx = 0; xx < w; ++xx) {
                int base = (yy * w + xx) * 3;
                uint8_t r = pixels[base + 0];
                uint8_t g = pixels[base + 1];
                uint8_t b = pixels[base + 2];
                matrix->drawPixelRGB888(x0 + xx, y0 + yy, r, g, b);
            }
        }
    }
}

static int gifPlay( const char* gifPath )
{ // 0=infinite

  if( ! gif.open( gifPath, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw ) ) {
    log_n("Could not open gif %s", gifPath );
  }
  ESP_LOGI(__func__, "Opening GIF: %s", gifPath);

  int frameDelay = 0; // store delay for the last frame
  int then = 0; // store overall delay

  while (gif.playFrame(true, &frameDelay)) {

    then += frameDelay;
    if( then > maxGifDuration ) { // avoid being trapped in infinite GIF's
      //log_w("Broke the GIF loop, max duration exceeded");
      break;
    }
  }

  gif.close();

  return then;
}

// --- GIF playback task using FatFS ---
static void gif_task(void *arg) {
    const char *path = (const char *)arg;
    ESP_LOGI("GIF_TASK", "Opening GIF: %s", path);

    // // Open with FatFS
    // FRESULT fres = f_open(&gif_fil, path, FA_READ);
    // if (fres != FR_OK) {
    //     ESP_LOGE("GIF_TASK", "f_open failed: %d", fres);
    //     gif_fil_open = false;
    //     vTaskDelete(NULL);
    //     return;
    // }
    // gif_fil_open = true;

    // Initialize AnimatedGIF decoder with our read/seek callbacks.
    // Many AnimatedGIF forks provide a begin/open API that accepts function pointers.
    // Example pseudo-call (adapt to your AnimatedGIF variant):
    // gif.begin(gif_read, gif_seek, gif_tell_size, gif_draw_callback, NULL);
    // If your AnimatedGIF has a different API, adapt accordingly.
    // open(name, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)
    // if (!gif.open((path, gif_read, gif_seek, gif_tell_size, gif_draw_callback)) {
    //     ESP_LOGE("GIF_TASK", "AnimatedGIF begin failed");
    //     f_close(&gif_fil);
    //     gif_fil_open = false;
    //     vTaskDelete(NULL);
    //     return;
    // }
    gif_fil_open = gifPlay(path);

    // Start decoding loop
    while (s_gif_running) {
        int delay_ms = gif.playFrame(true, NULL); // API variant; adapt if needed
        // Present frame: flip DMA buffer so the driver outputs the new frame
        matrix->flipDMABuffer();
        if (delay_ms <= 0) delay_ms = 40;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Cleanup
    gif.close();
    f_close(&gif_fil);
    gif_fil_open = false;
    ESP_LOGI("GIF_TASK", "GIF playback stopped");
    vTaskDelete(NULL);
}

// Public API implementations

esp_err_t display_hub75_init(void) {
    ();
    if (!init_matrix_driver()) {
        return ESP_FAIL;
    }
    return ESP_OK;panel_power_gpio_init
}

esp_err_t display_hub75_play_gif(const char *sd_path) {
    if (!sd_path || strlen(sd_path) == 0) return ESP_ERR_INVALID_ARG;
    if (!matrix) {
        ESP_LOGE(TAG, "Matrix not initialized");
        return ESP_FAIL;
    }

    // copy path
    strncpy(s_gif_path, sd_path, sizeof(s_gif_path) - 1);
    s_gif_path[sizeof(s_gif_path) - 1] = '\0';

    if (s_gif_running) {
        ESP_LOGW(TAG, "GIF already running, restarting");
        display_hub75_stop_gif();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    s_gif_running = true;
    // create task
    xTaskCreatePinnedToCore(gif_task, "gif_task", 8192, NULL, 5, &s_gif_task, 1);
    return ESP_OK;
}

void display_hub75_stop_gif(void) {
    if (!s_gif_running) return;
    s_gif_running = false;
    // task will self-delete; wait a bit
    for (int i = 0; i < 20 && s_gif_task != NULL; ++i) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_gif_task = NULL;
}

void display_hub75_set_brightness(uint8_t level) {
    s_brightness = level;
    if (matrix) matrix->setBrightness(level);
}

void display_hub75_show_color(uint8_t r, uint8_t g, uint8_t b) {
    if (!matrix) return;
    matrix->fillScreenRGB888(r, g, b);
    matrix->flipDMABuffer();
}
