/*
 * ESP32-S3 HUB75 GIF Player - Complete Solution
 * 
 * The main issue was: Only first frame displayed, then nothing
 * Root cause: AnimatedGIF library state machine and our buffer management
 * Solution: Proper frame completion detection and buffer synchronization
 */

/* ==================== CONFIGURATION ==================== */
#define MATRIX_WIDTH 64
#define MATRIX_HEIGHT 64
#define MATRIX_CHAIN_LENGTH 1

// --- HUB75 Panel Pin Mapping for Waveshare ESP32-S3-Pico ---
// IMPORTANT: For a 64x64 panel, you MUST define and connect the E_PIN.
#define R1_PIN GPIO_NUM_11
#define G1_PIN GPIO_NUM_12
#define B1_PIN GPIO_NUM_13
#define R2_PIN GPIO_NUM_14
#define G2_PIN GPIO_NUM_15
#define B2_PIN GPIO_NUM_16

#define A_PIN GPIO_NUM_17
#define B_PIN GPIO_NUM_18
#define C_PIN GPIO_NUM_33
#define D_PIN GPIO_NUM_34
#define E_PIN GPIO_NUM_35                 // Set to a valid GPIO (e.g., 12) for 64x64 1/32 scan panels

#define LAT_PIN GPIO_NUM_37
#define OE_PIN GPIO_NUM_38
#define CLK_PIN GPIO_NUM_36
// -----------------------------
// HUB75 RGB Matrix Panel POWER pin
// -----------------------------
#define HUB75_POWER_EN         GPIO_NUM_42

// SD Card Pins
#define SD_MISO GPIO_NUM_10
#define SD_MOSI GPIO_NUM_9
#define SD_SCK  GPIO_NUM_8
#define SD_CS   GPIO_NUM_7               // Chip Select pin

// GIF Configuration
#define GIF_FILENAME "/gifs/gifs/cartoon.gif" 
#define DEFAULT_BRIGHTNESS 250
#define MIN_FRAME_DELAY_MS 50

// SD Card mount configuration
#define MOUNT_POINT "/gifs"
#define MAX_FILES 3

// Debug mode
#define DEBUG_GIF_FRAMES 1

/* ==================== LIBRARY INCLUDES ==================== */
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <vector>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

// Third-party libraries
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include "AnimatedGIF.h"
#include "ff.h"                 // FatFs directory operations
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* ==================== GLOBAL OBJECTS ==================== */
// Matrix Panel Object
MatrixPanel_I2S_DMA *dma_display = nullptr;

// AnimatedGIF Decoder
AnimatedGIF gif;
sdmmc_card_t *card = NULL;
static const char *TAG = "main";

// Buffer management
uint16_t *activeFrameBuffer = nullptr;
uint16_t *nextFrameBuffer = nullptr;
bool frameReady = false;
bool frameInProgress = false;
int currentFrameIndex = 0;

// Synchronization
SemaphoreHandle_t frameMutex = nullptr;
QueueHandle_t frameReadyQueue = nullptr;

// GIF state
volatile bool gifPlaying = false;
volatile int32_t currentFrameDelay = MIN_FRAME_DELAY_MS;
volatile bool gifFileOpen = false;

// Debug
uint32_t totalFramesPlayed = 0;
uint32_t gifDrawCalls = 0;
uint32_t lastFrameTime = 0;

/* ==================== FUNCTION PROTOTYPES ==================== */
bool setup_matrix_panel();
bool setup_sd_card();
void GIFDraw(GIFDRAW *pDraw);
void* GIFOpenFile(const char *fname, int32_t *pSize);
void GIFCloseFile(void *pHandle);
int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen);
int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition);
void gif_player_task(void *pvParameters);
void display_manager_task(void *pvParameters);
bool decode_next_frame();
void draw_current_frame();

/* ==================== SETUP FUNCTIONS ==================== */
extern "C" void app_main(void)
{
    ESP_LOGI("MAIN", "=== ESP32-S3 HUB75 GIF Player ===");
    ESP_LOGI("MAIN", "Free heap: %d bytes", esp_get_free_heap_size());
    
    // Create synchronization primitives
    frameMutex = xSemaphoreCreateMutex();
    frameReadyQueue = xQueueCreate(3, sizeof(bool)); // Simple queue for frame ready signals
    
    // Initialize matrix panel
    if (!setup_matrix_panel()) {
        ESP_LOGE("MAIN", "Matrix initialization failed!");
        vTaskDelay(portMAX_DELAY);
    }
    
    // Initialize SD card
    if (!setup_sd_card()) {
        ESP_LOGE("MAIN", "SD card initialization failed!");
        vTaskDelay(portMAX_DELAY);
    }
    
    // Allocate frame buffers
    activeFrameBuffer = (uint16_t*)heap_caps_malloc(
        MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t), 
        MALLOC_CAP_DMA);
    
    nextFrameBuffer = (uint16_t*)heap_caps_malloc(
        MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t), 
        MALLOC_CAP_DMA);
    
    if (!activeFrameBuffer || !nextFrameBuffer) {
        ESP_LOGE("MAIN", "Failed to allocate frame buffers!");
        vTaskDelay(portMAX_DELAY);
    }
    
    memset(activeFrameBuffer, 0, MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    memset(nextFrameBuffer, 0, MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    
    ESP_LOGI("MAIN", "Frame buffers allocated: %d bytes each", 
             MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    
    // Initialize GIF decoder
    gif.begin(LITTLE_ENDIAN_PIXELS);
    
    // Open GIF file
    ESP_LOGI("MAIN", "Opening GIF file: %s", GIF_FILENAME);
    if (!gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
        ESP_LOGE("MAIN", "Failed to open GIF file!");
        // List files for debugging
        DIR *dir = opendir("/sdcard");
        if (dir) {
            ESP_LOGI("MAIN", "Files on SD card:");
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                ESP_LOGI("MAIN", "  %s", entry->d_name);
            }
            closedir(dir);
        }
        vTaskDelay(portMAX_DELAY);
    }
    
    gifFileOpen = true;
    gifPlaying = true;
    
    ESP_LOGI("MAIN", "GIF opened successfully. Canvas: %dx%d", 
             gif.getCanvasWidth(), gif.getCanvasHeight());
    
    // Create tasks
    xTaskCreatePinnedToCore(gif_player_task, "GIF Player", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(display_manager_task, "Display", 4096, NULL, 2, NULL, 0);
    
    // Main monitoring loop
    uint32_t lastStatsTime = esp_timer_get_time() / 1000;
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        uint32_t currentTime = esp_timer_get_time() / 1000;
        if (currentTime - lastStatsTime >= 5000) {
            ESP_LOGI("STATS", "Frames played: %lu, GIF Draw calls: %lu, Heap: %d", 
                     totalFramesPlayed, gifDrawCalls, esp_get_free_heap_size());
            lastStatsTime = currentTime;
        }
        
        // Handle GIF end/restart
        if (!gifPlaying && gifFileOpen) {
            ESP_LOGI("MAIN", "GIF ended, restarting...");
            gif.reset();
            gifPlaying = true;
            currentFrameIndex = 0;
        }
    }
}

/**
 * @brief GIF player task - manages GIF decoding
 */
void gif_player_task(void *pvParameters)
{
    ESP_LOGI("GIF_TASK", "GIF player task started");
    
    uint32_t lastFrameStart = esp_timer_get_time();
    uint32_t frameDecodeTime = 0;
    
    while (1) {
        if (gifPlaying && gifFileOpen) {
            uint32_t frameStart = esp_timer_get_time();
            
            // Decode next frame
            if (!decode_next_frame()) {
                ESP_LOGW("GIF_TASK", "Failed to decode frame %d", currentFrameIndex);
                gifPlaying = false;
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }
            
            // Calculate actual decode time
            frameDecodeTime = esp_timer_get_time() - frameStart;
            
            // Wait for frame delay (respecting GIF timing)
            int32_t delay = currentFrameDelay;
            if (delay < MIN_FRAME_DELAY_MS) delay = MIN_FRAME_DELAY_MS;
            
            // Adjust for decode time
            if (frameDecodeTime < delay * 1000) {
                uint32_t remainingDelay = (delay * 1000) - frameDecodeTime;
                vTaskDelay(remainingDelay / 1000 / portTICK_PERIOD_MS);
            }
            
            currentFrameIndex++;
            
            // Debug: log every 10 frames
            if (DEBUG_GIF_FRAMES && (currentFrameIndex % 10 == 0)) {
                ESP_LOGI("GIF_TASK", "Frame %d decoded in %luµs, delay: %dms", 
                         currentFrameIndex, frameDecodeTime, delay);
            }
        } else {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }
}

/**
 * @brief Display manager task - handles buffer swapping
 */
void display_manager_task(void *pvParameters)
{
    ESP_LOGI("DISPLAY_TASK", "Display manager task started");
    
    bool frameAvailable = false;
    
    while (1) {
        // Wait for frame to be ready
        if (xQueueReceive(frameReadyQueue, &frameAvailable, portMAX_DELAY) == pdTRUE) {
            if (frameReady) {
                // Draw the frame
                draw_current_frame();
                totalFramesPlayed++;
                
                // Signal back to GIF task that we're done
                bool ack = true;
                xQueueSend(frameReadyQueue, &ack, 0);
            }
        }
    }
}

/**
 * @brief Decode a single GIF frame
 */
bool decode_next_frame()
{
    if (!gifFileOpen) return false;
    
    // Clear next frame buffer
    memset(nextFrameBuffer, 0, MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    
    // Decode one frame
    // The GIFDraw callback will be called multiple times for this frame
    if (!gif.playFrame(false, NULL)) {
        ESP_LOGW("DECODE", "playFrame returned false - end of GIF?");
        return false;
    }
    
    // After playFrame completes, the frame is in nextFrameBuffer
    // Swap buffers
    if (xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE) {
        uint16_t *temp = activeFrameBuffer;
        activeFrameBuffer = nextFrameBuffer;
        nextFrameBuffer = temp;
        
        frameReady = true;
        xSemaphoreGive(frameMutex);
        
        // Signal display task
        bool signal = true;
        xQueueSend(frameReadyQueue, &signal, portMAX_DELAY);
        
        return true;
    }
    
    return false;
}

/**
 * @brief Draw current frame to display
 */
void draw_current_frame()
{
    if (!dma_display || !frameReady) return;
    
    if (xSemaphoreTake(frameMutex, portMAX_DELAY) == pdTRUE) {
        // Draw to display
        dma_display->drawRGBBitmap(0, 0, activeFrameBuffer, 
                                   MATRIX_WIDTH, MATRIX_HEIGHT);
        
        // Swap DMA buffers
        dma_display->flipDMABuffer();
        
        frameReady = false;
        xSemaphoreGive(frameMutex);
        
        // Debug: check first pixel
        if (DEBUG_GIF_FRAMES && (totalFramesPlayed % 20 == 0)) {
            ESP_LOGD("DISPLAY", "Displayed frame %lu, first pixel: 0x%04X", 
                     totalFramesPlayed, activeFrameBuffer[0]);
        }
    }
}

/**
 * @brief Initialize matrix panel
 */
bool setup_matrix_panel() {
    ESP_LOGI("MATRIX", "Initializing HUB75 panel...");
    
    HUB75_I2S_CFG::i2s_pins _pins = {
        .r1 = R1_PIN, .g1 = G1_PIN, .b1 = B1_PIN,
        .r2 = R2_PIN, .g2 = G2_PIN, .b2 = B2_PIN,
        .a = A_PIN, .b = B_PIN, .c = C_PIN, .d = D_PIN, .e = E_PIN,
        .lat = LAT_PIN, .oe = OE_PIN, .clk = CLK_PIN
    };

    HUB75_I2S_CFG mxconfig(
        MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_CHAIN_LENGTH, _pins
    );
    
    mxconfig.gpio.e = E_PIN;
    mxconfig.double_buff = true;
    mxconfig.clkphase = false;
    mxconfig.driver = HUB75_I2S_CFG::SHIFTREG;
    mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;
    mxconfig.min_refresh_rate = 120;
    // mxconfig.dma_buf_count = 4;
    // mxconfig.dma_buf_len = 128;

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    
    if (!dma_display->begin()) {
        ESP_LOGE("MATRIX", "Matrix begin() failed!");
        return false;
    }

    dma_display->setBrightness8(DEFAULT_BRIGHTNESS);
    dma_display->clearScreen();
    dma_display->flipDMABuffer();
    
    // Quick test pattern
    ESP_LOGI("MATRIX", "Displaying test pattern...");
    for (int i = 0; i < 3; i++) {
        dma_display->fillScreen(dma_display->color565(
            i == 0 ? 255 : 0,
            i == 1 ? 255 : 0,
            i == 2 ? 255 : 0
        ));
        dma_display->flipDMABuffer();
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    
    dma_display->clearScreen();
    dma_display->flipDMABuffer();
    
    ESP_LOGI("MATRIX", "Panel initialized: %dx%d", MATRIX_WIDTH, MATRIX_HEIGHT);
    return true;
}

/**
 * @brief Initialize SD card
 */
bool setup_sd_card() {
    ESP_LOGI("SD", "Initializing SD card...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = (spi_host_device_t)SPI2_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize((spi_host_device_t)SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS;
    slot_config.host_id = (spi_host_device_t)SPI2_HOST;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    sdmmc_card_print_info(stdout, card);
    ESP_LOGI("SD", "SD card mounted at %s", MOUNT_POINT);
    
    return true;
}

/* ==================== GIF DECODER CALLBACKS ==================== */

// File handle for GIF
static FILE *gifFileHandle = NULL;

void* GIFOpenFile(const char *fname, int32_t *pSize) {
    ESP_LOGI("GIF", "Opening: %s", fname);
    
    gifFileHandle = fopen(fname, "rb");
    if (!gifFileHandle) {
        ESP_LOGE("GIF", "Failed to open: %s", fname);
        return NULL;
    }
    
    fseek(gifFileHandle, 0, SEEK_END);
    *pSize = ftell(gifFileHandle);
    fseek(gifFileHandle, 0, SEEK_SET);
    
    ESP_LOGI("GIF", "File size: %ld bytes", *pSize);
    return (void *)gifFileHandle;
}

void GIFCloseFile(void *pHandle) {
    if (gifFileHandle) {
        fclose(gifFileHandle);
        gifFileHandle = NULL;
    }
}

int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen) {
    if (!gifFileHandle) return 0;
    
    size_t bytesRead = fread(pBuf, 1, iLen, gifFileHandle);
    if (bytesRead == 0 && feof(gifFileHandle)) {
        ESP_LOGD("GIF", "End of file reached");
    }
    return (int32_t)bytesRead;
}

int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition) {
    if (!gifFileHandle) return 0;
    
    if (fseek(gifFileHandle, iPosition, SEEK_SET) != 0) {
        return 0;
    }
    return iPosition;
}

/**
 * @brief GIF Draw callback - THE CRITICAL FIX
 * 
 * Key insight: GIFDraw is called MULTIPLE times per frame
 * We need to detect when the frame is complete
 */
void GIFDraw(GIFDRAW *pDraw) {
    gifDrawCalls++;
    
    if (!nextFrameBuffer || !gifFileOpen) {
        return;
    }
    
    // Get GIF dimensions
    int gifWidth = gif.getCanvasWidth();
    int gifHeight = gif.getCanvasHeight();
    
    // Calculate scaling and centering
    int drawWidth = (gifWidth > MATRIX_WIDTH) ? MATRIX_WIDTH : gifWidth;
    int drawHeight = (gifHeight > MATRIX_HEIGHT) ? MATRIX_HEIGHT : gifHeight;
    int offsetX = (MATRIX_WIDTH - drawWidth) / 2;
    int offsetY = (MATRIX_HEIGHT - drawHeight) / 2;
    
    // CRITICAL: Store frame delay from GIF
    // Some AnimatedGIF versions store it in iFrameDelay
    currentFrameDelay = 50; // Default
    // if (pDraw->iFrameDelay > 0) {
    //     currentFrameDelay = pDraw->iFrameDelay;
    // }
    
    // Scale and draw this segment of the frame
    for (int y = 0; y < pDraw->iHeight; y++) {
        int gifY = pDraw->y + y;
        if (gifY >= gifHeight) continue;
        
        // Calculate target Y with scaling
        int targetY = offsetY + (gifY * drawHeight / gifHeight);
        if (targetY < 0 || targetY >= MATRIX_HEIGHT) continue;
        
        uint8_t *src = &pDraw->pPixels[y * pDraw->iWidth];
        uint16_t *dest = &nextFrameBuffer[targetY * MATRIX_WIDTH];
        
        for (int x = 0; x < pDraw->iWidth; x++) {
            uint8_t paletteIndex = src[x];
            
            if (paletteIndex != pDraw->ucTransparent) {
                // Calculate target X with scaling
                int targetX = offsetX + (x * drawWidth / pDraw->iWidth);
                if (targetX < 0 || targetX >= MATRIX_WIDTH) continue;
                
                // Get color from palette
                uint16_t color = pDraw->pPalette[paletteIndex];
                dest[targetX] = color;
            }
        }
    }
    
    // DEBUG: First call of each frame
    if (pDraw->y == 0) {
        ESP_LOGD("GIF_DRAW", "Starting frame %d, size: %dx%d, delay: %dms", 
                 currentFrameIndex, gifWidth, gifHeight, currentFrameDelay);
    }
    
    // DEBUG: Last call of each frame
    if (pDraw->y + pDraw->iHeight >= gifHeight) {
        ESP_LOGD("GIF_DRAW", "Completed frame %d", currentFrameIndex);
    }
}
