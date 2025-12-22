/*
 * ESP32-S3 HUB75 GIF Player with Proper Synchronization
 * 
 * Key fixes:
 * 1. Proper double buffering for GIF frames
 * 2. Synchronization between GIF decoding and DMA buffer swapping
 * 3. Fixed frame timing and memory management
 */

/* ==================== CONFIGURATION ==================== */
// --- HUB75 LED Matrix Configuration ---
#define MATRIX_WIDTH 64         // Width of your panel in pixels
#define MATRIX_HEIGHT 64        // Height of your panel in pixels (use 64 for 64x64 panel)
#define MATRIX_CHAIN_LENGTH 1   // Number of chained panels (horizontal)

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

// --- SD Card (SPI) Pin Mapping ---
// Using default HSPI pins for Waveshare ESP32-S3-Pico
#define SD_MISO GPIO_NUM_10
#define SD_MOSI GPIO_NUM_9
#define SD_SCK  GPIO_NUM_8
#define SD_CS   GPIO_NUM_7               // Chip Select pin

// --- GIF Playback Configuration ---
// #define GIF_FILENAME "/gifs/gifs/cartoon.gif" // Path to GIF file on SD card (mounted at /gifs)
#define GIF_FILENAME "/gifs/gifs/Revision.gif" // Path to GIF file on SD card (mounted at /gifs)
#define DEFAULT_BRIGHTNESS 250    // Brightness level (0-255)
#define MIN_FRAME_DELAY_MS 20    // Minimum delay between GIF frames (prevents excessive speed)
#define MAX_FRAME_DELAY_MS 200

// SD Card mount configuration
#define MOUNT_POINT "/gifs"
#define MAX_FILES 3

// --- Double Buffering ---
#define NUM_BUFFERS 2

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
#include "SD.h"                 // FatFs directory operations
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

// // Double buffering
// uint16_t *gifBuffers[NUM_BUFFERS] = {nullptr, nullptr};
// volatile int currentBuffer = 1;
// volatile int displayBuffer = 0;

// Synchronization
// SemaphoreHandle_t bufferMutex = nullptr;
// QueueHandle_t frameQueue = nullptr;
// QueueHandle_t retFrameQueue = nullptr;

// GIF state
volatile bool gifPlaying = false;
volatile bool gifReady = false;
// volatile int32_t currentFrameDelay = MIN_FRAME_DELAY_MS;
// volatile bool frameDecoded = false;

// Debug counters
// uint32_t framesDecoded = 0;
// uint32_t framesDisplayed = 0;

/* ==================== FUNCTION PROTOTYPES ==================== */
bool setup_matrix_panel();
bool setup_sd_card();
static void GIFDraw(GIFDRAW *pDraw);
static void* GIFOpenFile(const char *fname, int32_t *pSize);
static void GIFCloseFile(void *pHandle);
static int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen);
static int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition);
// void gif_decoder_task(void *pvParameters);
// void display_task(void *pvParameters);
void displayPlay(void *pvParameters);
void list_files(const char *path);
int32_t get_gif_frame_delay();  // Helper function to get frame delay

/* ==================== SETUP FUNCTIONS ==================== */
void run_sync_test() {
    ESP_LOGI("TEST", "Running synchronization test...");
    
    // Test 1: Sequential colors
    uint16_t colors[] = {
        dma_display->color565(255, 0, 0),    // Red
        dma_display->color565(0, 255, 0),    // Green
        dma_display->color565(0, 0, 255),    // Blue
        dma_display->color565(255, 255, 0),  // Yellow
        dma_display->color565(255, 0, 255),  // Magenta
        dma_display->color565(0, 255, 255),  // Cyan
    };
    
    for (int i = 0; i < 6; i++) {
        dma_display->fillScreen(colors[i]);
        dma_display->flipDMABuffer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    // Test 2: Moving dot
    for (int frame = 0; frame < 100; frame++) {
        dma_display->clearScreen();
        int x = frame % MATRIX_WIDTH;
        int y = (frame * 2) % MATRIX_HEIGHT;
        
        // Draw 3x3 dot
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT) {
                    dma_display->drawPixel(px, py, dma_display->color565(255, 255, 255));
                }
            }
        }
        
        dma_display->flipDMABuffer();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    dma_display->clearScreen();
    dma_display->flipDMABuffer();
    
    ESP_LOGI("TEST", "Synchronization test complete!");
}

extern "C" void app_main(void)
{
    ESP_LOGI("MAIN", "=== ESP32-S3 HUB75 GIF Player ===");
    ESP_LOGI("MAIN", "Free heap: %d bytes", esp_get_free_heap_size());
    
    // // Create synchronization primitives
    // bufferMutex = xSemaphoreCreateMutex();
    // frameQueue = xQueueCreate(2, sizeof(uint16_t*));
    
    // 2. Initialize SD card
    if (!setup_sd_card()) {
        ESP_LOGE("MAIN", "SD card initialization failed!");
        vTaskDelay(portMAX_DELAY);
    }
    
	// List files for debugging
    ESP_LOGI("MAIN", "Listing files on SD card:");
    list_files(MOUNT_POINT);

    // 1. Initialize matrix panel
    /* ==========   Allocate pointer and setup for MatrixPanel_I2S_DMA
                    Also start MatrixPanel_I2S_DMA with begin() function..
                    Set Briteness
                    Clears screen. =================  */
    if (!setup_matrix_panel()) {
        ESP_LOGE("MAIN", "Matrix initialization failed!");
        vTaskDelay(portMAX_DELAY);
    }

	// run_sync_test();
    
    

    // ////////// ======== Buffers allocation already done in MatrixPanel_I2S_DMA::begin()
    // // 3. Allocate double buffers in DMA-capable memory
    // for (int i = 0; i < NUM_BUFFERS; i++) {
    //     gifBuffers[i] = (uint16_t*)heap_caps_malloc(
    //         MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t), 
    //         MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        
    //     if (!gifBuffers[i]) {
    //         ESP_LOGE("MAIN", "Failed to allocate buffer %d", i);
    //         vTaskDelay(portMAX_DELAY);
    //     }
    //     memset(gifBuffers[i], 0, MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    // }
    // ESP_LOGI("MAIN", "Buffers allocated: %d bytes each", 
    //          MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    
	

    // 4. Initialize GIF decoder
    gif.begin(LITTLE_ENDIAN_PIXELS);
    
    // 5. Create tasks
    // xTaskCreatePinnedToCore(gif_decoder_task, "GIF Decoder", 8192, NULL, 3, NULL, 0);
    // xTaskCreatePinnedToCore(display_task, "Display", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(displayPlay, "DisplayPlay", 4096, NULL, 2, NULL, 0);
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // // Try to open GIF file
    // ESP_LOGI("MAIN", "Opening GIF: %s", GIF_FILENAME);
    // if (!gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
    //     ESP_LOGE("MAIN", "Failed to open GIF file");
    //     list_files(MOUNT_POINT);
    //     vTaskDelay(portMAX_DELAY);
    // }
    
    gifPlaying = true;
    ESP_LOGI("MAIN", "GIF opened: %dx%d", gif.getCanvasWidth(), gif.getCanvasHeight());
    

    // 6. Main monitoring task
    uint32_t lastPrint = esp_timer_get_time() / 1000;
    
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // uint32_t now = esp_timer_get_time() / 1000;
        // if (now - lastPrint >= 5000) {
        //     ESP_LOGI("STATS", "Decoded: %lu, Displayed: %lu, Heap: %d", 
        //              framesDecoded, framesDisplayed, esp_get_free_heap_size());
        //     lastPrint = now;
        // }
        
        // Check if GIF needs restart
        if (!gifPlaying) {
            ESP_LOGI("MAIN", "Restarting GIF...");
            gif.reset();
           
            if (gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
                gifPlaying = true;
            } else {
                ESP_LOGE("MAIN", "Failed to reopen GIF");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }
        // dma_display->flipDMABuffer();
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief Display task - handles DMA buffer swapping
 */
void displayPlay(void *pvParameters)
{
    ESP_LOGI("DISPLAY", "Display task started");
    uint32_t framesDisplayed = 0;
    int frameDelay = 100;
    uint16_t *bufferToDisplay = nullptr;
    
    // Try to open GIF file
    ESP_LOGI("MAIN", "Opening GIF: %s", GIF_FILENAME);
    if (!gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
        ESP_LOGE("MAIN", "Failed to open GIF file");
        list_files(MOUNT_POINT);
        vTaskDelay(portMAX_DELAY);
    }

    while (1) {
        if(!gifPlaying) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            framesDisplayed = 0;
            ESP_LOGI("DISPLAY", "GIF not playing, waiting...");
            continue;
        }
        // get next frame buffer to display using gif.playFrame function
        gif.playFrame(true, &frameDelay); // Non-blocking call to prepare next frame
        // Draw to display
        ESP_LOGI("DISPLAY", "frameDelay %lu", frameDelay / portTICK_PERIOD_MS);
        framesDisplayed++;
        // dma_display->drawRect(0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, 
        //                  dma_display->color565(255, 255, 255));
        dma_display->flipDMABuffer();
        // vTaskDelay(frameDelay / portTICK_PERIOD_MS);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        // Debug: check first pixel
        if (framesDisplayed % 30 == 0) {
            ESP_LOGD("DISPLAY", "Displayed frame %lu", framesDisplayed);
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
    
    // mxconfig.gpio.e = E_PIN;
    mxconfig.double_buff = true;
    // mxconfig.clkphase = false;
    // mxconfig.driver = HUB75_I2S_CFG::SHIFTREG;
    // mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_8M;
    // mxconfig.min_refresh_rate = 60;
    // mxconfig.dma_buf_count = 4;
    // mxconfig.dma_buf_len = 64;

    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    
    if (!dma_display->begin()) {
        ESP_LOGE("MATRIX", "Matrix begin() failed!");
        return false;
    }

    dma_display->setBrightness8(DEFAULT_BRIGHTNESS);
    dma_display->clearScreen();
    
    // Test pattern
    dma_display->drawRect(0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, 
                         dma_display->color565(255, 255, 255));
    dma_display->flipDMABuffer();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    dma_display->fillScreen(dma_display->color565(255, 0, 0));
    dma_display->flipDMABuffer();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    dma_display->fillScreen(dma_display->color565(0, 255, 0));
    dma_display->flipDMABuffer();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    dma_display->fillScreen(dma_display->color565(0, 0, 255));
    dma_display->flipDMABuffer();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
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

void list_files(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE("SD", "Failed to open dir: %s", path);
        return;
    }
    
    struct dirent *entry;
    ESP_LOGI("SD", "Files in %s:", path);
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI("SD", "  %s", entry->d_name);
    }
    closedir(dir);
}

/* ==================== GIF DECODER CALLBACKS ==================== */
static void* GIFOpenFile(const char *fname, int32_t *pSize) {
    ESP_LOGI("GIFOpenFile", "Opening: %s", fname);
    // //log_d("GIFOpenFile( %s )\n", fname );
    // FSGifFile = M5STACK_SD.open(fname);
    // if (FSGifFile) {
    //     *pSize = FSGifFile.size();
    //     return (void *)&FSGifFile;
    // }
    // list_files(MOUNT_POINT);
    // ESP_LOGE("GIFOpenFile", "Failed to open: %s", fname);
    // return NULL;
    FILE *file = fopen(fname, "rb");
    if (!file) {
        ESP_LOGE("GIFOpenFile", "Failed to open: %s", fname);
        return NULL;
    }
    
    fseek(file, 0, SEEK_END);
    *pSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    ESP_LOGI("GIFOpenFile", "Size: %ld bytes", *pSize);
    return (void *)file;
}

static void GIFCloseFile(void *pHandle) {
    File *f = static_cast<File *>(pHandle);
    if (f != NULL)
        f->close();
    ESP_LOGE("GIFCloseFile", "File closed");
}

static int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen) {
    // int32_t iBytesRead;
    // iBytesRead = iLen;
    // Serial.print("Read File Length count : ");
    // Serial.println(iBytesRead);
    // File *f = static_cast<File *>(pFile->fHandle);
    // // Note: If you read a file all the way to the last byte, seek() stops working
    // if ((pFile->iSize - pFile->iPos) < iLen)
    //     iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
    // if (iBytesRead <= 0)
    //     return 0;
    // iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
    // pFile->iPos = f->position();
    // return iBytesRead;
    static int32_t totalFrameRead = 0;
    FILE *file = (FILE *)pHandle->fHandle;
    if (!file) return 0;
    
    size_t bytesRead = fread(pBuf, 1, iLen, file);
    // totalFrameRead++;   
    ESP_LOGI("GIFReadFile", "From frame :%d Read %ld bytes [%d]", totalFrameRead++, bytesRead, iLen);
    return (int32_t)bytesRead;
}

static int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition) {
    // int i = micros();
    // File *f = static_cast<File *>(pFile->fHandle);
    // f->seek(iPosition);
    // pFile->iPos = (int32_t)f->position();
    // i = micros() - i;
    // //log_d("Seek time = %d us\n", i);
    // return pFile->iPos;
    FILE *file = (FILE *)pHandle->fHandle;
    if (!file) return 0;
    
    if (fseek(file, iPosition, SEEK_SET) != 0) {
        return 0;
    }
    ESP_LOGI("GIFSeekFile", "Seeked to position %d", iPosition);
    return iPosition;
}

/**
 * @brief GIF Draw callback - called for each decoded line
 * 
 * This writes to the current buffer in the double buffer array
 */
static void GIFDraw(GIFDRAW *pDraw) {
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;
    ESP_LOGI("GIFDraw", "Drawing line %d of frame at (%d,%d)", pDraw->y, pDraw->iX, pDraw->iY);
    iWidth = pDraw->iWidth;
    if (iWidth > MATRIX_WIDTH)
        iWidth = MATRIX_WIDTH;
    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line

    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) {// restore to background color
        for (x=0; x<iWidth; x++) {
        if (s[x] == pDraw->ucTransparent)
            s[x] = pDraw->ucBackground;
        }
        pDraw->ucHasTransparency = 0;
    }
    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) { // if transparency used
        uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
        int x, iCount;
        pEnd = s + iWidth;
        x = 0;
        iCount = 0; // count non-transparent pixels
        while(x < iWidth) {
        c = ucTransparent-1;
        d = usTemp;
        while (c != ucTransparent && s < pEnd) {
            c = *s++;
            if (c == ucTransparent) { // done, stop
            s--; // back up to treat it like transparent
            } else { // opaque
                *d++ = usPalette[c];
                iCount++;
            }
        } // while looking for opaque pixels
        if (iCount) { // any opaque pixels?
            for(int xOffset = 0; xOffset < iCount; xOffset++ ){
                dma_display->drawPixel(x + xOffset, y, usTemp[xOffset]); // 565 Color Format
            }
            x += iCount;
            iCount = 0;
        }
        // no, look for a run of transparent pixels
        c = ucTransparent;
        while (c == ucTransparent && s < pEnd) {
            c = *s++;
            if (c == ucTransparent)
                iCount++;
            else
                s--;
        }
        if (iCount) {
            x += iCount; // skip these
            iCount = 0;
        }
        }
    } else {
        s = pDraw->pPixels;
        // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
        for (x=0; x<iWidth; x++)
        dma_display->drawPixel(x, y, usPalette[*s++]); // color 565
        /*
        usTemp[x] = usPalette[*s++];

        for (x=0; x<pDraw->iWidth; x++) {
            dma_display->drawPixel(x, y, usTemp[*s++]); // color 565
        } */     
    }
}


// /*
//  * ESP32-S3 HUB75 GIF Player with Proper Synchronization
//  * 
//  * Key fixes:
//  * 1. Proper double buffering for GIF frames
//  * 2. Synchronization between GIF decoding and DMA buffer swapping
//  * 3. Fixed frame timing and memory management
//  */

// /* ==================== CONFIGURATION ==================== */
// // --- HUB75 LED Matrix Configuration ---
// #define MATRIX_WIDTH 64         // Width of your panel in pixels
// #define MATRIX_HEIGHT 64        // Height of your panel in pixels (use 64 for 64x64 panel)
// #define MATRIX_CHAIN_LENGTH 1   // Number of chained panels (horizontal)

// // --- HUB75 Panel Pin Mapping for Waveshare ESP32-S3-Pico ---
// // IMPORTANT: For a 64x64 panel, you MUST define and connect the E_PIN.
// #define R1_PIN GPIO_NUM_11
// #define G1_PIN GPIO_NUM_12
// #define B1_PIN GPIO_NUM_13
// #define R2_PIN GPIO_NUM_14
// #define G2_PIN GPIO_NUM_15
// #define B2_PIN GPIO_NUM_16

// #define A_PIN GPIO_NUM_17
// #define B_PIN GPIO_NUM_18
// #define C_PIN GPIO_NUM_33
// #define D_PIN GPIO_NUM_34
// #define E_PIN GPIO_NUM_35                 // Set to a valid GPIO (e.g., 12) for 64x64 1/32 scan panels

// #define LAT_PIN GPIO_NUM_37
// #define OE_PIN GPIO_NUM_38
// #define CLK_PIN GPIO_NUM_36
// // -----------------------------
// // HUB75 RGB Matrix Panel POWER pin
// // -----------------------------
// #define HUB75_POWER_EN         GPIO_NUM_42

// // --- SD Card (SPI) Pin Mapping ---
// // Using default HSPI pins for Waveshare ESP32-S3-Pico
// #define SD_MISO GPIO_NUM_10
// #define SD_MOSI GPIO_NUM_9
// #define SD_SCK  GPIO_NUM_8
// #define SD_CS   GPIO_NUM_7               // Chip Select pin

// // --- GIF Playback Configuration ---
// #define GIF_FILENAME "/gifs/gifs/cartoon.gif" // Path to GIF file on SD card (mounted at /gifs)
// // #define GIF_FILENAME "/gifs/gifs/Revision.gif" // Path to GIF file on SD card (mounted at /gifs)
// #define DEFAULT_BRIGHTNESS 250    // Brightness level (0-255)
// #define MIN_FRAME_DELAY_MS 20    // Minimum delay between GIF frames (prevents excessive speed)
// #define MAX_FRAME_DELAY_MS 200

// // SD Card mount configuration
// #define MOUNT_POINT "/gifs"
// #define MAX_FILES 3

// // --- Double Buffering ---
// #define NUM_BUFFERS 2

// /* ==================== LIBRARY INCLUDES ==================== */
// #include <stdio.h>
// #include <string.h>
// #include <sys/unistd.h>
// #include <sys/stat.h>
// #include <dirent.h>
// #include <vector>
// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_vfs_fat.h"
// #include "driver/sdmmc_host.h"
// #include "driver/sdspi_host.h"
// #include "sdmmc_cmd.h"
// #include "driver/spi_common.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "freertos/queue.h"
// #include "esp_heap_caps.h"
// #include "esp_timer.h"

// // Third-party libraries
// #include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
// #include "AnimatedGIF.h"
// #include "ff.h"                 // FatFs directory operations
// #include <stdio.h>
// #include <string.h>
// #include <errno.h>

// /* ==================== GLOBAL OBJECTS ==================== */
// // Matrix Panel Object
// MatrixPanel_I2S_DMA *dma_display = nullptr;

// // AnimatedGIF Decoder
// AnimatedGIF gif;
// sdmmc_card_t *card = NULL;
// static const char *TAG = "main";

// // Double buffering
// uint16_t *gifBuffers[NUM_BUFFERS] = {nullptr, nullptr};
// volatile int currentBuffer = 1;
// volatile int displayBuffer = 0;

// // Synchronization
// SemaphoreHandle_t bufferMutex = nullptr;
// QueueHandle_t frameQueue = nullptr;
// QueueHandle_t retFrameQueue = nullptr;

// // GIF state
// volatile bool gifPlaying = false;
// volatile bool gifReady = false;
// volatile int32_t currentFrameDelay = MIN_FRAME_DELAY_MS;
// volatile bool frameDecoded = false;

// // Debug counters
// uint32_t framesDecoded = 0;
// uint32_t framesDisplayed = 0;

// /* ==================== FUNCTION PROTOTYPES ==================== */
// bool setup_matrix_panel();
// bool setup_sd_card();
// void GIFDraw(GIFDRAW *pDraw);
// void* GIFOpenFile(const char *fname, int32_t *pSize);
// void GIFCloseFile(void *pHandle);
// int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen);
// int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition);
// void gif_decoder_task(void *pvParameters);
// void display_task(void *pvParameters);
// void displayPlay(void *pvParameters);
// void list_files(const char *path);
// int32_t get_gif_frame_delay();  // Helper function to get frame delay

// /* ==================== SETUP FUNCTIONS ==================== */
// void run_sync_test() {
//     ESP_LOGI("TEST", "Running synchronization test...");
    
//     // Test 1: Sequential colors
//     uint16_t colors[] = {
//         dma_display->color565(255, 0, 0),    // Red
//         dma_display->color565(0, 255, 0),    // Green
//         dma_display->color565(0, 0, 255),    // Blue
//         dma_display->color565(255, 255, 0),  // Yellow
//         dma_display->color565(255, 0, 255),  // Magenta
//         dma_display->color565(0, 255, 255),  // Cyan
//     };
    
//     for (int i = 0; i < 6; i++) {
//         dma_display->fillScreen(colors[i]);
//         dma_display->flipDMABuffer();
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
    
//     // Test 2: Moving dot
//     for (int frame = 0; frame < 100; frame++) {
//         dma_display->clearScreen();
//         int x = frame % MATRIX_WIDTH;
//         int y = (frame * 2) % MATRIX_HEIGHT;
        
//         // Draw 3x3 dot
//         for (int dy = -1; dy <= 1; dy++) {
//             for (int dx = -1; dx <= 1; dx++) {
//                 int px = x + dx;
//                 int py = y + dy;
//                 if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT) {
//                     dma_display->drawPixel(px, py, dma_display->color565(255, 255, 255));
//                 }
//             }
//         }
        
//         dma_display->flipDMABuffer();
//         vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
    
//     dma_display->clearScreen();
//     dma_display->flipDMABuffer();
    
//     ESP_LOGI("TEST", "Synchronization test complete!");
// }

// extern "C" void app_main(void)
// {
//     ESP_LOGI("MAIN", "=== ESP32-S3 HUB75 GIF Player ===");
//     ESP_LOGI("MAIN", "Free heap: %d bytes", esp_get_free_heap_size());
    
//     // Create synchronization primitives
//     bufferMutex = xSemaphoreCreateMutex();
//     frameQueue = xQueueCreate(2, sizeof(uint16_t*));
    

//     // 1. Initialize matrix panel
//     /* ==========   Allocate pointer and setup for MatrixPanel_I2S_DMA
//                     Also start MatrixPanel_I2S_DMA with begin() function..
//                     Set Briteness
//                     Clears screen. =================  */
//     if (!setup_matrix_panel()) {
//         ESP_LOGE("MAIN", "Matrix initialization failed!");
//         vTaskDelay(portMAX_DELAY);
//     }

// 	// run_sync_test();
    
//     // 2. Initialize SD card
//     if (!setup_sd_card()) {
//         ESP_LOGE("MAIN", "SD card initialization failed!");
//         vTaskDelay(portMAX_DELAY);
//     }
    
// 	// List files for debugging
//     ESP_LOGI("MAIN", "Listing files on SD card:");
//     list_files(MOUNT_POINT);

//     ////////// ======== Buffers allocation already done in MatrixPanel_I2S_DMA::begin()
//     // 3. Allocate double buffers in DMA-capable memory
//     for (int i = 0; i < NUM_BUFFERS; i++) {
//         gifBuffers[i] = (uint16_t*)heap_caps_malloc(
//             MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t), 
//             MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        
//         if (!gifBuffers[i]) {
//             ESP_LOGE("MAIN", "Failed to allocate buffer %d", i);
//             vTaskDelay(portMAX_DELAY);
//         }
//         memset(gifBuffers[i], 0, MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
//     }
//     ESP_LOGI("MAIN", "Buffers allocated: %d bytes each", 
//              MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
    
	

//     // 4. Initialize GIF decoder
//     gif.begin(LITTLE_ENDIAN_PIXELS);
    
//     // Try to open GIF file
//     ESP_LOGI("MAIN", "Opening GIF: %s", GIF_FILENAME);
//     if (!gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
//         ESP_LOGE("MAIN", "Failed to open GIF file");
//         list_files("/sdcard");
//         vTaskDelay(portMAX_DELAY);
//     }
    
//     gifPlaying = true;
//     ESP_LOGI("MAIN", "GIF opened: %dx%d", gif.getCanvasWidth(), gif.getCanvasHeight());
    
//     // 5. Create tasks
//     xTaskCreatePinnedToCore(gif_decoder_task, "GIF Decoder", 8192, NULL, 3, NULL, 0);
//     xTaskCreatePinnedToCore(display_task, "Display", 4096, NULL, 2, NULL, 0);
//     // xTaskCreatePinnedToCore(displayPlay, "DisplayPlay", 4096, NULL, 2, NULL, 0);
    
//     // 6. Main monitoring task
//     uint32_t lastPrint = esp_timer_get_time() / 1000;
    
//     while (1) {
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
        
//         uint32_t now = esp_timer_get_time() / 1000;
//         if (now - lastPrint >= 5000) {
//             ESP_LOGI("STATS", "Decoded: %lu, Displayed: %lu, Heap: %d", 
//                      framesDecoded, framesDisplayed, esp_get_free_heap_size());
//             lastPrint = now;
//         }
        
//         // Check if GIF needs restart
//         if (!gifPlaying) {
//             ESP_LOGI("MAIN", "Restarting GIF...");
//             gif.reset();
           
//             if (gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
//                 gifPlaying = true;
//             } else {
//                 ESP_LOGE("MAIN", "Failed to reopen GIF");
//                 vTaskDelay(2000 / portTICK_PERIOD_MS);
//             }
//         }
//         // dma_display->flipDMABuffer();
//         // vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

// /**
//  * @brief GIF decoder task - runs separately from display
//  */
// void gif_decoder_task(void *pvParameters)
// {
//     ESP_LOGI("DECODER", "GIF decoder task started");
//     int frameDelay = MIN_FRAME_DELAY_MS;
//     while (1) {
//         if (gifPlaying) {
//             frameDelay = MIN_FRAME_DELAY_MS;
//             // Wait for buffer to be available
//             if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
//                 ESP_LOGI("DECODER", "Semaphore taken for buffer %d", currentBuffer);
//                 // Clear current buffer
//                 memset(gifBuffers[currentBuffer], 0, 
//                        MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));
                
//                 // Decode one frame
//                 if (gif.playFrame(false, NULL, gifBuffers[currentBuffer])) {
//                     framesDecoded++;
//                     frameDecoded = true;
                    
//                     // Calculate frame delay
//                     currentFrameDelay = MIN_FRAME_DELAY_MS;
                    
//                     // Send buffer to disframeDelayplay task
//                     uint16_t *buffer = gifBuffers[currentBuffer];
//                     // uint16_t buffer = 1; // Use buffer index instead of pointer
//                     ESP_LOGI("DECODER", "Sending buffer with address %d to display", buffer);
//                     xQueueSend(frameQueue, &buffer, portMAX_DELAY);
                    
//                     // Switch to next buffer
//                     currentBuffer = (currentBuffer + 1) % NUM_BUFFERS;
//                 } else {
//                     // End of GIF
//                     ESP_LOGE("DECODER", "GIF ended, resetting");
//                     gifPlaying = false;
//                 }


//                 for (int i = 0; i < MATRIX_WIDTH * MATRIX_HEIGHT; i += MATRIX_WIDTH) {
//                     ESP_LOGI("", "%04X", i / MATRIX_WIDTH, gifBuffers[currentBuffer][i]);
//                 }
                     
//                 xSemaphoreGive(bufferMutex);
//                 ESP_LOGI("DECODER", "Semaphore release for buffer %d", currentBuffer);
                
//             }
//             // Delay between frames
//             // ESP_LOGI("DECODER", "Frame decoded, delay %d ms", currentFrameDelay);
//             // vTaskDelay(currentFrameDelay / portTICK_PERIOD_MS);
//             ESP_LOGI("DECODER", "Frame decoded, delay %d ms, for testing at frame %lu", frameDelay, framesDecoded);
//             vTaskDelay(frameDelay);
//         } else {

//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         }
//     }
// }
// /**
//  * @brief Display task - handles DMA buffer swapping
//  */
// void display_task(void *pvParameters)
// {
//     ESP_LOGI("DISPLAY", "Display task started");
    
//     uint16_t *bufferToDisplay = nullptr;
    
//     while (1) {
//         // Wait for a frame to be ready
//         if (xQueueReceive(frameQueue, &bufferToDisplay, portMAX_DELAY) == pdTRUE) {
//             if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
//                 ESP_LOGI("Display", "Semaphore taken for bufferToDisplay %d", bufferToDisplay);
                
//                 if (bufferToDisplay) {
//                     // // // Draw to display
//                     dma_display->drawRGBBitmap(0, 0, bufferToDisplay, 
//                                             MATRIX_WIDTH, MATRIX_HEIGHT);
                    
//                     // Swap DMA buffers
//                     dma_display->flipDMABuffer();
                    
//                     framesDisplayed++;
                    
//                     // Debug: check first pixel
//                     if (framesDisplayed % 30 == 0) {
//                         ESP_LOGD("DISPLAY", "Displayed frame %lu", framesDisplayed);
//                     }

//                     // // Switch to next buffer
//                     displayBuffer = (displayBuffer + 1) % NUM_BUFFERS;
//                 }
//             }
//             xSemaphoreGive(bufferMutex);
//             ESP_LOGI("Display", "Semaphore release for buffer %d", displayBuffer);
                
//         }
//     }
// }

// /**
//  * @brief Display task - handles DMA buffer swapping
//  */
// void displayPlay(void *pvParameters)
// {
//     ESP_LOGI("DISPLAY", "Display task started");
    
//     uint16_t *bufferToDisplay = nullptr;
//     int frameDelay = MIN_FRAME_DELAY_MS;
//     uint8_t colorCntr = 0;
//     uint16_t *gifBuffers = (uint16_t*)heap_caps_malloc(
//             MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t), 
//             MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
//     while (1) {
//         // get next frame buffer to display using gif.playFrame function
//         gif.playFrame(true, &frameDelay, gifBuffers); // Non-blocking call to prepare next frame
//         // bufferToDisplay = gifBuffers[displayBuffer];
//         // // //get next frame buffer to display using gif.playFrame function 
        
//         // uint16_t colors[] = {
//         // dma_display->color565(255, 0, 0),    // Red
//         // dma_display->color565(0, 255, 0),    // Green
//         // dma_display->color565(0, 0, 255),    // Blue
//         // dma_display->color565(255, 255, 0),  // Yellow
//         // dma_display->color565(255, 0, 255),  // Magenta
//         // dma_display->color565(0, 255, 255),  // Cyan
//         // };
    
//         // dma_display->fillScreen(colors[colorCntr++%6]);

//         dma_display->drawRGBBitmap(0, 0, gifBuffers, 
//                                 MATRIX_WIDTH, MATRIX_HEIGHT);
        
//         // Swap DMA buffers
//         dma_display->flipDMABuffer();
        
//         framesDisplayed++;
//         vTaskDelay(2500 );
        
//         // Debug: check first pixel
//         if (framesDisplayed % 30 == 0) {
//             ESP_LOGD("DISPLAY", "Displayed frame %lu", framesDisplayed);
//         }

//     }
// }


// /**
//  * @brief Initialize matrix panel
//  */
// bool setup_matrix_panel() {
//     ESP_LOGI("MATRIX", "Initializing HUB75 panel...");
    
//     HUB75_I2S_CFG::i2s_pins _pins = {
//         .r1 = R1_PIN, .g1 = G1_PIN, .b1 = B1_PIN,
//         .r2 = R2_PIN, .g2 = G2_PIN, .b2 = B2_PIN,
//         .a = A_PIN, .b = B_PIN, .c = C_PIN, .d = D_PIN, .e = E_PIN,
//         .lat = LAT_PIN, .oe = OE_PIN, .clk = CLK_PIN
//     };

//     HUB75_I2S_CFG mxconfig(
//         MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_CHAIN_LENGTH, _pins
//     );
    
//     // mxconfig.gpio.e = E_PIN;
//     mxconfig.double_buff = true;
//     // mxconfig.clkphase = false;
//     // mxconfig.driver = HUB75_I2S_CFG::SHIFTREG;
//     // mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_8M;
//     // mxconfig.min_refresh_rate = 60;
//     // mxconfig.dma_buf_count = 4;
//     // mxconfig.dma_buf_len = 64;

//     dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    
//     if (!dma_display->begin()) {
//         ESP_LOGE("MATRIX", "Matrix begin() failed!");
//         return false;
//     }

//     dma_display->setBrightness8(DEFAULT_BRIGHTNESS);
//     dma_display->clearScreen();
    
//     // Test pattern
//     dma_display->drawRect(0, 0, MATRIX_WIDTH, MATRIX_HEIGHT, 
//                          dma_display->color565(255, 255, 255));
//     dma_display->flipDMABuffer();
//     vTaskDelay(500 / portTICK_PERIOD_MS);
    
//     dma_display->fillScreen(dma_display->color565(255, 0, 0));
//     dma_display->flipDMABuffer();
//     vTaskDelay(500 / portTICK_PERIOD_MS);
    
//     dma_display->fillScreen(dma_display->color565(0, 255, 0));
//     dma_display->flipDMABuffer();
//     vTaskDelay(500 / portTICK_PERIOD_MS);
    
//     dma_display->fillScreen(dma_display->color565(0, 0, 255));
//     dma_display->flipDMABuffer();
//     vTaskDelay(500 / portTICK_PERIOD_MS);
    
//     dma_display->clearScreen();
//     dma_display->flipDMABuffer();
    
//     ESP_LOGI("MATRIX", "Panel initialized: %dx%d", MATRIX_WIDTH, MATRIX_HEIGHT);
//     return true;
// }

// /**
//  * @brief Initialize SD card
//  */
// bool setup_sd_card() {
//     ESP_LOGI("SD", "Initializing SD card...");

//     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
//         .format_if_mount_failed = false,
//         .max_files = 5,
//         .allocation_unit_size = 16 * 1024,
//         .disk_status_check_enable = false
//     };

//     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//     host.slot = (spi_host_device_t)SPI2_HOST;

//     spi_bus_config_t bus_cfg = {
//         .mosi_io_num = SD_MOSI,
//         .miso_io_num = SD_MISO,
//         .sclk_io_num = SD_SCK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4000,
//     };

//     esp_err_t ret = spi_bus_initialize((spi_host_device_t)SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
//     if (ret != ESP_OK) {
//         ESP_LOGE("SD", "SPI bus init failed: %s", esp_err_to_name(ret));
//         return false;
//     }

//     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//     slot_config.gpio_cs = (gpio_num_t)SD_CS;
//     slot_config.host_id = (spi_host_device_t)SPI2_HOST;

//     ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    
//     if (ret != ESP_OK) {
//         ESP_LOGE("SD", "Mount failed: %s", esp_err_to_name(ret));
//         return false;
//     }

//     sdmmc_card_print_info(stdout, card);
//     ESP_LOGI("SD", "SD card mounted at %s", MOUNT_POINT);
    
//     return true;
// }

// void list_files(const char *path) {
//     DIR *dir = opendir(path);
//     if (!dir) {
//         ESP_LOGE("SD", "Failed to open dir: %s", path);
//         return;
//     }
    
//     struct dirent *entry;
//     ESP_LOGI("SD", "Files in %s:", path);
//     while ((entry = readdir(dir)) != NULL) {
//         ESP_LOGI("SD", "  %s", entry->d_name);
//     }
//     closedir(dir);
// }

// /* ==================== GIF DECODER CALLBACKS ==================== */
// void* GIFOpenFile(const char *fname, int32_t *pSize) {
//     ESP_LOGI("GIFOpenFile", "Opening: %s", fname);
    
//     FILE *file = fopen(fname, "rb");
//     if (!file) {
//         ESP_LOGE("GIFOpenFile", "Failed to open: %s", fname);
//         return NULL;
//     }
    
//     fseek(file, 0, SEEK_END);
//     *pSize = ftell(file);
//     fseek(file, 0, SEEK_SET);
    
//     ESP_LOGI("GIFOpenFile", "Size: %ld bytes", *pSize);
//     return (void *)file;
// }

// void GIFCloseFile(void *pHandle) {
//     FILE *file = (FILE *)pHandle;
//     if (file) fclose(file);
//     ESP_LOGE("GIFCloseFile", "File closed");
// }

// int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen) {
//     static int32_t totalFrameRead = 0;
//     FILE *file = (FILE *)pHandle->fHandle;
//     if (!file) return 0;
    
//     size_t bytesRead = fread(pBuf, 1, iLen, file);
//     // totalFrameRead++;   
//     ESP_LOGI("GIFReadFile", "From frame :%d Read %ld bytes [%d]", totalFrameRead++, bytesRead, iLen);
//     return (int32_t)bytesRead;
// }

// int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition) {
//     FILE *file = (FILE *)pHandle->fHandle;
//     if (!file) return 0;
    
//     if (fseek(file, iPosition, SEEK_SET) != 0) {
//         return 0;
//     }
//     ESP_LOGI("GIFSeekFile", "Seeked to position %d", iPosition);
//     return iPosition;
// }

// /**
//  * @brief GIF Draw callback - called for each decoded line
//  * 
//  * This writes to the current buffer in the double buffer array
//  */
// void GIFDraw(GIFDRAW *pDraw) {
//     if (!gifPlaying || currentBuffer < 0 || currentBuffer >= NUM_BUFFERS) {
//         ESP_LOGE("GIFDraw", "GIF is not Playing or Invalid buffer state");
//         return;
//     }
//     ESP_LOGE("GIFDraw", "GIF called for line %d of height %d", pDraw->y, pDraw->iHeight);
//     uint16_t *currentGifBuffer = gifBuffers[currentBuffer];
//     if (!currentGifBuffer) return;
    
//     int gifWidth = gif.getCanvasWidth();
//     int gifHeight = gif.getCanvasHeight();
    
//     // Scaling factors
//     float scaleX = (float)gifWidth / MATRIX_WIDTH;
//     float scaleY = (float)gifHeight / MATRIX_HEIGHT;
//     int offsetX = 0;
//     int offsetY = 0;
    
//     if (gifWidth < MATRIX_WIDTH) {
//         offsetX = (MATRIX_WIDTH - gifWidth) / 2;
//     }
//     if (gifHeight < MATRIX_HEIGHT) {
//         offsetY = (MATRIX_HEIGHT - gifHeight) / 2;
//     }
    
//     // Draw this segment of the GIF
//     for (int y = 0; y < pDraw->iHeight; y++) {
//         int gifY = pDraw->y + y;
//         int matrixY = offsetY + (int)(gifY / scaleY);
        
//         if (matrixY < 0 || matrixY >= MATRIX_HEIGHT) continue;
        
//         uint8_t *src = &pDraw->pPixels[y * pDraw->iWidth];
//         uint16_t *dest = &currentGifBuffer[matrixY * MATRIX_WIDTH];
        
//         for (int x = 0; x < pDraw->iWidth; x++) {
//             uint8_t paletteIndex = src[x];
            
//             if (paletteIndex != pDraw->ucTransparent) {
//                 int matrixX = offsetX + (int)(x / scaleX);
                
//                 if (matrixX >= 0 && matrixX < MATRIX_WIDTH) {
//                     // Convert palette index to RGB565
//                     uint16_t color = pDraw->pPalette[paletteIndex];
//                     dest[matrixX] = color;
//                 }
//             }
//         }
//     }
//     ESP_LOGI("GIFDraw", " completed for line %d", pDraw->y);
// }
