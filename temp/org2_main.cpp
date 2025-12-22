/*
 * ESP32-S3 HUB75 Animated GIF Player
 * 
 * Description: Plays animated GIFs from a microSD card on a HUB75 RGB LED matrix.
 * Libraries: ESP32-HUB75-MatrixPanel-I2S-DMA, Adafruit_GFX, AnimatedGIF
 * SD Card: Uses ESP-IDF SDMMC driver with FAT filesystem
 * 
 * How to Use:
 * 1. Update the CONFIGURATION section below for your panel and setup.
 * 2. Format your microSD card as FAT16 or FAT32.
 * 3. Place your GIF file(s) in the root directory (e.g., "demo.gif").
 * 4. Connect the HUB75 panel and SD card to the ESP32-S3-Pico as described in the WIRING section.
 * 5. Compile and flash this project to your board using ESP-IDF.
 * 
 * Important Notes:
 * - The AnimatedGIF library does NOT support interlaced GIFs.
 * - GIFs are streamed from the SD card, not loaded fully into RAM.
 * - Power supply is critical. Use a robust 5V supply and consider adding capacitors to each panel.
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
#define GIF_FILENAME "/gifs/gifs/cartoon.gif" // Path to GIF file on SD card (mounted at /gifs)
#define DEFAULT_BRIGHTNESS 250    // Brightness level (0-255)
#define MIN_FRAME_DELAY_MS 20    // Minimum delay between GIF frames (prevents excessive speed)



/* ==================== LIBRARY INCLUDES ==================== */
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"  // Added for spi_host_device_t

// Third-party Arduino libraries (ensure these are in your components folder)
#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include "AnimatedGIF.h"
#include "ff.h"                 // FatFs directory operations
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>
/* ==================== GLOBAL OBJECTS ==================== */
// Matrix Panel Object
MatrixPanel_I2S_DMA *dma_display = nullptr;

// AnimatedGIF Decoder
AnimatedGIF gif;
// Frame buffer for scaling. Size for a single panel.
uint16_t gifFrameBuffer[MATRIX_WIDTH * MATRIX_HEIGHT];

// SD Card handle
sdmmc_card_t *card = NULL;
static const char *TAG = "SDCARD";

// GIF playback state
static bool gifPlaying = false;
static int32_t currentFrameDelay = MIN_FRAME_DELAY_MS;

/* ==================== FUNCTION PROTOTYPES ==================== */
bool setup_matrix_panel();
bool setup_sd_card();
void GIFDraw(GIFDRAW *pDraw);
void* GIFOpenFile(const char *fname, int32_t *pSize);
void GIFCloseFile(void *pHandle);
int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen);
int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition);
void list_files(const char *path);
int32_t get_gif_frame_delay();  // Helper function to get frame delay

/* ==================== SETUP FUNCTIONS ==================== */
extern "C" void app_main(void)
{
    ESP_LOGI("MAIN", "Starting ESP32-S3 HUB75 GIF Player");

    // 1. Initialize the LED Matrix Panel
    if (!setup_matrix_panel()) {
        ESP_LOGE("MAIN", "Matrix panel initialization failed. Halting.");
        vTaskDelay(portMAX_DELAY); // Stop here
    }
    ESP_LOGE("MAIN", "=======================================================");
    ESP_LOGI("MAIN", "Matrix panel initialization Complete...");
    ESP_LOGE("MAIN", "=======================================================");
        
    // Generate some initial graphics like a white boarder with moving lines.
    dma_display->drawRect(0, 0, MATRIX_WIDTH * MATRIX_CHAIN_LENGTH, MATRIX_HEIGHT, dma_display->color565(255, 255, 255));
    for (int i = 0; i < 10; i++) {
        dma_display->drawLine(0, 0, (MATRIX_WIDTH * MATRIX_CHAIN_LENGTH) - 1 - i * 5, (MATRIX_HEIGHT) - 1 - i * 5, dma_display->color565(0, 255, 0));
        dma_display->drawLine((MATRIX_WIDTH * MATRIX_CHAIN_LENGTH) - 1, 0, i * 5, (MATRIX_HEIGHT) - 1 - i * 5, dma_display->color565(0, 255, 0));
        dma_display->drawLine(0, (MATRIX_HEIGHT) - 1, (MATRIX_WIDTH * MATRIX_CHAIN_LENGTH) - 1 - i * 5, i * 5, dma_display->color565(0, 255, 0));
        dma_display->drawLine((MATRIX_WIDTH * MATRIX_CHAIN_LENGTH) - 1, (MATRIX_HEIGHT) - 1, i * 5, i * 5, dma_display->color565(0, 255, 0));
        dma_display->flipDMABuffer();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    dma_display->fillScreen(0); // Clear screen before GIF playback
    
    ESP_LOGI("MAIN", "Initial graphics drawn on matrix panel.");
    

    // 2. Initialize the SD Card
    if (!setup_sd_card()) {
        ESP_LOGE("MAIN", "SD card initialization failed. Halting.");
        vTaskDelay(portMAX_DELAY); // Stop here
    }

    // List files for debugging
    ESP_LOGI("MAIN", "Listing files on SD card:");
    list_files(MOUNT_POINT);

    // 3. Configure the GIF decoder with file I/O callbacks
    gif.begin(LITTLE_ENDIAN_PIXELS);
    
    // Try to open the GIF file
    ESP_LOGI("MAIN", "Attempting to open GIF file: %s", GIF_FILENAME);
    if (gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
        gifPlaying = true;
        ESP_LOGI("MAIN", "GIF opened successfully. Canvas size: %d x %d..", 
                 gif.getCanvasWidth(), gif.getCanvasHeight());
    } else {
        ESP_LOGE("MAIN", "Failed to open GIF file: %s", GIF_FILENAME);
        ESP_LOGI("MAIN", "Please ensure the file exists at the root of the SD card");
        vTaskDelay(portMAX_DELAY);
    }

    // 4. Main GIF Playback Loop
    uint32_t frameCount = 0;
    int32_t lastFrameTime = 0;
    
    while (1) {
        if (gifPlaying) {
            // Play a single frame. The GIFDraw callback will be triggered.
            if (!gif.playFrame(false, NULL)) {
                // End of GIF reached, restart from beginning
                ESP_LOGI("MAIN", "End of GIF reached. Restarting.");
                gif.reset();
                frameCount = 0;
                vTaskDelay(50 / portTICK_PERIOD_MS);
                continue;
            }
            
            frameCount++;
            
            // Get frame delay for this frame
            currentFrameDelay = get_gif_frame_delay();
            ESP_LOGI("MAIN", "Frame11 %lu displayed with delay %ldms", frameCount, currentFrameDelay);
            if (currentFrameDelay < MIN_FRAME_DELAY_MS) {
                currentFrameDelay = MIN_FRAME_DELAY_MS;
            }
            ESP_LOGI("MAIN", "Frame22 %lu displayed with delay %ldms", frameCount, currentFrameDelay);
            // Debug output every 30 frames
            if (frameCount % 30 == 0) {
                ESP_LOGI("MAIN", "Frame %lu, delay: %ldms", frameCount, currentFrameDelay);
            }
            
            // Wait for the duration specified in the GIF file before next frame
            vTaskDelay(currentFrameDelay / portTICK_PERIOD_MS);
        } else {
            // If GIF stopped (e.g., error), try to reopen
            ESP_LOGI("MAIN", "Attempting to reopen GIF.");
            if (gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
                gifPlaying = true;
                frameCount = 0;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}


/**
 * @brief Helper function to get frame delay from GIF
 * @return Frame delay in milliseconds
 */
int32_t get_gif_frame_delay() {
    // Different versions of AnimatedGIF library have different APIs
    // This is a workaround to handle the API differences
    static int32_t lastDelay = MIN_FRAME_DELAY_MS;
    GIFINFO info;
    if (gif.getInfo(&info) == 0) {
        ESP_LOGI("GIFDelay", "Got GIF info: FrameCount=%d, Duration=%dms, MaxDelay=%dms, MinDelay=%dms",
                 info.iFrameCount, info.iDuration, info.iMaxDelay, info.iMinDelay);
        return info.iMinDelay; // Use minimum delay as a fallback
    }
    
    // Try to get frame delay using various methods depending on library version
    // Some versions store it in a member variable, others have a getter
    // We'll use a reasonable default and update when we can
    return lastDelay;
}

/**
 * @brief Initializes the HUB75 LED Matrix Panel using DMA.
 * @return true if successful, false otherwise.
 */
bool setup_matrix_panel() {
    ESP_LOGI("MATRIX", "Initializing HUB75 panel, Start by turning on Display pin...");
    
    // Initialize power pin and OE pin (if you use them)
    // gpio_config_t cfg = {};
    // cfg.pin_bit_mask = (1ULL << HUB75_POWER_EN);
    // cfg.mode = GPIO_MODE_OUTPUT;
    // // cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    // // cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // // cfg.intr_type = GPIO_INTR_DISABLE;
    // gpio_config(&cfg);
    // gpio_set_level(HUB75_POWER_EN, 1); // enable power by default; change if your hardware differs
    
    // HUB75 panel configuration structure
    HUB75_I2S_CFG::i2s_pins _pins = {
        .r1 = R1_PIN, .g1 = G1_PIN, .b1 = B1_PIN,
        .r2 = R2_PIN, .g2 = G2_PIN, .b2 = B2_PIN,
        .a = A_PIN, .b = B_PIN, .c = C_PIN, .d = D_PIN, .e = E_PIN,
        .lat = LAT_PIN, .oe = OE_PIN, .clk = CLK_PIN
    };

    // Matrix options: width, height, chain length, pin mapping, color depth, double buffer
    HUB75_I2S_CFG mxconfig(
        MATRIX_WIDTH,            // Panel width
        MATRIX_HEIGHT,           // Panel height
        MATRIX_CHAIN_LENGTH,     // Number of chained panels
        _pins                    // Pin mapping
    );
    
    // --- Critical Configuration Parameters ---
    mxconfig.gpio.e = E_PIN;                     // REQUIRED for 64x64 1/32 scan panels
    mxconfig.double_buff = true;                 // Smooth animation, uses 2x memory
    mxconfig.clkphase = false;                   // Usually false for HUB75
    mxconfig.driver = HUB75_I2S_CFG::FM6124;   // Common driver type
    mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_8M;   // 10MHz clock, good for P3 panels
    mxconfig.min_refresh_rate = 60;             // Target refresh rate in Hz
    
    // Create the display object
    dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    
    if (!dma_display->begin()) {
        ESP_LOGE("MATRIX", "Matrix begin() failed. Check wiring and pins.");
        return false;
    }
    ESP_LOGI("MATRIX", "Matrix begin() Successful...");
        
    dma_display->setBrightness8(DEFAULT_BRIGHTNESS); // 0-255 scale
    dma_display->clearScreen();
    dma_display->fillScreen(255); // Start with black screen

    ESP_LOGI("MATRIX", "Panel initialized: %dx%d (chain: %d)", 
             MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_CHAIN_LENGTH);
    return true;
}

/**
 * @brief Initializes the SPI bus and mounts the SD card using ESP-IDF FATFS.
 * @return true if successful, false otherwise.
 */
bool setup_sd_card() {
    ESP_LOGI(TAG, "Initializing SD card...");

    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = MAX_FILES,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };

    // Configure the SPI bus - FIXED: Properly cast to spi_host_device_t
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = (spi_host_device_t)SPI2_HOST; // HSPI (SPI2) on ESP32-S3-Pico

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    // Initialize SPI bus - FIXED: Properly cast to spi_host_device_t
    ESP_LOGI(TAG, "Initializing SPI bus...");
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure SD SPI interface
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS;
    slot_config.host_id = (spi_host_device_t)SPI2_HOST;

    ESP_LOGI(TAG, "Mounting filesystem...");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", 
                     esp_err_to_name(ret));
        }
        return false;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD card mounted successfully at %s", MOUNT_POINT);
    
    return true;
}

/**
 * @brief Lists files in a directory (for debugging)
 */
void list_files(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", path);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  %s", entry->d_name);
    }
    closedir(dir);
}

/* ==================== GIF DECODER CALLBACKS ==================== */
/**
 * @brief Called by AnimatedGIF library to open a file.
 */
void* GIFOpenFile(const char *fname, int32_t *pSize) {
    ESP_LOGI("GIF", "Opening file: %s", fname);
    
    FILE *file = fopen(fname, "rb");
    if (!file) {
        ESP_LOGE("GIF", "Failed to open file: %s", fname);
        return NULL;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    *pSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    ESP_LOGI("GIF", "File opened successfully. Size: %ld bytes", *pSize);
    return (void *)file;
}

/**
 * @brief Called by AnimatedGIF library to close a file.
 */
void GIFCloseFile(void *pHandle) {
    FILE *file = (FILE *)pHandle;
    if (file) {
        fclose(file);
    }
}

/**
 * @brief Called by AnimatedGIF library to read data from file.
 */
int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen) {
    FILE *file = (FILE *)pHandle->fHandle;
    if (!file) return 0;
    
    size_t bytesRead = fread(pBuf, 1, iLen, file);
    ESP_LOGI("GIFRead", "Requested %d bytes, read %d bytes", iLen, (int32_t)bytesRead);
    pHandle->iPos += file->position();
    return (int32_t)bytesRead;
}

/**
 * @brief Called by AnimatedGIF library to seek in file.
 */
int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition) {
    FILE *file = (FILE *)pHandle->fHandle;
    if (!file) return 0;
    
    if (fseek(file, iPosition, SEEK_SET) != 0) {
        return 0;
    }
    return iPosition;
}

/**
 * @brief Callback for each decoded GIF frame. Handles scaling and drawing to matrix.
 * @param pDraw Pointer to GIFDRAW structure with frame data.
 */
void GIFDraw(GIFDRAW *pDraw) {
    if (!dma_display) return;
    ESP_LOGI("GIFDRAW:", "Called....");
    // Calculate scaling factors to fit GIF into matrix
    int gifWidth = gif.getCanvasWidth();
    int gifHeight = gif.getCanvasHeight();
    
    // Target drawing area (centered)
    int drawWidth = (gifWidth > MATRIX_WIDTH) ? MATRIX_WIDTH : gifWidth;
    int drawHeight = (gifHeight > MATRIX_HEIGHT) ? MATRIX_HEIGHT : gifHeight;
    int offsetX = (MATRIX_WIDTH - drawWidth) / 2;
    int offsetY = (MATRIX_HEIGHT - drawHeight) / 2;

    // Skip if offsets are invalid
    if (offsetX < 0) offsetX = 0;
    if (offsetY < 0) offsetY = 0;

    // Clear frame buffer to black
    memset(gifFrameBuffer, 0, sizeof(gifFrameBuffer));

    // FIXED: pDraw->pPalette is now uint16_t*, not a struct with r,g,b members
    // We'll extract colors directly from the palette
    for (int y = 0; y < pDraw->iHeight; y++) {
        // Calculate target Y position with scaling
        int sourceY = pDraw->y + y;
        int matrixY = offsetY + (sourceY * drawHeight / gifHeight);
        if (matrixY < 0 || matrixY >= MATRIX_HEIGHT) continue;

        uint8_t *src = &pDraw->pPixels[y * pDraw->iWidth];
        uint16_t *dest = &gifFrameBuffer[matrixY * MATRIX_WIDTH];

        for (int x = 0; x < pDraw->iWidth; x++) {
            // Calculate target X position with scaling
            int matrixX = offsetX + (x * drawWidth / pDraw->iWidth);
            if (matrixX < 0 || matrixX >= MATRIX_WIDTH) continue;

            uint8_t paletteIndex = src[x];
            if (paletteIndex != pDraw->ucTransparent && paletteIndex < 256) {
                // FIXED: pPalette is uint16_t array, not a struct
                // Use the palette color directly (already in RGB565 format)
                dest[matrixX] = pDraw->pPalette[paletteIndex];
            }
        }
    }

    // If this is the last line of the frame, push the buffer to the matrix
    if (pDraw->y + pDraw->iHeight >= gifHeight) {
        // Use Adafruit_GFX drawRGBBitmap for efficient full-screen update
        ESP_LOGI("GIFDRAW:", "Drawing frame to matrix, by pussing it to  dma_display->drawRGBBitmap...");
        dma_display->drawRGBBitmap(0, 0, gifFrameBuffer, MATRIX_WIDTH, MATRIX_HEIGHT);
        // dma_display->drawGrayscaleBitmap(0, 0, (uint8_t*)gifFrameBuffer, MATRIX_WIDTH, MATRIX_HEIGHT); // Optional: for testing grayscale
        dma_display->setBrightness8(DEFAULT_BRIGHTNESS); // Reset brightness in case it was changed
        // dma_display->clearScreen(); // Clear previous frame
        // ESP_LOGI("GIFDRAW:", "Frame drawn, flipping DMA buffer...");
        // dma_display->waitDmaReady();
        // dma_display->flipDMABuffer();
    }
}


// /*
//  * ESP32-S3 HUB75 Animated GIF Player
//  * 
//  * Description: Plays animated GIFs from a microSD card on a HUB75 RGB LED matrix.
//  * Libraries: ESP32-HUB75-MatrixPanel-I2S-DMA, Adafruit_GFX, AnimatedGIF
//  * SD Card: Uses ESP-IDF SDMMC driver with FAT filesystem
//  * 
//  * How to Use:
//  * 1. Update the CONFIGURATION section below for your panel and setup.
//  * 2. Format your microSD card as FAT16 or FAT32.
//  * 3. Place your GIF file(s) in the root directory (e.g., "demo.gif").
//  * 4. Connect the HUB75 panel and SD card to the ESP32-S3-Pico as described in the WIRING section.
//  * 5. Compile and flash this project to your board using ESP-IDF.
//  * 
//  * Important Notes:
//  * - The AnimatedGIF library does NOT support interlaced GIFs.
//  * - GIFs are streamed from the SD card, not loaded fully into RAM.
//  * - Power supply is critical. Use a robust 5V supply and consider adding capacitors to each panel.
//  */

// /* ==================== CONFIGURATION ==================== */
// // --- HUB75 LED Matrix Configuration ---
// #define MATRIX_WIDTH 64         // Width of your panel in pixels
// #define MATRIX_HEIGHT 32        // Height of your panel in pixels (use 64 for 64x64 panel)
// #define MATRIX_CHAIN_LENGTH 1   // Number of chained panels (horizontal)

// // --- HUB75 Panel Pin Mapping for Waveshare ESP32-S3-Pico ---
// // IMPORTANT: For a 64x64 panel, you MUST define and connect the E_PIN.
// #define R1_PIN 4
// #define G1_PIN 5
// #define B1_PIN 6
// #define R2_PIN 7
// #define G2_PIN 15
// #define B2_PIN 16

// #define A_PIN 8
// #define B_PIN 9
// #define C_PIN 10
// #define D_PIN 11
// #define E_PIN -1                 // Set to a valid GPIO (e.g., 12) for 64x64 1/32 scan panels

// #define LAT_PIN 13
// #define OE_PIN 14
// #define CLK_PIN 12

// // --- SD Card (SPI) Pin Mapping ---
// // Using default HSPI pins for Waveshare ESP32-S3-Pico
// #define SD_MISO 13
// #define SD_MOSI 11
// #define SD_SCK  12
// #define SD_CS   10               // Chip Select pin

// // --- GIF Playback Configuration ---
// #define GIF_FILENAME "/gifs/demo.gif" // Path to GIF file on SD card (mounted at /gifs)
// #define DEFAULT_BRIGHTNESS 32    // Brightness level (0-255)
// #define MIN_FRAME_DELAY_MS 20    // Minimum delay between GIF frames (prevents excessive speed)

// // SD Card mount configuration
// #define MOUNT_POINT "/gifs"
// #define MAX_FILES 5

// /* ==================== LIBRARY INCLUDES ==================== */
// #include <stdio.h>
// #include <string.h>
// #include <sys/unistd.h>
// #include <sys/stat.h>
// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_vfs_fat.h"
// #include "driver/sdmmc_host.h"
// #include "driver/sdspi_host.h"
// #include "sdmmc_cmd.h"

// // Third-party Arduino libraries (ensure these are in your components folder)
// #include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
// #include "AnimatedGIF.h"
// #include "ff.h"
// #include <stdio.h>
// #include <string.h>
// #include <errno.h>
// #include <dirent.h>

// /* ==================== GLOBAL OBJECTS ==================== */
// // Matrix Panel Object
// MatrixPanel_I2S_DMA *dma_display = nullptr;

// // AnimatedGIF Decoder
// AnimatedGIF gif;
// // Frame buffer for scaling. Size for a single panel.
// uint16_t gifFrameBuffer[MATRIX_WIDTH * MATRIX_HEIGHT];

// // SD Card handle
// sdmmc_card_t *card = NULL;
// static const char *TAG = "SDCARD";

// /* ==================== FUNCTION PROTOTYPES ==================== */
// bool setup_matrix_panel();
// bool setup_sd_card();
// void GIFDraw(GIFDRAW *pDraw);
// void* GIFOpenFile(const char *fname, int32_t *pSize);
// void GIFCloseFile(void *pHandle);
// int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen);
// int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition);
// void list_files(const char *path);

// /* ==================== SETUP FUNCTIONS ==================== */
// extern "C" void app_main(void)
// {
//     ESP_LOGI("MAIN", "Starting ESP32-S3 HUB75 GIF Player");

//     // 1. Initialize the LED Matrix Panel
//     if (!setup_matrix_panel()) {
//         ESP_LOGE("MAIN", "Matrix panel initialization failed. Halting.");
//         vTaskDelay(portMAX_DELAY); // Stop here
//     }

//     // 2. Initialize the SD Card
//     if (!setup_sd_card()) {
//         ESP_LOGE("MAIN", "SD card initialization failed. Halting.");
//         vTaskDelay(portMAX_DELAY); // Stop here
//     }

//     // List files for debugging
//     ESP_LOGI("MAIN", "Listing files on SD card:");
//     list_files(MOUNT_POINT);

//     // 3. Configure the GIF decoder with file I/O callbacks
//     gif.begin(LITTLE_ENDIAN_PIXELS);
    
//     // Try to open the GIF file
//     ESP_LOGI("MAIN", "Attempting to open GIF file: %s", GIF_FILENAME);
//     if (!gif.open(GIF_FILENAME, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
//         ESP_LOGE("MAIN", "Failed to open GIF file: %s", GIF_FILENAME);
//         ESP_LOGI("MAIN", "Please ensure the file exists at the root of the SD card");
//         vTaskDelay(portMAX_DELAY);
//     }

//     ESP_LOGI("MAIN", "GIF opened successfully. Canvas size: %d x %d", 
//              gif.getCanvasWidth(), gif.getCanvasHeight());

//     // 4. Main GIF Playback Loop
//     int32_t gifFrameDelayMs = MIN_FRAME_DELAY_MS;
//     uint32_t frameCount = 0;
    
//     while (1) {
//         if (gif.isPlaying()) {
//             // Get the frame delay from the GIF file, enforce minimum
//             int32_t thisDelay = gif.getFrameDelay();
//             if (thisDelay < MIN_FRAME_DELAY_MS) {
//                 thisDelay = MIN_FRAME_DELAY_MS;
//             }
//             gifFrameDelayMs = thisDelay;

//             // Play a single frame. The GIFDraw callback will be triggered.
//             if (!gif.playFrame(false, NULL)) {
//                 ESP_LOGW("MAIN", "GIF playFrame returned false. Restarting GIF.");
//                 gif.reset();
//                 continue;
//             }
            
//             frameCount++;
//             if (frameCount % 30 == 0) {
//                 ESP_LOGI("MAIN", "Frame %lu, delay: %dms", frameCount, gifFrameDelayMs);
//             }
            
//             // Wait for the duration specified in the GIF file before next frame
//             vTaskDelay(gifFrameDelayMs / portTICK_PERIOD_MS);
//         } else {
//             // If GIF stopped (e.g., end of file), reset to start
//             ESP_LOGI("MAIN", "Restarting GIF from beginning.");
//             gif.reset();
//             frameCount = 0;
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         }
//     }
// }

// /**
//  * @brief Initializes the HUB75 LED Matrix Panel using DMA.
//  * @return true if successful, false otherwise.
//  */
// bool setup_matrix_panel() {
//     ESP_LOGI("MATRIX", "Initializing HUB75 panel...");
    
//     // HUB75 panel configuration structure
//     HUB75_I2S_CFG::i2s_pins _pins = {
//         .r1 = R1_PIN, .g1 = G1_PIN, .b1 = B1_PIN,
//         .r2 = R2_PIN, .g2 = G2_PIN, .b2 = B2_PIN,
//         .a = A_PIN, .b = B_PIN, .c = C_PIN, .d = D_PIN, .e = E_PIN,
//         .lat = LAT_PIN, .oe = OE_PIN, .clk = CLK_PIN
//     };

//     // Matrix options: width, height, chain length, pin mapping, color depth, double buffer
//     HUB75_I2S_CFG mxconfig(
//         MATRIX_WIDTH,            // Panel width
//         MATRIX_HEIGHT,           // Panel height
//         MATRIX_CHAIN_LENGTH,     // Number of chained panels
//         _pins                    // Pin mapping
//     );
    
//     // --- Critical Configuration Parameters ---
//     mxconfig.gpio.e = E_PIN;                     // REQUIRED for 64x64 1/32 scan panels
//     mxconfig.double_buff = true;                 // Smooth animation, uses 2x memory
//     mxconfig.clkphase = false;                   // Usually false for HUB75
//     mxconfig.driver = HUB75_I2S_CFG::SHIFTREG;   // Common driver type
//     mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_10M;   // 10MHz clock, good for P3 panels
//     mxconfig.min_refresh_rate = 120;             // Target refresh rate in Hz

//     // Create the display object
//     dma_display = new MatrixPanel_I2S_DMA(mxconfig);
    
//     if (!dma_display->begin()) {
//         ESP_LOGE("MATRIX", "Matrix begin() failed. Check wiring and pins.");
//         return false;
//     }

//     dma_display->setBrightness8(DEFAULT_BRIGHTNESS); // 0-255 scale
//     dma_display->clearScreen();
//     dma_display->fillScreen(0); // Start with black screen

//     ESP_LOGI("MATRIX", "Panel initialized: %dx%d (chain: %d)", 
//              MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_CHAIN_LENGTH);
//     return true;
// }

// /**
//  * @brief Initializes the SPI bus and mounts the SD card using ESP-IDF FATFS.
//  * @return true if successful, false otherwise.
//  */
// bool setup_sd_card() {
//     ESP_LOGI(TAG, "Initializing SD card...");

//     // Options for mounting the filesystem
//     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
//         .format_if_mount_failed = false,
//         .max_files = MAX_FILES,
//         .allocation_unit_size = 16 * 1024,
//         .disk_status_check_enable = false
//     };

//     // Configure the SPI bus
//     sdmmc_host_t host = SDSPI_HOST_DEFAULT();
//     host.slot = SPI2_HOST; // HSPI (SPI2) on ESP32-S3-Pico

//     spi_bus_config_t bus_cfg = {
//         .mosi_io_num = SD_MOSI,
//         .miso_io_num = SD_MISO,
//         .sclk_io_num = SD_SCK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4000,
//     };

//     // Initialize SPI bus
//     ESP_LOGI(TAG, "Initializing SPI bus...");
//     esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
//         return false;
//     }

//     // Configure SD SPI interface
//     sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
//     slot_config.gpio_cs = (gpio_num_t)SD_CS;
//     slot_config.host_id = host.slot;

//     ESP_LOGI(TAG, "Mounting filesystem...");
//     ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

//     if (ret != ESP_OK) {
//         if (ret == ESP_FAIL) {
//             ESP_LOGE(TAG, "Failed to mount filesystem. "
//                      "If you want the card to be formatted, set format_if_mount_failed = true.");
//         } else {
//             ESP_LOGE(TAG, "Failed to initialize the card (%s). "
//                      "Make sure SD card lines have pull-up resistors in place.", 
//                      esp_err_to_name(ret));
//         }
//         return false;
//     }

//     // Card has been initialized, print its properties
//     sdmmc_card_print_info(stdout, card);
//     ESP_LOGI(TAG, "SD card mounted successfully at %s", MOUNT_POINT);
    
//     return true;
// }

// /**
//  * @brief Lists files in a directory (for debugging)
//  */
// void list_files(const char *path) {
//     DIR *dir = opendir(path);
//     if (!dir) {
//         ESP_LOGE(TAG, "Failed to open directory: %s", path);
//         return;
//     }

//     struct dirent *entry;
//     while ((entry = readdir(dir)) != NULL) {
//         ESP_LOGI(TAG, "  %s", entry->d_name);
//     }
//     closedir(dir);
// }

// /* ==================== GIF DECODER CALLBACKS ==================== */
// /**
//  * @brief Called by AnimatedGIF library to open a file.
//  */
// void* GIFOpenFile(const char *fname, int32_t *pSize) {
//     ESP_LOGI("GIF", "Opening file: %s", fname);
    
//     FILE *file = fopen(fname, "rb");
//     if (!file) {
//         ESP_LOGE("GIF", "Failed to open file: %s", fname);
//         return NULL;
//     }
    
//     // Get file size
//     fseek(file, 0, SEEK_END);
//     *pSize = ftell(file);
//     fseek(file, 0, SEEK_SET);
    
//     ESP_LOGI("GIF", "File opened successfully. Size: %ld bytes", *pSize);
//     return (void *)file;
// }

// /**
//  * @brief Called by AnimatedGIF library to close a file.
//  */
// void GIFCloseFile(void *pHandle) {
//     FILE *file = (FILE *)pHandle;
//     if (file) {
//         fclose(file);
//     }
// }

// /**
//  * @brief Called by AnimatedGIF library to read data from file.
//  */
// int32_t GIFReadFile(GIFFILE *pHandle, uint8_t *pBuf, int32_t iLen) {
//     FILE *file = (FILE *)pHandle->fHandle;
//     if (!file) return 0;
    
//     size_t bytesRead = fread(pBuf, 1, iLen, file);
//     return (int32_t)bytesRead;
// }

// /**
//  * @brief Called by AnimatedGIF library to seek in file.
//  */
// int32_t GIFSeekFile(GIFFILE *pHandle, int32_t iPosition) {
//     FILE *file = (FILE *)pHandle->fHandle;
//     if (!file) return 0;
    
//     if (fseek(file, iPosition, SEEK_SET) != 0) {
//         return 0;
//     }
//     return iPosition;
// }

// /**
//  * @brief Callback for each decoded GIF frame. Handles scaling and drawing to matrix.
//  * @param pDraw Pointer to GIFDRAW structure with frame data.
//  */
// void GIFDraw(GIFDRAW *pDraw) {
//     if (!dma_display) return;

//     // Calculate scaling factors to fit GIF into matrix
//     int gifWidth = gif.getCanvasWidth();
//     int gifHeight = gif.getCanvasHeight();
    
//     // Target drawing area (centered)
//     int drawWidth = (gifWidth > MATRIX_WIDTH) ? MATRIX_WIDTH : gifWidth;
//     int drawHeight = (gifHeight > MATRIX_HEIGHT) ? MATRIX_HEIGHT : gifHeight;
//     int offsetX = (MATRIX_WIDTH - drawWidth) / 2;
//     int offsetY = (MATRIX_HEIGHT - drawHeight) / 2;

//     // Skip if offsets are invalid
//     if (offsetX < 0) offsetX = 0;
//     if (offsetY < 0) offsetY = 0;

//     // Clear frame buffer to black
//     memset(gifFrameBuffer, 0, sizeof(gifFrameBuffer));

//     // Scale and convert 8-bit GIF palette to 16-bit RGB565 for the matrix
//     for (int y = 0; y < pDraw->iHeight; y++) {
//         // Calculate target Y position with scaling
//         int sourceY = pDraw->y + y;
//         int matrixY = offsetY + (sourceY * drawHeight / gifHeight);
//         if (matrixY < 0 || matrixY >= MATRIX_HEIGHT) continue;

//         uint8_t *src = &pDraw->pPixels[y * pDraw->iWidth];
//         uint16_t *dest = &gifFrameBuffer[matrixY * MATRIX_WIDTH];

//         for (int x = 0; x < pDraw->iWidth; x++) {
//             // Calculate target X position with scaling
//             int matrixX = offsetX + (x * drawWidth / pDraw->iWidth);
//             if (matrixX < 0 || matrixX >= MATRIX_WIDTH) continue;

//             uint8_t paletteIndex = src[x];
//             if (paletteIndex != pDraw->ucTransparent && paletteIndex < 256) {
//                 // Convert 8-bit palette to 16-bit color
//                 // The GIF library provides the actual palette in pDraw->pPalette
//                 uint8_t r = pDraw->pPalette[paletteIndex].r;
//                 uint8_t g = pDraw->pPalette[paletteIndex].g;
//                 uint8_t b = pDraw->pPalette[paletteIndex].b;
                
//                 // Convert 24-bit RGB to 16-bit RGB565
//                 uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
//                 dest[matrixX] = color;
//             }
//         }
//     }

//     // If this is the last line of the frame, push the buffer to the matrix
//     if (pDraw->y + pDraw->iHeight >= gifHeight) {
//         // Use Adafruit_GFX drawRGBBitmap for efficient full-screen update
//         dma_display->drawRGBBitmap(0, 0, gifFrameBuffer, MATRIX_WIDTH, MATRIX_HEIGHT);
//     }
// }
