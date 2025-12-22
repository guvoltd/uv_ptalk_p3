#pragma once
#include "driver/gpio.h"

// -----------------------------
// Buttons (active-low, internal pull-ups)
// -----------------------------
#define BUTTON_PIN_VOL_UP      GPIO_NUM_2
#define BUTTON_PIN_VOL_DOWN    GPIO_NUM_3
#define BUTTON_PIN_NEXT        GPIO_NUM_0   // strap; ensure high at boot
#define BUTTON_PIN_PREV        GPIO_NUM_1
#define BUTTON_PIN_PLAY        GPIO_NUM_22  // remapped off I2S DOUT
#define BUTTON_PIN_LN_CHANGE   GPIO_NUM_23  // remapped off I2S BCLK
// #define BUTTON_PIN_PODCAST   GPIO_NUM_?  // TBD
// #define BUTTON_PIN_SYS_POWER GPIO_NUM_?  // TBD

// -----------------------------
// NeoPixel (status LED)
// -----------------------------
#define NEOPIXEL_PIN           GPIO_NUM_21

// -----------------------------
// SPI (SD card) custom pins (SPI3 host)
// -----------------------------
#define SD_SCK                 GPIO_NUM_8
#define SD_MISO                GPIO_NUM_10
#define SD_MOSI                GPIO_NUM_9
#define SD_CS                  GPIO_NUM_7

// -----------------------------
// I2S (audio)
// -----------------------------
#define I2S_DOUT               GPIO_NUM_4
#define I2S_BCLK               GPIO_NUM_5
#define I2S_LRC                GPIO_NUM_6

// -----------------------------
// I2C (PN532 / PCF8574)
// -----------------------------
#define I2C_SDA                GPIO_NUM_39
#define I2C_SCL                GPIO_NUM_40

// -----------------------------
// HUB75 RGB Matrix Panel pins (P3 64x64)
// -----------------------------
#define HUB75_R1               GPIO_NUM_11
#define HUB75_G1               GPIO_NUM_12
#define HUB75_B1               GPIO_NUM_13
#define HUB75_R2               GPIO_NUM_14
#define HUB75_G2               GPIO_NUM_15
#define HUB75_B2               GPIO_NUM_16
#define HUB75_A                GPIO_NUM_17
#define HUB75_B                GPIO_NUM_18
#define HUB75_C                GPIO_NUM_33
#define HUB75_D                GPIO_NUM_34
#define HUB75_E                GPIO_NUM_35 // use for 1/32 scan
#define HUB75_CLK              GPIO_NUM_36
#define HUB75_LAT              GPIO_NUM_37
#define HUB75_OE               GPIO_NUM_38

// -----------------------------
// HUB75 RGB Matrix Panel POWER pin
// -----------------------------
#define HUB75_POWER_EN         GPIO_NUM_42
