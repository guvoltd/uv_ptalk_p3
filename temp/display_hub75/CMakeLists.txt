idf_component_register(
    SRCS "src/display_hub75.cpp"
    INCLUDE_DIRS "include" "../AnimatedGIF/src/"
    REQUIRES driver esp_timer MatrixPanel_I2S_DMA arduino spi_flash fatfs common
)