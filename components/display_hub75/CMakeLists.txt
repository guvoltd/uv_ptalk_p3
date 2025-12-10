idf_component_register(
    SRCS "src/display_hub75.cpp"
    INCLUDE_DIRS "include" "../common/include"
    REQUIRES driver esp_timer MatrixPanel_I2S_DMA
)