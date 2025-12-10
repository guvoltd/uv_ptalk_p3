idf_component_register(
    SRCS "src/storage_sd.cpp"
    INCLUDE_DIRS "include" "../common/include"
    REQUIRES spi_flash fatfs driver
)
