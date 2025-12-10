idf_component_register(
    SRCS "src/logging.cpp" "src/app_config.cpp"
    INCLUDE_DIRS "include"
    REQUIRES spi_flash
)
