# BMP280 barometer: LibDriver vendor sources (unmodified) + CosFly ESP-IDF port.
idf_component_register(SRCS "driver_bmp280.c" "bmp280_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES i2c_bus)
