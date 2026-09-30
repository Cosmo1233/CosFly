# BMM150 magnetometer: LibDriver vendor sources (unmodified) + CosFly ESP-IDF port.
idf_component_register(SRCS "driver_bmm150.c" "bmm150_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES i2c_bus)
