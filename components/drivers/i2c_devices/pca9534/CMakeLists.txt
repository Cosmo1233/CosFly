# PCA9534/PCA9534A I2C GPIO expander (drives the CosFly status LEDs).
# Mahda vendor driver (unmodified) + CosFly ESP-IDF port. The library's sample
# PCA9534_platform.c is deliberately not used -- see pca9534_port.h.
idf_component_register(SRCS "PCA9534.c" "pca9534_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES i2c_bus)
