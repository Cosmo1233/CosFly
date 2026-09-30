# PCA9534/PCA9534A I2C GPIO expander (drives the CosFly status LEDs).
idf_component_register(SRCS "pca9534.c"
                       INCLUDE_DIRS "include"
                       REQUIRES i2c_bus)
