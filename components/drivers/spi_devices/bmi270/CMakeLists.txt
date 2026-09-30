# Bosch BMI270 SensorAPI v2.86.1, vendor sources unmodified.
# Bosch's own common.c is COINES/shuttle-board glue and is deliberately not
# included -- bmi270_port.c is the ESP-IDF SPI glue that fills in bmi2_dev's
# read/write/delay callbacks instead.
idf_component_register(SRCS "bmi2.c" "bmi270.c" "bmi270_port.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)
