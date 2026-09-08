# Bosch BMI270 SensorAPI v2.86.1, vendor sources unmodified.
# Bosch's own common.c is COINES/shuttle-board glue and is deliberately not
# included -- the ESP-IDF SPI glue that fills in bmi2_dev's read/write/delay
# callbacks lives outside this vendor drop.
idf_component_register(SRCS "bmi2.c" "bmi270.c"
                       INCLUDE_DIRS "include")
