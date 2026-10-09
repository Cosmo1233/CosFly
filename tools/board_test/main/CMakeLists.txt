# The Bosch BMI270 driver is compiled straight from the flight firmware's copy,
# so the test exercises the same vendor code (and the same 8 KB config upload)
# the drone will fly with. bmi270_port.c is left out: it reads its pins from the
# flight firmware's menuconfig, so board_test.c has its own SPI glue instead.
set(BMI270_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../components/drivers/spi_devices/bmi270")

idf_component_register(SRCS "board_test.c"
                            "${BMI270_DIR}/bmi2.c"
                            "${BMI270_DIR}/bmi270.c"
                       INCLUDE_DIRS "${BMI270_DIR}/include"
                       REQUIRES driver esp_adc esp_timer esp_wifi esp_netif
                                esp_event nvs_flash spi_flash vfs)
