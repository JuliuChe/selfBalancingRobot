idf_component_register(
    SRCS "motor_sine_test.c"
    INCLUDE_DIRS "."
    REQUIRES
      esp_driver_gpio
    PRIV_REQUIRES
        drv8825
        freertos
        log
)