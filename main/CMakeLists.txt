idf_component_register(SRCS "drivebase.c" "controller.c" "kalmanfilter.c" "timer_controller.c" "mpu_reader.c" "main.c" "led_rgb.c" "my_i2c.c" "pid.c" "balance_control.c"  
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES drv8825)
