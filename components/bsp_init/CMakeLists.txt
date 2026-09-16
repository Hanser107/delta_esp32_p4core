idf_component_register(
    SRCS "bsp_init.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_system  driver step_motor led uart_comm motor_feedback app_task servo
)