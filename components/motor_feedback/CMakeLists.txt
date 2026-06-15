idf_component_register(
        SRCS "motor_feedback.c"
        INCLUDE_DIRS "include"
        REQUIRES driver bsp_init uart_comm esp_timer
)