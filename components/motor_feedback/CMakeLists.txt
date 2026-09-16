idf_component_register(
        SRCS "motor_feedback.c"
        INCLUDE_DIRS "include"
        REQUIRES uart_comm esp_timer
)
