idf_component_register(
        SRCS "step_motor.c"
        INCLUDE_DIRS "include"
        REQUIRES motor_feedback esp_timer
)
