idf_component_register(
        SRCS "move.c"
        INCLUDE_DIRS "include"
        REQUIRES step_motor motor_feedback esp_timer
)
