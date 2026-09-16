idf_component_register(
        SRCS "bsp_init.c"
        INCLUDE_DIRS "include"
        REQUIRES uart_comm motor_feedback step_motor end_effector driver
)
