idf_component_register(
        SRCS "app_task.c"
        INCLUDE_DIRS "include"
        REQUIRES bsp_init led uart_comm move
)