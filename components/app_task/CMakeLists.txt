idf_component_register(
        SRCS "app_task.c"
        INCLUDE_DIRS "include"
        REQUIRES bsp_init move step_motor esp_timer
)
