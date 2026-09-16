idf_component_register(
        SRCS "main.c"
        INCLUDE_DIRS "."
        REQUIRES bsp_init
                 move
                 delta
                 app_task
                 app_wifi
                 screen
)
