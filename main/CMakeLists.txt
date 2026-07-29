idf_component_register(
        SRCS "delta_esp32_p4core.c"
        INCLUDE_DIRS "."
        REQUIRES bsp_init app_task delta screen app_wifi app_mic
)