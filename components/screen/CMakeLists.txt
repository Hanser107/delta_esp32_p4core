idf_component_register(
        SRCS "screen.c"
        INCLUDE_DIRS "include"
        REQUIRES driver bsp_init  spi_flash wt99p4c5_s1_board esp_lcd delta_ui
)
