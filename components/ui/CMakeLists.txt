idf_component_register(
        SRCS "ui.c"
        INCLUDE_DIRS "include"
        REQUIRES driver bsp_init delta wt99p4c5_s1_board
)