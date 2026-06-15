idf_component_register(
    SRCS "move.c"
    INCLUDE_DIRS "include"
    REQUIRES driver bsp_init delta
)