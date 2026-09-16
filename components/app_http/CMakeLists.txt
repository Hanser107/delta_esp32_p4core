idf_component_register(
        SRCS "app_http.c"
        INCLUDE_DIRS "include"
        REQUIRES app_pattern esp_http_server json
)
