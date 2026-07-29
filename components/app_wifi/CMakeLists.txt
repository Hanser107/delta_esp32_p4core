idf_component_register(
        SRCS "app_wifi.c"
        INCLUDE_DIRS "include"
        REQUIRES esp_wifi_remote esp_wifi nvs_flash app_http app_pattern
)