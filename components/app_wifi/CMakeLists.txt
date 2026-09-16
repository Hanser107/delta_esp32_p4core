idf_component_register(
        SRCS "app_wifi.c"
        INCLUDE_DIRS "include"
        REQUIRES esp_wifi esp_wifi_remote esp_event esp_netif nvs_flash
                 app_http app_pattern
)
