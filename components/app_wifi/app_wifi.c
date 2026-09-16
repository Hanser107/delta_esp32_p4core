/**
 * @file app_wifi.c
 * @brief Wi-Fi STA 建链与网络服务启动的实现。
 * @details 初始化 NVS、netif 与事件循环，连接成功后启动 HTTP 服务器与图案播放器。
 */

#include "app_wifi.h"
#include "app_http.h"
#include "app_pattern.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define TAG "app_wifi"

#define WIFI_SSID   CONFIG_DELTA_WIFI_SSID
#define WIFI_PASS   CONFIG_DELTA_WIFI_PASSWORD

/** @brief 网络服务仅在首次成功获取 IP 时启动一次。 */
static bool s_services_started;

static void start_network_services(const ip_event_got_ip_t *ip)
{
    if (s_services_started) {
        return;
    }
    s_services_started = true;

    if (http_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start");
        return;
    }
    pattern_player_init();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Web UI: http://" IPSTR "/", IP2STR(&ip->ip_info.ip));
    ESP_LOGI(TAG, "========================================");
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Connecting to \"%s\"...", WIFI_SSID);
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected, retrying...");
            esp_wifi_connect();
            break;

        default:
            break;
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ip = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP " IPSTR, IP2STR(&ip->ip_info.ip));
        start_network_services(ip);
    }
}

esp_err_t app_wifi_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "NVS init failed");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}
