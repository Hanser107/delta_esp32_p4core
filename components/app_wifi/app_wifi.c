#include "app_wifi.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "string.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "time.h"
#include "app_http.h"
#include "app_pattern.h"

#define TAG "p4_wifi"

#define WIFI_SSID "nagatoYuki"
#define WIFI_PASS "nagatoYuki"

static void ESP_WIFI_Event_CallBack(void* event_handler_arg,\
                                    esp_event_base_t event_base,\
                                    int32_t event_id,\
                                    void* event_data)
{
    if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG,"开始初始化WIFI组件");
                ESP_ERROR_CHECK(esp_wifi_connect());

                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGE(TAG,"WIFI连接失败");
                ESP_ERROR_CHECK(esp_wifi_connect());
                vTaskDelay(pdMS_TO_TICKS(500));
            default:
                break;
        }

    }
    else if(event_base == IP_EVENT)
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG,"Wi-Fi connected successfully");
                ESP_LOGI(TAG,"Starting the HTTP server");
                esp_err_t ret =  http_server_start();
                if(ret == ESP_OK) {
                    ESP_LOGI(TAG,"HTTP server started");
                    pattern_player_init();
                    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
                    pattern_player_init();
                    ESP_LOGI(TAG, "========================================");
                    ESP_LOGI(TAG, "  打开浏览器访问: http://" IPSTR "/", IP2STR(&event->ip_info.ip));
                    ESP_LOGI(TAG, "========================================");
                }
                else {
                    ESP_LOGE(TAG,"HTTP server start failed: %x", ret);
                }
                //SNTP_Init();


                break;
            default:
                break;
        }
    }

}



void WIFI_Init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,ESP_WIFI_Event_CallBack,NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,ESP_WIFI_Event_CallBack,NULL));
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t WIFI_Init_Config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {
        .sta.ssid = WIFI_SSID,
        .sta.password = WIFI_PASS,
        .sta.threshold.authmode = WIFI_AUTH_WPA2_PSK,
    };

    ESP_ERROR_CHECK(esp_wifi_init(&WIFI_Init_Config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());


}
