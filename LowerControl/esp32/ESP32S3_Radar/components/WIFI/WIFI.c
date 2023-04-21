#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_smartconfig.h"

#include "WIFI.h"

#define ESP_DEFAULT_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_DEFAULT_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_WIFI_MAXIMUM_RETRY     CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static wifi_config_t g_wifi_config = {
        .sta = {
            .ssid = "xxx",//ESP_DEFAULT_WIFI_SSID,
            .password = "123",//ESP_DEFAULT_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
	     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1
#define WIFI_SMART_CONFIG_BIT BIT2

#define NVS_SPACE_NAME "WiFi_cfg"
#define NVS_KEY_SSID "wifi_ssid"
#define NVS_KEY_PASSED "wifi_passwd"

static const char *TAG = "wifi station";

static int s_retry_num = 0;

SemaphoreHandle_t smartconfig_Semaphore = NULL;

static void smartconfig_task(void * parm);
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static esp_err_t NVSread_to_wifi_config( const char* namespace_name );
static void NVSupdate_data(const char* namespace_name, smartconfig_event_got_ssid_pswd_t* data);

esp_err_t wifi_init_sta(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &g_wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) { //successed
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 g_wifi_config.sta.ssid);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) { // start smartconfig
        ESP_LOGW(TAG, "Failed to connect to SSID:%s",
                 g_wifi_config.sta.ssid);
        ESP_LOGW(TAG, "Start SmartConfig!");
        smartconfig_Semaphore = xSemaphoreCreateBinary();
        TaskHandle_t smart_task_Handle;
        xTaskCreatePinnedToCore(smartconfig_task, "smartconfig_task", 4096, NULL, 12, &smart_task_Handle, 0);
        if (xSemaphoreTake(smartconfig_Semaphore, portMAX_DELAY) == pdTRUE) // wait smartconfig
            return ESP_OK;
        else {
            vTaskDelete(smart_task_Handle);
            esp_smartconfig_stop();
            return ESP_ERR_NOT_FOUND;
        }
    } else { // other error
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) { // first connect
            esp_wifi_connect();
        }
        if (event_id == WIFI_EVENT_STA_DISCONNECTED) { //retry
            ESP_LOGI(TAG,"connect to the AP fail");
            if (s_retry_num < ESP_WIFI_MAXIMUM_RETRY) {
                //user config SSID and PASSWORD
                s_retry_num++;
                ESP_LOGI(TAG, "retry to connect to the AP");
                esp_wifi_connect();
            } else if (s_retry_num < ESP_WIFI_MAXIMUM_RETRY * 2) {
                //if over max_try_num, use config in NVS to try ESP_WIFI_MAXIMUM_RETRY
                if (ESP_OK == NVSread_to_wifi_config("WiFi_cfg")) {
                    s_retry_num++;
                    ESP_LOGI(TAG, "retry to connect to the AP by NVSconfig");
                    esp_wifi_connect();
                }
                else {
                    ESP_LOGW(TAG, "NVS config error!");
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
            } else {
                //all old config fail
                ESP_LOGW(TAG, "Here!");
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    }

    if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }

    if (event_base == SC_EVENT) {
        if (event_id == SC_EVENT_SCAN_DONE) {
            ESP_LOGI(TAG, "Scan done");
        }
        if (event_id == SC_EVENT_FOUND_CHANNEL) {
            ESP_LOGI(TAG, "Found channel");
        }
        if (event_id == SC_EVENT_SEND_ACK_DONE) {
            ESP_LOGI(TAG, "sent ACK");
        }
        if (event_id == SC_EVENT_GOT_SSID_PSWD) {
            ESP_LOGI(TAG, "Got SSID and password");
            NVSupdate_data("WiFi_cfg", (smartconfig_event_got_ssid_pswd_t*)event_data);
            xEventGroupSetBits(s_wifi_event_group, WIFI_SMART_CONFIG_BIT);
            esp_wifi_connect();
        }
    }
}

static void smartconfig_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_esptouch_set_timeout(255));
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    ESP_LOGI(TAG, "smartconfig start .......");
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, 
                                        WIFI_CONNECTED_BIT | WIFI_SMART_CONFIG_BIT, 
                                            true, false, portMAX_DELAY);
        if(uxBits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & WIFI_SMART_CONFIG_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            //esp_smartconfig_stop();
            xSemaphoreGive(smartconfig_Semaphore);
            //vTaskDelete(NULL);
        }
    }
}

// read NVS config
// and updata wifi config
static esp_err_t NVSread_to_wifi_config( const char* namespace_name )
{
    nvs_handle_t wifi_config_HandleNvs;
    size_t len;
    char wifi_ssid[33] = { 0 };
    char wifi_passwd[65] = { 0 };
    esp_err_t err = nvs_open(namespace_name, NVS_READONLY, &wifi_config_HandleNvs);
    if (err != ESP_OK)
    {
        ESP_LOGW("NVS", "open fail!");
        nvs_close(wifi_config_HandleNvs);
        return err;
    } else {
        ESP_LOGI(TAG, "get NVS config");
        len = sizeof(wifi_ssid);
        err = nvs_get_str(wifi_config_HandleNvs, NVS_KEY_SSID, wifi_ssid, &len);
        if (err != ESP_OK)
        {
            ESP_LOGW("NVS", "no wifi_ssid!");
            nvs_close(wifi_config_HandleNvs);
            return err;
        }
        len = sizeof(wifi_passwd);
        err = nvs_get_str(wifi_config_HandleNvs, NVS_KEY_PASSED, wifi_passwd, &len);
        if (err != ESP_OK)
        {
            ESP_LOGW("NVS", "no wifi_passwd!");
            nvs_close(wifi_config_HandleNvs);
            return err;
        }
    }
    memcpy(g_wifi_config.sta.ssid, wifi_ssid, sizeof(g_wifi_config.sta.ssid));
    memcpy(g_wifi_config.sta.password, wifi_passwd, sizeof(g_wifi_config.sta.password));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &g_wifi_config) );

    nvs_close(wifi_config_HandleNvs);
    return ESP_OK;
}

// updata NVS config
// and updata wifi config
static void NVSupdate_data(const char* namespace_name, smartconfig_event_got_ssid_pswd_t* data)
{
    smartconfig_event_got_ssid_pswd_t *evt = data;
    char ssid[33] = { 0 };
    char password[65] = { 0 };
    uint8_t rvd_data[33] = { 0 };

    bzero(&g_wifi_config, sizeof(wifi_config_t));
    memcpy(g_wifi_config.sta.ssid, evt->ssid, sizeof(g_wifi_config.sta.ssid));
    memcpy(g_wifi_config.sta.password, evt->password, sizeof(g_wifi_config.sta.password));
    g_wifi_config.sta.bssid_set = evt->bssid_set;
    if (g_wifi_config.sta.bssid_set == true) {
        memcpy(g_wifi_config.sta.bssid, evt->bssid, sizeof(g_wifi_config.sta.bssid));
    }
    memcpy(ssid, evt->ssid, sizeof(evt->ssid));
    memcpy(password, evt->password, sizeof(evt->password));

    ESP_LOGI(TAG, "SSID:%s", ssid);
    ESP_LOGI(TAG, "PASSWORD:%s", password);
    if (evt->type == SC_TYPE_ESPTOUCH_V2) {
        ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
        ESP_LOGI(TAG, "RVD_DATA:");
        for (int i=0; i<33; i++) {
            printf("%02x ", rvd_data[i]);
        }
        printf("\n");
    }
    // write new config to nvs
    nvs_handle_t wifi_config_HandleNvs;
    ESP_ERROR_CHECK( nvs_open(namespace_name, NVS_READWRITE, &wifi_config_HandleNvs) );
    ESP_ERROR_CHECK( nvs_set_str(wifi_config_HandleNvs, NVS_KEY_SSID, ssid) );
    ESP_ERROR_CHECK( nvs_set_str(wifi_config_HandleNvs, NVS_KEY_PASSED, password) );
    ESP_ERROR_CHECK( nvs_commit(wifi_config_HandleNvs) );
    nvs_close(wifi_config_HandleNvs);
    ESP_ERROR_CHECK( esp_wifi_disconnect() );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &g_wifi_config) );
}