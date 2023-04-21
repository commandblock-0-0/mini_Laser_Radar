#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "WIFI.h"
#include "sdkconfig.h"
#include "radar_manager.h"
#include "radar_uart.h"
#include "steering_control.h"
#include "atk_ms53l0m.h"

static const char *TAG = "main";

//xSteering_manager_t* g_pxSteering_manager;
//xRadar_UART_t* g_xRadar_uart_Opr;

void app_main(void)
{
    ESP_LOGI(TAG, "[helloworld!]");

    //uint8_t ret;
    //uint16_t atk_ms53l0m_addr, dat;

    //xTaskCreatePinnedToCore(udp_client_task, "udp_client_task", 4096, NULL, 0, NULL, 0);
    //g_pxSteering_manager = vSteering_init();
    //ret = atk_ms53l0m_init(1, &atk_ms53l0m_addr);
    //if (ret != ATK_MS53L0M_EOK)
    //    ESP_LOGW(TAG,"atk_ms53l0m init error num:%d!",ret);
    //else
    //{
    //    ESP_LOGI(TAG,"atk_ms53l0m init done!");
    //    while (1)
    //    {
    //        ret = atk_ms53l0m_modbus_get_data(atk_ms53l0m_addr, &dat);
    //        if (ret != ATK_MS53L0M_EOK)
    //            ESP_LOGW(TAG,"atk_ms53l0m get data error!");
    //        else
    //            ESP_LOGI(TAG, "d: %d", dat);

    //        vTaskDelay(1000 / portTICK_PERIOD_MS);
    //    }
    //}

    Radar_manager_init();
    vTaskDelete(NULL);
}