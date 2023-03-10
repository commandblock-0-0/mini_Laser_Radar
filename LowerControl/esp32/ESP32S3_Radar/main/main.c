#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "steering_control.h"
#include "radar_UART.h"

static const char *TAG = "main";

xSteering_manager_t* g_pxSteering_manager;

void vSteering(void *data)
{
    int i = 0;
    bool status = true;

    while (1)
    {
        if (status)
            i += 20;
        else
            i -= 20;
        
        if (i >= CONFIG_STEERING_ANGLE_SCOPE || i <= 0)
        {    
            status = !status;
            if (i > CONFIG_STEERING_ANGLE_SCOPE)
                i = CONFIG_STEERING_ANGLE_SCOPE;
            if (i < 0)
                i = 0;
            ESP_LOGI(TAG, "[loop!]");
        }

        steering_ChangeAngle(&g_arrSteering_arguments[0], i);
        steering_ChangeAngle(&g_arrSteering_arguments[1], i);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[here!]");

    xRadar_UART_t* xRadar_uart_Opr;

    g_pxSteering_manager = vSteering_init();

    xTaskCreate(vSteering, "SteeringTask", 2000, NULL, 5, NULL);

    //UART test
    xRadar_uart_Opr = radar_UART_Run(NULL, NULL);
    char* test_str = "This is a test string.\n";
    while (1)   
    {
        uart_write_bytes(xRadar_uart_Opr->uart_num, (const char*)test_str, strlen(test_str));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}