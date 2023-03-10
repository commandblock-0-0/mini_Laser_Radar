#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "steering_control.h"

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

        vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[0], i);
        vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[0], i);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[here!]");

    g_pxSteering_manager = vSteering_init();

    xTaskCreate(vSteering, "SteeringTask", 2000, NULL, 5, NULL);

    return;
}