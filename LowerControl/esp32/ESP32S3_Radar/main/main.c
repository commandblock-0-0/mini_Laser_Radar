#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "radar_manager.h"
#include "radar_UART.h"
#include "steering_control.h"

static const char *TAG = "main";

xSteering_manager_t* g_pxSteering_manager;
xRadar_UART_t* g_xRadar_uart_Opr;

void RadarMainTask(void *data)
{
    int32_t angle = 0;
    *((int32_t**)data) = &angle;
    uint32_t task_status = TASK_RESET;
    bool steering_status = true;
    vTaskSuspend(NULL);
    vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[0], 0);
    vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[1], 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    // Use the notification value to manage the status of the task
    while (1)
    {
        xTaskNotifyWait(0, 0, &task_status, 0);
        if (task_status == TASK_RESET)// task reset
        {
            angle = 0;
            steering_status = true;
            vTaskSuspend(NULL);
            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[0], 0);
            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[1], 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        if (task_status == TASK_SUSPEND)
            vTaskSuspend(NULL);
        if (task_status == TASK_RUN)
        {
            if (steering_status)
                angle += 5;
            else
                angle -= 5;

            if (angle >= CONFIG_STEERING_ANGLE_SCOPE || angle <= 0)
            {    
                steering_status = !steering_status;
                if (angle > CONFIG_STEERING_ANGLE_SCOPE)
                    angle = CONFIG_STEERING_ANGLE_SCOPE;
                if (angle < 0)
                    angle = 0;
                ESP_LOGI(TAG, "[loop!]");
            }

            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[0], angle);
            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[1], angle);

            vTaskDelay(30 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[helloworld!]");

    g_pxSteering_manager = vSteering_init();

    g_xRadar_uart_Opr = radar_UART_Run(NULL, NULL);

    pRadarManager_task_Create(RadarMainTask);
}