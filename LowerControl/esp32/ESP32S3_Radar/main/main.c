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

static ledc_timer_config_t g_Steering_Timer;
static ledc_channel_config_t g_arrSteering_Channels[5];
static xSteering_arguments_t g_arrSteering_arguments[5];

void vSteering(void *data)
{
    int i = 0;
    bool status = true;

    while (1)
    {
        if (status)
            i++;
        else
            i--;
        
        if (i == CONFIG_STRING_ANGLE_SCOPE || i == 0)
            status = !status;

        vSteering_ChangeAngle(&g_arrSteering_arguments[0], i);
        vSteering_ChangeAngle(&g_arrSteering_arguments[1], i);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    printf("here!\n");

    xRadar_UART_t* xRadar_uart_Opr;

    vSteering_init(&g_Steering_Timer, g_arrSteering_Channels, g_arrSteering_arguments);

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