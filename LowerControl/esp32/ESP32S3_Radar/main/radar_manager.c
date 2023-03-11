#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "radar_manager.h"
#include "radar_UART.h"
#include "steering_control.h"

static const char* TAG = "RadarManager";
static TaskHandle_t g_RadarMainTask_handle = NULL;
static int32_t* g_pAngleinTask;

//Used by app
//This function create Radar_main_task
//After task creating, suspend it and Wait for wake-up
//This function must be used after the initialization of other devices is completed
TaskHandle_t* pRadarManager_task_Create(TaskFunction_t const RadarMainTask)
{
    xTaskCreatePinnedToCore(RadarMainTask, "RadarMainTask", 4096, &g_pAngleinTask, 
                    RADAR_TASK_PRIORITY, &g_RadarMainTask_handle, 1);

    return &g_RadarMainTask_handle;
}

//Make the task handle visible to the lower level
TaskHandle_t* pRadarManager_Task_get(void)
{
    return &g_RadarMainTask_handle;
}

void vRadarManager_Task_Suspend(void)
{
    // Notify the Main task to suspend itself
    if (g_RadarMainTask_handle)
        xTaskNotify(g_RadarMainTask_handle, TASK_SUSPEND, eSetValueWithOverwrite);
}

void vRadarManager_Task_run(void)
{
    // Notify the Main task to run
    if (g_RadarMainTask_handle)
    {
        xTaskNotify(g_RadarMainTask_handle, TASK_RUN, eSetValueWithOverwrite);
        vTaskResume(g_RadarMainTask_handle);
    }
}

//reset the status and stops
void vRadarManager_Task_Stop(void)
{
    if (g_RadarMainTask_handle)
    {    
        // Notify the Main task to reset
        xTaskNotify(g_RadarMainTask_handle, TASK_RESET, eSetValueWithOverwrite);
        vSteering_DefaultAngle();
    }
}

//Specify the angle of a single steering gear
void vRadarManager_Specify_Angle(int32_t* command_code)
{
    char* str;
    // <RADAR_MOD> <sreeringNum> <angle>
    if (command_code[0] != 3)
    {
        ESP_LOGW(TAG, "wrong number of parameters !");
        str = "wrong number of parameters !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    if ((command_code[2] <= 0) || (command_code[2] > CONFIG_STEERING_NUM))
    {
        ESP_LOGW(TAG, "no this sreering engine !");
        str = "no this sreering engine !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    if ( (command_code[3] < 0) || (command_code[3]) > CONFIG_STEERING_ANGLE_SCOPE )
    {
        ESP_LOGW(TAG, "wrong angle !");
        str = "wrong angle !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    vRadarManager_Task_Suspend();
    *g_pAngleinTask = command_code[3];
    vSteering_ChangeAngle(xSteering_GetArgumentbyNum(command_code[2]), command_code[3]);
    str = "angle change !\n";
    uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
}

//After judging the calibration mode in UART
//check whether the remaining parameters are correct
//try to start the calibration mode in steering
void vRadarManager_enable_calibration(int32_t* command_code)
{
    char* str;
    // <RADAR_MOD> <sreeringNum> <timeNum> <H/L>
    if (command_code[0] != 4)
    {
        ESP_LOGW(TAG, "wrong number of parameters !");
        str = "wrong number of parameters !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    if ((command_code[2] <= 0) || (command_code[2] > CONFIG_STEERING_NUM))
    {
        ESP_LOGW(TAG, "no this sreering engine !");
        str = "no this sreering engine !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    if ((command_code[3] <= 0) || (command_code[3] > (1000000 / CONFIG_STEERING_BASE_FREQUENCY)))
    {
        ESP_LOGW(TAG, "timeNum wrong !");
        str = "timeNum wrong !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    } 
    if (command_code[4] != 1 && command_code[4] != 0)
    {
        ESP_LOGW(TAG, "last parameter error of steering gear !");
        str = "last parameter error of steering gear !\n";
        uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
        return;
    }
    vRadarManager_Task_Suspend();
    vSteering_Calibration(command_code[2], command_code[3], command_code[4]);
    str = "Calibration done !\n";
    uart_write_bytes(CONFIG_RADAR_UART_PORT_NUM, str, strlen(str));
}
