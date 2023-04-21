#ifndef _RADAR_MANAGER_H_
#define _RADAR_MANAGER_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "sdkconfig.h"
#include "steering_control.h"
#include "radar_uart.h"
#include "mod_bus.h"

#define RADAR_TASK_PRIORITY 0

enum {  /* task priority */
    HIGH_PRIORITY   = 12,
    MIDDLE_PRIORITY = 6,
    LOW_PRIORITY    = 1,
};


typedef struct {
    int32_t MainTask_command;
    TaskHandle_t RadarMainTask_handle;

} xRadar_Task_t;

typedef struct {
    uint8_t scan_rate;                  /* Automatic scanning rate setting */
    uint8_t work_mode;                  /* Work mode setting */
    uint8_t Measure_mode;               /* Measurement mode settings */
    xRadar_UART_t* Uart_listHand;       /* UART */
    Modbus_uart_rx_data* p_uart_data;   /* Frames received by UART */
    uint16_t Measurement_sensor_address;/* Measurement sensor address */
    uint16_t measure_data;              /* measure data */
    xSteering_manager_t* p_steering;    /* Including all available steering gears */
    EventGroupHandle_t Task_EventGroup; /* 0~4 bits is steering, 5 bits is Distance Sensor*/
    TaskHandle_t Steering_task_Handle;
    TaskHandle_t input_measure_Task_Handle;
    TaskHandle_t input_Execution_Task_Handle;
} Radar_status;

esp_err_t Radar_manager_init(void);
esp_err_t Radar_manager_Modbus_carry_out(TickType_t xTicksToWait);

void Radar_input_Execution_Task(void* pvParameters);
void Radar_input_measure_Task(void* pRadar_status);

#endif