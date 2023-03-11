#ifndef _RADAR_MANAGER_H_
#define _RADAR_MANAGER_H_

#include <stdio.h>
#include "sdkconfig.h"

#define RADAR_TASK_PRIORITY 0

typedef enum { //Task notification value
    TASK_RESET,
    TASK_SUSPEND,
    TASK_RUN,
}Radar_task_notify_t;

typedef struct {
    int32_t MainTask_command;
    TaskHandle_t RadarMainTask_handle;

} xRadar_Task_t;

TaskHandle_t* pRadarManager_task_Create(TaskFunction_t const RadarMainTask);
TaskHandle_t* pRadarManager_Task_get(void);
void vRadarManager_Task_Suspend(void);
void vRadarManager_Task_run(void);
void vRadarManager_Task_Stop(void);
void vRadarManager_enable_calibration(int32_t* command_code);
void vRadarManager_Specify_Angle(int32_t* command_code);

#endif