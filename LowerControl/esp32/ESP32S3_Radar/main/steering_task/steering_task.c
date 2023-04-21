 
#include "steering_control.h"
#include "radar_manager.h"
#include "steering_task.h"
#include "atk_ms53l0m.h"

#define STEERING_0 0

static Radar_status* g_pRadar_status;
static xSteering_manager_t* g_pxSteering_manager; //Steering gear structure,after the initialization of the steering gear, 
                                                 //the manager.c transfers it into the task function
static EventGroupHandle_t g_xTask_EventGroup; /* event group 0~4 bits is steering, 5 bits is Distance Sensor */
static uint8_t g_scan_step = 5;
static uint8_t g_scan_speed = 20;

void Radar_Steering_task(void* pRadar_status)
{
    /* During system initialization, the servo task initializes and pauses waiting to start */
    g_pRadar_status = (Radar_status*)pRadar_status;
    g_xTask_EventGroup = g_pRadar_status->Task_EventGroup;
    g_pxSteering_manager = g_pRadar_status->p_steering;

    uint32_t task_status = STEERING_TASK_SUSPEND; 
    bool* steering_direction = &g_pxSteering_manager->steering_arr[STEERING_0].steering_direction;
    int32_t loop_angle;

    *steering_direction = true;
    vTaskSuspend(NULL); //Wait for first Wakeup

    while (1)
    {
        xTaskNotifyWait(0, 0, &task_status, 0); // Detect externally sent notifications per loop

        if (task_status == STEERING_TASK_RUN) { 
            /* Determine the scanning direction of the servo */
            if (*steering_direction)
                loop_angle = g_pxSteering_manager->steering_arr[STEERING_0].angle_now + g_scan_step;
            else 
                loop_angle = g_pxSteering_manager->steering_arr[STEERING_0].angle_now - g_scan_step;
            /* Determine if the angle has reached the range of the steering gear */
            if (loop_angle > CONFIG_STEERING_ANGLE_SCOPE)
            {
                *steering_direction = !*steering_direction; /* change scan direction */
                loop_angle = CONFIG_STEERING_ANGLE_SCOPE;
            }
            if (loop_angle < 0)
            {
                *steering_direction = !*steering_direction; /* change scan direction */
                loop_angle = 0;
            }
            /* Change angle */
            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[STEERING_0], (uint16_t)loop_angle); 
            vTaskDelay(g_scan_speed / portTICK_PERIOD_MS);

        } else if (task_status == STEERING_TASK_SUSPEND) {
            vTaskSuspend(NULL); /* task suspension */

        } else if (task_status == STEERING_TASK_RESET) {
            *steering_direction = true; /* Reset scan direction */
            vSteering_ResetAngle(); /* Reset the steering angle */
            vTaskSuspend(NULL); /* task suspension */
        } else if (task_status == STEERING_TASK_SPECIAL) {
            /*  */
            vSteering_ChangeAngle(&g_pxSteering_manager->steering_arr[STEERING_0], g_pxSteering_manager->steering_arr[STEERING_0].angle_now); 
            /* Wait for 500ms for the steering gear to rotate in place */
            vTaskDelay(500 / portTICK_PERIOD_MS);
            /* Report the event group that the steering gear rotation is complete */
            xEventGroupSetBits(g_xTask_EventGroup, 0x1F); /* event group 0~4 bits is steering */
            vTaskSuspend(NULL); /* task suspension */
        }
    }
}