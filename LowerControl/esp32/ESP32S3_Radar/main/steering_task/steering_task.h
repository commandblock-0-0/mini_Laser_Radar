#ifndef _STEERING_TASK_H
#define _STEERING_TASK_H

enum { //Task notification value
    STEERING_TASK_RESET,
    STEERING_TASK_SUSPEND,
    STEERING_TASK_RUN,
    STEERING_TASK_SPECIAL,
};

void Radar_Steering_task(void* Radar_status);

#endif