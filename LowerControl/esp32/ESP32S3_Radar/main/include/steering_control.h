#ifndef _STRING_CONTROL_H_
#define _STRING_CONTROL_H_

#include "driver/ledc.h"
#include "sdkconfig.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

/*
 *Use all parameters of steering gear
*/
typedef struct {
    uint32_t steering_name;//Start at 1
    uint32_t angle_now;      //angle
    uint32_t angle_scope;    //from kconfig
    uint32_t min_high_time; //High level duration corresponding to minimum angle
    float angle_base_dutyNum;//Duty value corresponding to zero degree
    uint32_t max_high_time; //High level duration corresponding to maximum angle
    float angle_max_dutyNum;//Duty value corresponding to Max degree
    float angle_step_long;  //Duty value corresponding to one degree
    uint32_t channel;       //pwm channel
    ledc_mode_t speedmode;//pwm speed mode
} xSteering_arguments_t;

typedef struct {
    uint8_t steeringNum;
    ledc_timer_config_t *pPWM_timer;
    ledc_channel_config_t *config_arr;
    xSteering_arguments_t *steering_arr;
} xSteering_manager_t;

xSteering_manager_t* vSteering_init(void);
void vSteering_DefaultAngle(void);
void vSteering_ChangeAngle(xSteering_arguments_t *parguments, const uint32_t angle);
void vSteering_ChangeDutyNum(xSteering_arguments_t *parguments, const uint32_t duty);
void vSteering_Calibration(const uint32_t sreeringname, const uint32_t timeNum, const bool High_or_Low);

#endif 
