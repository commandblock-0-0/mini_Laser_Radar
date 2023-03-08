#ifndef _STRING_CONTROL_H_
#define _STRING_CONTROL_H_

#include "driver/ledc.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

/*
 *Use all parameters of steering gear
*/
typedef struct {
    uint8_t angle_now;      //angle

    uint32_t channel;       //pwm channel
    ledc_mode_t speedmode;//pwm speed mode
} xSteering_arguments_t;

esp_err_t steering_init(ledc_timer_config_t *ledc_timer, 
                        ledc_channel_config_t *config_arr,
                        xSteering_arguments_t *Steering_arr);//arr no more than 5 item
esp_err_t steering_ChangeAngle(xSteering_arguments_t *parguments, const uint8_t angle);

#endif 
