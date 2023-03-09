#include "driver/ledc.h"
#include "steering_control.h"
#include "sdkconfig.h"


static uint32_t g_total_duty = 2;//Total value of duty cycle
static float g_AngleBaseDutyNum;//Duty value corresponding to zero degree
static float g_AngleMaxDutyNum;//Duty value corresponding to Max degree
static float g_AngleStepLong;//Duty value corresponding to one degree

void vSteering_init(ledc_timer_config_t *ledc_timer, 
                        ledc_channel_config_t *config_arr,
                        xSteering_arguments_t *Steering_arr)//arr no more than 5 item
{
    int iconfigcount = CONFIG_STRING_NUM;

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer->speed_mode       = LEDC_MODE;
    ledc_timer->timer_num        = LEDC_TIMER_0;
    ledc_timer->duty_resolution  = CONFIG_STRING_DUTY_RESOLUTION;
    ledc_timer->freq_hz          = CONFIG_STRING_BASE_FREQUENCY;  // Set output frequency
    ledc_timer->clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    // change the configuration file if you want to set
    // Use up to five steering gears
    if (iconfigcount)//string 1
    {
        config_arr[0].speed_mode    = LEDC_MODE;
        config_arr[0].channel       = LEDC_CHANNEL_0;
        config_arr[0].timer_sel     = LEDC_TIMER;
        config_arr[0].intr_type     = LEDC_INTR_DISABLE;
        config_arr[0].gpio_num      = CONFIG_STRING_0_GPIO_NUM;//from sdkconfig
        config_arr[0].duty          = 0; // Set duty to 0%
        config_arr[0].hpoint        = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&config_arr[0]));
        iconfigcount--;
    }
    if (iconfigcount)//string 2
    {
        #ifndef CONFIG_STRING_1_GPIO_NUM
        #define CONFIG_STRING_1_GPIO_NUM 0
        #endif
        config_arr[1].speed_mode    = LEDC_MODE;
        config_arr[1].channel       = LEDC_CHANNEL_1;
        config_arr[1].timer_sel     = LEDC_TIMER;
        config_arr[1].intr_type     = LEDC_INTR_DISABLE;
        config_arr[1].gpio_num      = CONFIG_STRING_1_GPIO_NUM;//from sdkconfig
        config_arr[1].duty          = 0; // Set duty to 0%
        config_arr[1].hpoint        = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&config_arr[1]));
        iconfigcount--;
    }
    if (iconfigcount)//string 3
    {
        #ifndef CONFIG_STRING_2_GPIO_NUM
        #define CONFIG_STRING_2_GPIO_NUM 0
        #endif
        config_arr[2].speed_mode    = LEDC_MODE;
        config_arr[2].channel       = LEDC_CHANNEL_2;
        config_arr[2].timer_sel     = LEDC_TIMER;
        config_arr[2].intr_type     = LEDC_INTR_DISABLE;
        config_arr[2].gpio_num      = CONFIG_STRING_2_GPIO_NUM;//from sdkconfig
        config_arr[2].duty          = 0; // Set duty to 0%
        config_arr[2].hpoint        = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&config_arr[2]));
        iconfigcount--;
    }
    if (iconfigcount)//string 4
    {
        #ifndef CONFIG_STRING_3_GPIO_NUM
        #define CONFIG_STRING_3_GPIO_NUM 0
        #endif
        config_arr[3].speed_mode    = LEDC_MODE;
        config_arr[3].channel       = LEDC_CHANNEL_3;
        config_arr[3].timer_sel     = LEDC_TIMER;
        config_arr[3].intr_type     = LEDC_INTR_DISABLE;
        config_arr[3].gpio_num      = CONFIG_STRING_3_GPIO_NUM;//from sdkconfig
        config_arr[3].duty          = 0; // Set duty to 0%
        config_arr[3].hpoint        = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&config_arr[3]));
        iconfigcount--;
    }
    if (iconfigcount)//string 5
    {
        #ifndef CONFIG_STRING_4_GPIO_NUM
        #define CONFIG_STRING_4_GPIO_NUM 0
        #endif
        config_arr[4].speed_mode    = LEDC_MODE;
        config_arr[4].channel       = LEDC_CHANNEL_4;
        config_arr[4].timer_sel     = LEDC_TIMER;
        config_arr[4].intr_type     = LEDC_INTR_DISABLE;
        config_arr[4].gpio_num      = CONFIG_STRING_4_GPIO_NUM;//from sdkconfig
        config_arr[4].duty          = 0; // Set duty to 0%
        config_arr[4].hpoint        = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&config_arr[4]));
    }

    for (int i = 0; i < CONFIG_STRING_NUM; i++)
    {
        Steering_arr[i].angle_now  = CONFIG_STRING_DEFAULT_ANGLE;
        Steering_arr[i].channel    = config_arr[i].channel;
        Steering_arr[i].speedmode = config_arr[i].speed_mode;
    }

    for (int i = CONFIG_STRING_DUTY_RESOLUTION; i > 1; i--)
        g_total_duty *= 2;//exponentiation

    g_AngleBaseDutyNum = (float)CONFIG_STRING_MIN_HIGH_TIME * (float)CONFIG_STRING_BASE_FREQUENCY  
                                * (float)g_total_duty / (float)1000000; //Duty value corresponding to zero degree
    g_AngleMaxDutyNum = (float)CONFIG_STRING_MAX_HIGH_TIME * (float)CONFIG_STRING_BASE_FREQUENCY  
                                * (float)g_total_duty / (float)1000000; //Duty value corresponding to Max degree
    g_AngleStepLong = (g_AngleMaxDutyNum - g_AngleBaseDutyNum) / CONFIG_STRING_ANGLE_SCOPE; //Duty value corresponding to one degree
}

/* Angle converted to duty value */
static int iAngleToDutyNum(const uint8_t angle)
{
    int DutyNum;

    if(angle > CONFIG_STRING_ANGLE_SCOPE)
        return -1;
    
    DutyNum = (int)(g_AngleBaseDutyNum + (float)angle * g_AngleStepLong);

    return DutyNum;
}

//changing the PWM duty cycle by angle
void vSteering_ChangeAngle(xSteering_arguments_t *parguments, const uint8_t angle)
{
    parguments->angle_now = angle;

    ESP_ERROR_CHECK(ledc_set_duty(parguments->speedmode, parguments->channel, 
                                    iAngleToDutyNum(parguments->angle_now)));
    ESP_ERROR_CHECK(ledc_update_duty(parguments->speedmode, parguments->channel));
}

//changing the PWM duty cycle by dutyNum
void vSteering_ChangeDutyNum(xSteering_arguments_t *parguments, const uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(parguments->speedmode, parguments->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(parguments->speedmode, parguments->channel));
}