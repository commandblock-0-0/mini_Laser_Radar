#include "driver/ledc.h"
#include "steering_control.h"
#include "sdkconfig.h"

#define STEERING_NUM CONFIG_STEERING_NUM

static uint32_t g_total_duty = 2;//Total value of duty cycle
static float g_AngleBaseDutyNum;//Duty value corresponding to zero degree
static float g_AngleMaxDutyNum;//Duty value corresponding to Max degree
static float g_AngleStepLong;//Duty value corresponding to one degree
static ledc_channel_config_t g_Config_arr[STEERING_NUM];
static xSteering_arguments_t g_Steering_arr[STEERING_NUM];

static ledc_timer_config_t g_PWM_timer = { // config ledc_timer
    .speed_mode       = LEDC_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = CONFIG_STEERING_DUTY_RESOLUTION,
    .freq_hz          = CONFIG_STEERING_BASE_FREQUENCY, // Set output frequency
    .clk_cfg          = LEDC_AUTO_CLK,
};

static xSteering_manager_t g_xSteering_manager = { // structure returned to app
    .steeringNum  =  STEERING_NUM,
    .pPWM_timer   =  &g_PWM_timer,
    .config_arr   =  g_Config_arr,
    .steering_arr =  g_Steering_arr,
};

static void vSteering_channel_init(void)
{
    #if (STEERING_NUM >= 1)
        g_xSteering_manager.config_arr[0].speed_mode    = LEDC_MODE;
        g_xSteering_manager.config_arr[0].channel       = LEDC_CHANNEL_0;
        g_xSteering_manager.config_arr[0].timer_sel     = g_PWM_timer.timer_num;
        g_xSteering_manager.config_arr[0].intr_type     = LEDC_INTR_DISABLE;
        g_xSteering_manager.config_arr[0].gpio_num      = CONFIG_STEERING_0_GPIO_NUM;//from sdkconfig
        g_xSteering_manager.config_arr[0].duty          = 0; // Set duty to 0%
        g_xSteering_manager.config_arr[0].hpoint        = 0;
    #endif
    #if (STEERING_NUM >= 2)
        g_xSteering_manager.config_arr[1].speed_mode    = LEDC_MODE;
        g_xSteering_manager.config_arr[1].channel       = LEDC_CHANNEL_1;
        g_xSteering_manager.config_arr[1].timer_sel     = g_PWM_timer.timer_num;
        g_xSteering_manager.config_arr[1].intr_type     = LEDC_INTR_DISABLE;
        g_xSteering_manager.config_arr[1].gpio_num      = CONFIG_STEERING_1_GPIO_NUM;//from sdkconfig
        g_xSteering_manager.config_arr[1].duty          = 0; // Set duty to 0%
        g_xSteering_manager.config_arr[1].hpoint        = 0;
    #endif
    #if (STEERING_NUM >= 3)
        g_xSteering_manager.config_arr[2].speed_mode    = LEDC_MODE;
        g_xSteering_manager.config_arr[2].channel       = LEDC_CHANNEL_2;
        g_xSteering_manager.config_arr[2].timer_sel     = g_PWM_timer.timer_num;
        g_xSteering_manager.config_arr[2].intr_type     = LEDC_INTR_DISABLE;
        g_xSteering_manager.config_arr[2].gpio_num      = CONFIG_STEERING_2_GPIO_NUM;//from sdkconfig
        g_xSteering_manager.config_arr[2].duty          = 0; // Set duty to 0%
        g_xSteering_manager.config_arr[2].hpoint        = 0;
    #endif
    #if (STEERING_NUM >= 4)
        g_xSteering_manager.config_arr[3].speed_mode    = LEDC_MODE;
        g_xSteering_manager.config_arr[3].channel       = LEDC_CHANNEL_3;
        g_xSteering_manager.config_arr[3].timer_sel     = g_PWM_timer.timer_num;
        g_xSteering_manager.config_arr[3].intr_type     = LEDC_INTR_DISABLE;
        g_xSteering_manager.config_arr[3].gpio_num      = CONFIG_STEERING_3_GPIO_NUM;//from sdkconfig
        g_xSteering_manager.config_arr[3].duty          = 0; // Set duty to 0%
        g_xSteering_manager.config_arr[3].hpoint        = 0;
    #endif
    #if (STEERING_NUM >= 5)
        g_xSteering_manager.config_arr[4].speed_mode    = LEDC_MODE;
        g_xSteering_manager.config_arr[4].channel       = LEDC_CHANNEL_4;
        g_xSteering_manager.config_arr[4].timer_sel     = g_PWM_timer.timer_num;
        g_xSteering_manager.config_arr[4].intr_type     = LEDC_INTR_DISABLE;
        g_xSteering_manager.config_arr[4].gpio_num      = CONFIG_STEERING_4_GPIO_NUM;//from sdkconfig
        g_xSteering_manager.config_arr[4].duty          = 0; // Set duty to 0%
        g_xSteering_manager.config_arr[4].hpoint        = 0;
    #endif
    for (int i = 0; i < STEERING_NUM; i++)
    {
        ESP_ERROR_CHECK(ledc_channel_config(&g_xSteering_manager.config_arr[i]));
        g_xSteering_manager.steering_arr[i].angle_now  = CONFIG_STEERING_DEFAULT_ANGLE;
        g_xSteering_manager.steering_arr[i].channel    = g_xSteering_manager.config_arr[i].channel;
        g_xSteering_manager.steering_arr[i].speedmode  = g_xSteering_manager.config_arr[i].speed_mode;
    }
}

xSteering_manager_t* vSteering_init(void)
{
    ESP_ERROR_CHECK(ledc_timer_config(g_xSteering_manager.pPWM_timer));// config ledc_timer
    // Prepare and then apply the LEDC PWM channel configuration
    // change the configuration file if you want to set
    // Use up to five steering gears
    vSteering_channel_init();

    for (int i = CONFIG_STEERING_DUTY_RESOLUTION; i > 1; i--)
        g_total_duty *= 2;//exponentiation

    g_AngleBaseDutyNum = (float)CONFIG_STEERING_MIN_HIGH_TIME * (float)CONFIG_STEERING_BASE_FREQUENCY  
                                * (float)g_total_duty / (float)1000000; //Duty value corresponding to zero degree
    g_AngleMaxDutyNum = (float)CONFIG_STEERING_MAX_HIGH_TIME * (float)CONFIG_STEERING_BASE_FREQUENCY  
                                * (float)g_total_duty / (float)1000000; //Duty value corresponding to Max degree
    g_AngleStepLong = (g_AngleMaxDutyNum - g_AngleBaseDutyNum) / CONFIG_STEERING_ANGLE_SCOPE; //Duty value corresponding to one degree

    vSteering_DefaultAngle();

    return &g_xSteering_manager;
}

/* Angle converted to duty value */
static int iAngleToDutyNum(const uint8_t angle)
{
    int DutyNum;

    if(angle > CONFIG_STEERING_ANGLE_SCOPE)
        return -1;
    
    DutyNum = (int)(g_AngleBaseDutyNum + (float)angle * g_AngleStepLong);

    return DutyNum;
}

//changing the PWM duty cycle by angle
void vSteering_ChangeAngle(xSteering_arguments_t *parguments, const uint16_t angle)
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

void vSteering_DefaultAngle(void)
{
    for (int i = 0; i < STEERING_NUM; i++)
        vSteering_ChangeAngle(&g_xSteering_manager.steering_arr[i], CONFIG_STEERING_DEFAULT_ANGLE);
}