#include "driver/ledc.h"
#include "esp_log.h"
#include "steering_control.h"
#include "sdkconfig.h"

#define STEERING_NUM CONFIG_STEERING_NUM
#define STEERING_DUTY_RESOLUTION CONFIG_STEERING_DUTY_RESOLUTION
#define STEERING_BASE_FREQUENCY CONFIG_STEERING_BASE_FREQUENCY
#define STEERING_DEFAULT_ANGLE CONFIG_STEERING_DEFAULT_ANGLE
#define STEERING_ANGLE_SCOPE CONFIG_STEERING_ANGLE_SCOPE

static const char *TAG = "SteeringControl";

static ledc_channel_config_t g_Config_arr[STEERING_NUM];
static xSteering_arguments_t g_Steering_arr[STEERING_NUM];

static ledc_timer_config_t g_PWM_timer = { // config ledc_timer
    .speed_mode       = LEDC_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = STEERING_DUTY_RESOLUTION,
    .freq_hz          = STEERING_BASE_FREQUENCY, // Set output frequency
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
    float AngleBaseDutyNum;
    float AngleMaxDutyNum;
    float AngleStepLong;
    uint32_t total_duty = 2;

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
    /* Calculate the default PWM value corresponding to the angle */
    for (int i = STEERING_DUTY_RESOLUTION; i > 1; i--)
        total_duty *= 2;//exponentiation
    AngleBaseDutyNum = (float)CONFIG_STEERING_MIN_HIGH_TIME * (float)STEERING_BASE_FREQUENCY  
                                * (float)total_duty / (float)1000000; //Duty value corresponding to zero degree
    AngleMaxDutyNum = (float)CONFIG_STEERING_MAX_HIGH_TIME * (float)STEERING_BASE_FREQUENCY  
                                * (float)total_duty / (float)1000000; //Duty value corresponding to Max degree
    AngleStepLong = (AngleMaxDutyNum - AngleBaseDutyNum) / STEERING_ANGLE_SCOPE; //Duty value corresponding to one degree
    /* Initialize each steering gear parameter */
    for (int i = 0; i < STEERING_NUM; i++)
    {
        ESP_ERROR_CHECK(ledc_channel_config(&g_xSteering_manager.config_arr[i]));
        g_xSteering_manager.steering_arr[i].steering_name = i + 1;
        g_xSteering_manager.steering_arr[i].angle_now   = STEERING_DEFAULT_ANGLE;
        g_xSteering_manager.steering_arr[i].angle_scope = STEERING_ANGLE_SCOPE;
        g_xSteering_manager.steering_arr[i].channel     = g_xSteering_manager.config_arr[i].channel;
        g_xSteering_manager.steering_arr[i].speedmode   = g_xSteering_manager.config_arr[i].speed_mode;
        g_xSteering_manager.steering_arr[i].max_high_time = CONFIG_STEERING_MAX_HIGH_TIME;
        g_xSteering_manager.steering_arr[i].min_high_time = CONFIG_STEERING_MIN_HIGH_TIME;
        g_xSteering_manager.steering_arr[i].angle_base_dutyNum = AngleBaseDutyNum;
        g_xSteering_manager.steering_arr[i].angle_step_long    = AngleStepLong;
    }
    ESP_LOGI(TAG, "[init done!]");
}

//In calibration mode, Acceptance level continuous event
//
static void vSteering_Highlevel_updata(const uint32_t steering_number, 
                                       const uint32_t *new_max_time,
                                       const uint32_t *new_min_time) 
{
    xSteering_arguments_t *parguments = &g_xSteering_manager.steering_arr[steering_number - 1];

    if (new_min_time && !new_max_time)  
        parguments->min_high_time = *new_min_time;
    else if (!new_min_time && new_max_time)
        parguments->max_high_time = *new_max_time;
    else
        return;

    uint32_t total_duty = 2;
    float temp_AngleBaseDutyNum;
    float temp_AngleMaxDutyNum;
    float temp_AngleStepLong;
    for (int i = STEERING_DUTY_RESOLUTION; i > 1; i--)
        total_duty *= 2;//exponentiation

    temp_AngleBaseDutyNum = (float)parguments->min_high_time * (float)STEERING_BASE_FREQUENCY  
                                * (float)total_duty / (float)1000000; //Duty value corresponding to zero degree
    temp_AngleMaxDutyNum = (float)parguments->max_high_time * (float)STEERING_BASE_FREQUENCY  
                                * (float)total_duty / (float)1000000; //Duty value corresponding to Max degree
    temp_AngleStepLong = (temp_AngleMaxDutyNum - temp_AngleBaseDutyNum) / STEERING_ANGLE_SCOPE; //Duty value corresponding to one degree

    parguments->angle_base_dutyNum = temp_AngleBaseDutyNum;
    parguments->angle_max_dutyNum = temp_AngleMaxDutyNum;
    parguments->angle_step_long = temp_AngleStepLong;

    ESP_LOGI(TAG, "[Calibration update !]");
}

xSteering_manager_t* vSteering_init(void)
{
    ESP_ERROR_CHECK(ledc_timer_config(g_xSteering_manager.pPWM_timer));// config ledc_timer
    // Prepare and then apply the LEDC PWM channel configuration
    // change the configuration file if you want to set
    // Use up to five steering gears
    vSteering_channel_init();

    vSteering_DefaultAngle();

    return &g_xSteering_manager;
}

/* Angle converted to duty value */
static uint32_t iAngleToDutyNum(xSteering_arguments_t *parguments, const uint32_t angle)
{
    uint32_t DutyNum;

    if(angle > parguments->angle_scope)
        return (int)(parguments->angle_base_dutyNum + 45.0 * parguments->angle_step_long);
    
    DutyNum = (int)(parguments->angle_base_dutyNum + (float)angle * parguments->angle_step_long);

    return DutyNum;
}

//changing the PWM duty cycle by angle
void vSteering_ChangeAngle(xSteering_arguments_t *parguments, const uint32_t angle)
{
    parguments->angle_now = angle;

    ESP_ERROR_CHECK(ledc_set_duty(parguments->speedmode, parguments->channel, 
                                    iAngleToDutyNum(parguments, parguments->angle_now)));
    ESP_ERROR_CHECK(ledc_update_duty(parguments->speedmode, parguments->channel));
}

//changing the PWM duty cycle by dutyNum
void vSteering_ChangeDutyNum(xSteering_arguments_t *parguments, const uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(parguments->speedmode, parguments->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(parguments->speedmode, parguments->channel));
}

// steering gear returns to the default angle
void vSteering_DefaultAngle(void)
{
    for (int i = 0; i < STEERING_NUM; i++)
        vSteering_ChangeAngle(&g_xSteering_manager.steering_arr[i], STEERING_DEFAULT_ANGLE);
    ESP_LOGI(TAG, "[default angle!]");
}


void vSteering_Calibration(const uint32_t sreeringname, const uint32_t timeNum, const bool High_or_Low)
{
    vSteering_DefaultAngle();
    if (High_or_Low)
    {
        // updata max_high_time
        vSteering_Highlevel_updata(sreeringname, &timeNum, NULL);
        vSteering_ChangeAngle(&g_xSteering_manager.steering_arr[sreeringname - 1], g_xSteering_manager.steering_arr[sreeringname - 1].angle_scope);
    } else 
    {
        // updata min_high_time
        vSteering_Highlevel_updata(sreeringname, NULL, &timeNum);
        vSteering_ChangeAngle(&g_xSteering_manager.steering_arr[sreeringname - 1], 0);
    }
    
    
}