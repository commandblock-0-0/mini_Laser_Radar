#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/ledc.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 50 Hz
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

static uint32_t ledc_duty;
static uint32_t AngleBaseDutyNum;
static uint32_t AngleStepLong;
static const char* TAG = "ESP32S3_lOWER";
static ledc_timer_config_t my_ledc_timer;
static ledc_channel_config_t my_ledc_channel;

static void example_ledc_register(ledc_timer_config_t *ledc_timer, ledc_channel_config_t *ledc_channel)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer->speed_mode      = LEDC_MODE;
    ledc_timer->timer_num       = LEDC_TIMER;
    ledc_timer->duty_resolution  = LEDC_DUTY_RES;
    ledc_timer->freq_hz          = LEDC_FREQUENCY;  // Set output frequency at 50 Hz
    ledc_timer->clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel->speed_mode    = LEDC_MODE;
    ledc_channel->channel       = LEDC_CHANNEL;
    ledc_channel->timer_sel     = LEDC_TIMER;
    ledc_channel->intr_type     = LEDC_INTR_DISABLE;
    ledc_channel->gpio_num      = LEDC_OUTPUT_IO;
    ledc_channel->duty          = 0; // Set duty to 0%
    ledc_channel->hpoint        = 0;
    ESP_ERROR_CHECK(ledc_channel_config(ledc_channel));
}

static void example_ledc_init(ledc_timer_config_t *ledc_timer, ledc_channel_config_t *ledc_channel)
{
    ledc_timer_config(ledc_timer);
    ledc_channel_config(ledc_channel);
}

/* 角度化为占空比数值 */
/* 输入角度在0°到180°之间 */
/*
 *对应关系
 *20ms(50Hz)基时脉冲
 *高电平时间：
 *0.5ms-----2.5%------0°
 *1.0ms-----5.0%------45°
 *1.5ms-----7.5%------90°
 *2.0ms-----10.0%-----135°
 *2.5ms-----12.5%-----180°
*/
static int AngleToDutyNum(int angle)
{
    int DutyNum;

    if(angle < 0 || angle >180)
        return -1;
    
    DutyNum = AngleBaseDutyNum + angle * AngleStepLong;

    return DutyNum;
}

static void vTask1(void *data)
{
    uint32_t i = 1;
    bool les_status = true;

    printf("Task1 is running!\n");

    ledc_set_duty(my_ledc_channel.speed_mode,
                  my_ledc_channel.channel,
                  i);
    ledc_update_duty(my_ledc_channel.speed_mode, my_ledc_channel.channel);

    while (1)
    {
        if (les_status)
            i++;
        else
            i--;

        ledc_set_duty(my_ledc_channel.speed_mode,
                      my_ledc_channel.channel,
                      i);
        ledc_update_duty(my_ledc_channel.speed_mode, my_ledc_channel.channel);
        
        if (i == 301 || i == 0)
        {    
            les_status = !les_status;
            printf("loop!\n");
        }   

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void vTask2(void *data)
{
    int i = 0;
    bool angle_status = true;

    while (1)
    {
        if (i < 0 || i > 180)
        {
            angle_status = !angle_status;
            printf("loop!\n");
        }

        ledc_set_duty(my_ledc_channel.speed_mode,
                        my_ledc_channel.channel, AngleToDutyNum(i));
        ledc_update_duty(my_ledc_channel.speed_mode, my_ledc_channel.channel);

        if (angle_status)
            i++;
        else
            i--;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    printf("here!\n");
    ledc_duty = (uint32_t)pow((float)2.0, LEDC_DUTY_RES);
    AngleBaseDutyNum = (uint32_t)((float)ledc_duty * 0.025);
    AngleStepLong = (uint32_t)((float)0.025 / 45 * ledc_duty);

    example_ledc_register(&my_ledc_timer, &my_ledc_channel);
    example_ledc_init(&my_ledc_timer, &my_ledc_channel);
    //xTaskCreate(vTask1, "Task1", 2000, NULL, 5, NULL);
    xTaskCreate(vTask2, "Task2", 2000, NULL, 5, NULL);

    return;
}