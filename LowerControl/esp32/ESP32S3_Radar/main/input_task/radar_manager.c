#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"

#include "atk_ms53l0m.h"

#include "sdkconfig.h"
#include "mod_bus.h"
#include "radar_manager.h"
#include "radar_uart.h"
#include "steering_control.h"
#include "steering_task.h"

#define MODBUS_UART 1
#define ATK_MS53L0M_UART 2

static const char* TAG = "RadarManager";

static Radar_status g_Radar_status;

static uint8_t get_UART_baudrate_to_settings(uart_port_t uart_num);
static esp_err_t Processing_Funcode_0_write_data(void);
static esp_err_t Processing_Funcode_5_write_data(void);
static void steering_Task_run(void);
static void steering_Task_Suspend(void);
static void steering_Task_reset(void);
static void steering_Task_Specify_Angle(void);
static void steering_Task_calibration(uart_port_t uart_num, int32_t* command_code);

/**
 * @brief       init hardware driver,init Modbus
 * @param       void
 * 
 * @retval      ESP_OK              No error
 * @retval      ESP_FAIL            Core error
 * @retval      ESP_ERR_NOT_FOUND   Requested resource not found
*/
esp_err_t Radar_manager_init(void)
{
    esp_err_t err;

    g_Radar_status.Task_EventGroup = xEventGroupCreate();
    if (g_Radar_status.Task_EventGroup == NULL)
        return ESP_FAIL; /* EventGroup create fail */

    g_Radar_status.Uart_listHand = radar_UART_Run(HIGH_PRIORITY, NULL, NULL);
    if ( g_Radar_status.Uart_listHand == NULL)
        return ESP_ERR_NOT_FOUND; /* UART init error */
    
    err = Modbus_init(MODBUS_UART);
    if (err)
        return ESP_ERR_NOT_FOUND; /* UART number err */

    g_Radar_status.p_uart_data = Modbus_Get_rx_Data_Address(); /* get receive address */
    if (g_Radar_status.p_uart_data == NULL)
        return ESP_ERR_NOT_FOUND;

    g_Radar_status.p_steering = vSteering_init(); /* init steering */

    err = atk_ms53l0m_init(ATK_MS53L0M_UART, &g_Radar_status.Measurement_sensor_address); /* init measure sensor */
    if (err != ATK_MS53L0M_EOK)
        return ESP_FAIL;

    /* Task Creat */
    /* Steering Task Create */
    xTaskCreatePinnedToCore(Radar_Steering_task, 
                            "Steering Task", 
                            2048, 
                            &g_Radar_status, 
                            MIDDLE_PRIORITY, 
                            &g_Radar_status.Steering_task_Handle, 
                            1);

    /* ATK_MS53L0M measure Task Create */
    xTaskCreatePinnedToCore(Radar_input_measure_Task,
                            "measure Task",
                            2048,
                            &g_Radar_status,
                            MIDDLE_PRIORITY,
                            &g_Radar_status.input_measure_Task_Handle,
                            1);

    /* Data Execution Task Create */
    xTaskCreatePinnedToCore(Radar_input_Execution_Task,
                            "input Execution Task", 
                            2048, 
                            NULL, 
                            MIDDLE_PRIORITY, 
                            &g_Radar_status.input_Execution_Task_Handle, 
                            0);

    return ESP_OK;
}

/**
 * @brief       Perform operations based on data, blocking before obtaining data
 *
 * @param       xTicksToWait: wait time
 * 
 * @retval      ESP_OK                  : Obtain data and complete processing
 * @retval      ESP_ERR_TIMEOUT         : time out
*/
esp_err_t Radar_manager_Modbus_carry_out(TickType_t xTicksToWait)
{
    if ( pdTRUE == xSemaphoreTake(g_Radar_status.p_uart_data->xBinarySemaphore, xTicksToWait) )
    {
        g_Radar_status.p_uart_data->status = false; /* Processing data */

        if ( g_Radar_status.p_uart_data->opt_type == MODBUS_OPT_READ ) { /* Read operation */
            switch ( g_Radar_status.p_uart_data->fun_code )
            {
                case (uint8_t)MODBUS_FUNCODE_SYS:
                    printf("READ System settings.");
                    Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_OPR); /* Reading system settings is meaningless */
                    break;

                case (uint8_t)MODBUS_FUNCODE_SCANRATE:
                    printf("READ Scan ratt.");
                    Modbus_back_read_message(MODBUS_FUNCODE_SCANRATE, 1, (uint16_t)g_Radar_status.scan_rate); /* return scan rate */
                    break;

                case (uint8_t)MODBUS_FUNCODE_BAUDRATE:
                    printf("READ BPS rate setting.");
                    uint8_t baudrateCODE = get_UART_baudrate_to_settings(g_Radar_status.p_uart_data->uart_num);
                    Modbus_back_read_message(MODBUS_FUNCODE_BAUDRATE, 1, (uint16_t)baudrateCODE); /* return BPS rate */
                    break;

                case (uint8_t)MODBUS_FUNCODE_IDSET:
                    printf("READ Device Address Settings.");
                    Modbus_back_read_message(MODBUS_FUNCODE_IDSET, 2, g_Radar_status.p_uart_data->device_address);
                    break;

                case (uint8_t)MODBUS_FUNCODE_APPOINTDATA:
                    printf("READ Obtain specified azimuth data.");
                    break;

                case (uint8_t)MODBUS_FUNCODE_WORKMODE:
                    printf("READ Work mode.");
                    Modbus_back_read_message(MODBUS_FUNCODE_WORKMODE, 1, (uint16_t)g_Radar_status.work_mode);
                    break;

                case (uint8_t)MODBUS_FUNCODE_MEASUREMODE:
                    printf("READ Measurement mode settings.");
                    Modbus_back_read_message(MODBUS_FUNCODE_MEASUREMODE, 1, (uint16_t)g_Radar_status.Measure_mode);
                    break;

                case (uint8_t)MODBUS_FUNCODE_CALIMODE:
                    printf("READ Calibration Mode.");
                    break;

                default:
                    printf("READ ERROR!");
                    Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_FUNCODE);
                    break;
            }
        } else if ( g_Radar_status.p_uart_data->opt_type == MODBUS_OPT_WRITE ) { /* Write operation */
            switch ( g_Radar_status.p_uart_data->fun_code )
            {
                /* 0x00 System settings */
                case (uint8_t)MODBUS_FUNCODE_SYS:
                    ESP_LOGI(TAG, "[WRITE System settings]");
                    if (Processing_Funcode_0_write_data()) {
                        Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_DATA);
                    } else {
                        Modbus_back_write_message(MODBUS_FUNCODE_SYS);
                    }
                    break;
                /*  */
                case (uint8_t)MODBUS_FUNCODE_SCANRATE:
                    printf("WRITE Scan ratt.");
                    break;

                case (uint8_t)MODBUS_FUNCODE_BAUDRATE:
                    printf("WRITE BPS rate setting.");
                    break;

                case (uint8_t)MODBUS_FUNCODE_IDSET:
                    printf("WRITE Device Address Settings.");
                    break;
                /* 0x05 Obtain specified azimuth data */
                case (uint8_t)MODBUS_FUNCODE_APPOINTDATA:
                    printf("WRITE Obtain specified azimuth data.");
                    if (Processing_Funcode_5_write_data()) /* data error */
                        Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_DATA);
                    else 
                        steering_Task_Specify_Angle(); /* Special here, returning a read message, include 2 bytes measure data */
                    break;
                /*  */
                case (uint8_t)MODBUS_FUNCODE_WORKMODE:
                    printf("WRITE Work mode.");
                    break;

                case (uint8_t)MODBUS_FUNCODE_MEASUREMODE:
                    printf("WRITE Measurement mode settings.");
                    break;

                case (uint8_t)MODBUS_FUNCODE_CALIMODE:
                    printf("WRITE Calibration Mode.");
                    break;

                default:
                    printf("WRITE ERROR!");
                    Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_FUNCODE);
                    break;

            }
        }

        g_Radar_status.p_uart_data->status = true; /* Data processing completed */
        return ESP_OK;
    } else {
        printf("Modbus timeout!");
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief       Obtain UART baud rate and convert it to Modbus parameter
 * 
 * @param       uart_num    UART port number
 * 
 * @retval      BPS settings parameters
*/
static uint8_t get_UART_baudrate_to_settings(uart_port_t uart_num)
{
    uint32_t baudrate;
    uart_get_baudrate(uart_num, &baudrate);
    switch (baudrate)
    {
        case 2400:
            return MODBUS_BAUDRATE_2400;
        case 4800:
            return MODBUS_BAUDRATE_4800;
        case 9600:
            return MODBUS_BAUDRATE_9600;
        case 19200:
            return MODBUS_BAUDRATE_19200;
        case 38400:
            return MODBUS_BAUDRATE_38400;
        case 57600:
            return MODBUS_BAUDRATE_57600;
        case 115200:
            return MODBUS_BAUDRATE_115200;
        case 230400:
            return MODBUS_BAUDRATE_230400;
        case 460800:
            return MODBUS_BAUDRATE_460800;
        case 921600:
            return MODBUS_BAUDRATE_921600;
        default:
            return MODBUS_OPT_ERROR;
    }
}

/**
 * @brief       When receiving the 0x00 function code, this function processes the data within it
 * @retval      ESP_FAIL: data error
 * @retval      ESP_OK: OK
*/
static esp_err_t Processing_Funcode_0_write_data(void)
{
    if (g_Radar_status.p_uart_data->len != 1)
        return ESP_FAIL;
    else {
        if (g_Radar_status.p_uart_data->buf[0] == MODBUS_SYS_RUN) {
            steering_Task_run();
            return ESP_OK;
        } else if (g_Radar_status.p_uart_data->buf[0] == MODBUS_SYS_PARAM_RESET) {
            return ESP_OK;
        } else if (g_Radar_status.p_uart_data->buf[0] == MODBUS_SYS_RESET) {
            steering_Task_reset();
            return ESP_OK;
        } else if (g_Radar_status.p_uart_data->buf[0] == MODBUS_SYS_SUSPEND) {
            steering_Task_Suspend();
            return ESP_OK;
        } else 
            return ESP_FAIL;
    }
}

/**
 * @brief       When receiving the 0x05 function code, this function processes the data within it
 * @retval      ESP_FAIL: data error
 * @retval      ESP_OK: OK
*/
static esp_err_t Processing_Funcode_5_write_data(void)
{
    if (g_Radar_status.p_uart_data->len % 2 || g_Radar_status.p_uart_data->len > 10) /* Every 2 bytes describe a servo angle */
        return ESP_FAIL;

    uint8_t steering_name;
    uint16_t steering_angle;
    for (int i = 0; i < g_Radar_status.p_uart_data->len; i += 2)
    {   
        /* The high 7 bits indicate the steering gear number */
        steering_name = g_Radar_status.p_uart_data->buf[i] >> 1; 
        if (steering_name > g_Radar_status.p_steering->steering_totalNum - 1) 
        {
            /* The steering gear does not exist */
            return ESP_FAIL;
        }
        /* High 9 bits indicate angle */
        steering_angle = (((uint16_t)g_Radar_status.p_uart_data->buf[i] & 1) << 8) + (uint16_t)g_Radar_status.p_uart_data->buf[i + 1];
        if (steering_angle > g_Radar_status.p_steering->steering_arr[steering_name].angle_scope)
        {
            /* Exceeding maximum angle */
            return ESP_FAIL;
        }
        g_Radar_status.p_steering->steering_arr[steering_name].angle_now = steering_angle;
    }
    /* Special here, returning a read message, include 2 bytes measure data */
    return ESP_OK;
}

/**
 * @brief       Pause Task
*/
static void steering_Task_Suspend(void)
{
    // Notify the steering task to suspend itself
    if (g_Radar_status.Steering_task_Handle)
    {
        vTaskResume(g_Radar_status.Steering_task_Handle); /* The task may be delayed and needs to be awakened */
        xTaskNotify(g_Radar_status.Steering_task_Handle, STEERING_TASK_SUSPEND, eSetValueWithOverwrite);
    }
}

/**
 * @brief       Run Task
*/
static void steering_Task_run(void)
{
    // Notify the steering task to run
    if (g_Radar_status.Steering_task_Handle)
    {
        vTaskResume(g_Radar_status.Steering_task_Handle); /* The task may be delayed and needs to be awakened */
        xTaskNotify(g_Radar_status.Steering_task_Handle, STEERING_TASK_RUN, eSetValueWithOverwrite);
    }
}

/**
 * @brief       Reset Task
*/
static void steering_Task_reset(void)
{
    // Notify the steering task to reset
    if (g_Radar_status.Steering_task_Handle)
    {    
        vTaskResume(g_Radar_status.Steering_task_Handle); /* The task may be delayed and needs to be awakened */
        xTaskNotify(g_Radar_status.Steering_task_Handle, STEERING_TASK_RESET, eSetValueWithOverwrite);
    }
}

/**
 * @brief       Specify special orientation Specify a special orientation to obtain distance
*/
static void steering_Task_Specify_Angle(void)
{
    // Notify the steering task to reset
    if (g_Radar_status.Steering_task_Handle)
    {    
        vTaskResume(g_Radar_status.Steering_task_Handle); /* The task may be delayed and needs to be awakened */
        xTaskNotify(g_Radar_status.Steering_task_Handle, STEERING_TASK_SPECIAL, eSetValueWithOverwrite);
        uint32_t retval = xEventGroupWaitBits(g_Radar_status.Task_EventGroup, 0x20, pdTRUE, pdTRUE, pdMS_TO_TICKS(50));
        if ((retval & 0x20) == 0)
            Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_DEVICE);
        else
            Modbus_back_read_message(MODBUS_FUNCODE_APPOINTDATA, 2, g_Radar_status.measure_data);
    }
}

/**
 * @brief       Calibration Mode
*/
static void steering_Task_calibration(uart_port_t uart_num, int32_t* command_code)
{

}
