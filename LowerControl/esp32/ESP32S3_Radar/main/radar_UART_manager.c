#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "radar_UART.h"

#define RADAR_UART_NUM CONFIG_RADAR_UART_PORT_NUM

static const char *TAG = "Radar_uart";

static void vRadar_UART_Device_Init(void);
static void vRadar_UART_Device_Exit(void);

static QueueHandle_t g_xRadar_uart_queue;
static pRadar_UART_DataHand_t g_Radar_UART_DataHand = NULL;
static uint8_t* g_pcDataBuff = NULL;

static uart_config_t g_xRadar_uart_config = {
    .baud_rate  = CONFIG_RADAR_UART_BAUD_RATE,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
static xRadar_UART_t g_xRadar_uart_Opr = {
    .uart_num        = RADAR_UART_NUM,
    .pUart_config    = &g_xRadar_uart_config,
    .UART_Init       = vRadar_UART_Device_Init,
    .UART_Exit       = vRadar_UART_Device_Exit,
    .handle_receive_task = NULL,
};

//init function, according to the kconfig file
//app is not visible
static void vRadar_UART_Device_Init(void)
{
    g_xRadar_uart_Opr.uart_queue = g_xRadar_uart_queue;
    ESP_ERROR_CHECK(uart_param_config(RADAR_UART_NUM, g_xRadar_uart_Opr.pUart_config));//Set uart parameters
    ESP_ERROR_CHECK(uart_set_pin(RADAR_UART_NUM, CONFIG_RADAR_UART_TXD, CONFIG_RADAR_UART_RXD,
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));//Set uart pin
    ESP_ERROR_CHECK(uart_driver_install(RADAR_UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 20, 
                            &g_xRadar_uart_Opr.uart_queue, 0));//Install Driver
    ESP_ERROR_CHECK(uart_pattern_queue_reset(RADAR_UART_NUM, 20));//Reset the pattern queue length to record at most 20 pattern positions.

    ESP_LOGI(TAG, "UART Init done\n");
}

//exit function
//app is not visible
static void vRadar_UART_Device_Exit(void)
{
    vTaskDelete(g_xRadar_uart_Opr.handle_receive_task);
    ESP_ERROR_CHECK(uart_driver_delete(RADAR_UART_NUM));
    free(g_pcDataBuff);
}

// event processing task
// created by default when the app calls the run function
// use the event queue officially provided by ESP
// based on official examples: examples/peripherals/uart/uart_events
static void Radar_uart_default_receive_task(void *pvParameters)
{
    uart_event_t event;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(g_xRadar_uart_Opr.uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(g_pcDataBuff, RX_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", RADAR_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    uart_read_bytes(RADAR_UART_NUM, g_pcDataBuff, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[Recv str]: %s", (const char*) g_pcDataBuff);
                    (*g_Radar_UART_DataHand)(g_pcDataBuff, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(RADAR_UART_NUM);
                    xQueueReset(g_xRadar_uart_Opr.uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(RADAR_UART_NUM);
                    xQueueReset(g_xRadar_uart_Opr.uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
}

static void vpRadar_UART_default_DataHand(uint8_t* dtmp, size_t size)
{
    
}

// Used by app 
// Run function 
xRadar_UART_t* radar_UART_Run(TaskFunction_t UART_receive_task, 
                              pRadar_UART_DataHand_t Radar_UART_DataHand)
{
    g_xRadar_uart_Opr.UART_Init();

    g_pcDataBuff =(uint8_t*) malloc(RX_BUF_SIZE);
    
    if (Radar_UART_DataHand)// can provide your own receive function,  if provide NULL, use the default function
        g_Radar_UART_DataHand = Radar_UART_DataHand;
    else
        g_Radar_UART_DataHand = vpRadar_UART_default_DataHand;

    if (UART_receive_task)// can provide your own receive task,  if provide NULL, use the default task
        xTaskCreate(UART_receive_task, "radar_uart_event_task", CONFIG_RADAR_TASK_STACK_SIZE, 
                        NULL, 12, &(g_xRadar_uart_Opr.handle_receive_task));
    else
        xTaskCreate(Radar_uart_default_receive_task, "radar_uart_event_task", CONFIG_RADAR_TASK_STACK_SIZE, 
                        NULL, 12, &(g_xRadar_uart_Opr.handle_receive_task));

    return &g_xRadar_uart_Opr;
}
