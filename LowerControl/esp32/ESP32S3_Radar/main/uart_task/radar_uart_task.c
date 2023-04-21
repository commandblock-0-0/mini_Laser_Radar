#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "radar_uart.h"

static const char *TAG = "RadarUART";
static uint8_t pcDataBuff[RX_BUF_SIZE] = {0};

// event processing task
// created by default when the app calls the run function
// use the event queue officially provided by ESP
// based on official examples: examples/peripherals/uart/uart_events
void Radar_uart_default_receive_task(void *pxRadar_uart_Opr)
{
    const xRadar_UART_t* const pxUart_Opr = (xRadar_UART_t*)pxRadar_uart_Opr;
    uart_event_t event;
    for(;;) {
        bzero(pcDataBuff, RX_BUF_SIZE);
        //Waiting for UART event.
        if(xQueueReceive(*(pxUart_Opr->pUart_queue), (void * )&event, (TickType_t)portMAX_DELAY)) {
            ESP_LOGI(TAG, "uart[%d] event:", pxUart_Opr->uart_num);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                    uart_read_bytes(pxUart_Opr->uart_num, pcDataBuff, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[Recv str]: %s, [Len]: %d", (const char*) pcDataBuff, event.size);
                    (pxUart_Opr->DateHand_fun)((pxUart_Opr->uart_num), pcDataBuff, event.size);//Execute the corresponding processing function
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(pxUart_Opr->uart_num);
                    xQueueReset(*(pxUart_Opr->pUart_queue));
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(pxUart_Opr->uart_num);
                    xQueueReset(*(pxUart_Opr->pUart_queue));
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
