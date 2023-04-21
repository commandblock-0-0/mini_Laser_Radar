#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "radar_manager.h"
#include "radar_uart.h"
#include "steering_control.h"


static const char *TAG = "RadarUART";

static esp_err_t vRadar_UART_Device_Exit(uart_port_t uart_num);

static xRadar_UART_t* g_Uart_listHand = NULL;

/**
 * @brief       Init UART
 * @param       void
 * 
 * @retval      NULL   : uart number is invalid
 * @retval      other  : link header connecting all UART items
 */
static xRadar_UART_t* vRadar_UART_Device_Init(void)
{
    xRadar_UART_t* uart_p = NULL; 
    xRadar_UART_t* Uart_listHand = NULL;
#ifdef CONFIG_RADAR_USING_UART2
    //uart3 device list
    uart_p = malloc(sizeof(xRadar_UART_t));
    if (!uart_p)
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    uart_config_t uart3_config = {
        .baud_rate  = CONFIG_RADAR_UART2_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_p->uart_num      = CONFIG_RADAR_UART2_PORT_NUM;
    uart_p->Uart_config   = uart3_config;
    uart_p->usStackDepth  = CONFIG_RADAR_UART2_TASK_STACK_SIZE;
    uart_p->rx_io_num     = CONFIG_RADAR_UART2_RXD;
    uart_p->tx_io_num     = CONFIG_RADAR_UART2_TXD;
    uart_p->UART_Exit     = vRadar_UART_Device_Exit;
    uart_p->DateHand_fun  = NULL;
    uart_p->handle_receive_task = NULL;
    //Linked List
    uart_p->ptNext = Uart_listHand;
    Uart_listHand = uart_p;
#endif
#ifdef CONFIG_RADAR_USING_UART1
    //uart2 device list
    uart_p = malloc(sizeof(xRadar_UART_t));
    if (!uart_p)
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    uart_config_t uart2_config = {
        .baud_rate  = CONFIG_RADAR_UART1_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_p->uart_num      = CONFIG_RADAR_UART1_PORT_NUM;
    uart_p->Uart_config   = uart2_config;
    uart_p->usStackDepth  = CONFIG_RADAR_UART1_TASK_STACK_SIZE;
    uart_p->rx_io_num     = CONFIG_RADAR_UART1_RXD;
    uart_p->tx_io_num     = CONFIG_RADAR_UART1_TXD;
    uart_p->UART_Exit     = vRadar_UART_Device_Exit;
    uart_p->DateHand_fun  = NULL;
    uart_p->handle_receive_task = NULL;
    //Linked List
    uart_p->ptNext = Uart_listHand;
    Uart_listHand = uart_p;
#endif
#ifdef CONFIG_RADAR_USING_UART0
    //uart1 device list
    uart_p = malloc(sizeof(xRadar_UART_t));
    if (!uart_p)
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    uart_config_t uart1_config = {
        .baud_rate  = CONFIG_RADAR_UART0_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_p->uart_num      = CONFIG_RADAR_UART0_PORT_NUM;
    uart_p->Uart_config   = uart1_config;
    uart_p->usStackDepth  = CONFIG_RADAR_UART0_TASK_STACK_SIZE;
    uart_p->rx_io_num     = CONFIG_RADAR_UART0_RXD;
    uart_p->tx_io_num     = CONFIG_RADAR_UART0_TXD;
    uart_p->UART_Exit     = vRadar_UART_Device_Exit;
    uart_p->DateHand_fun  = NULL;
    uart_p->handle_receive_task = NULL;
    //Linked List
    uart_p->ptNext = Uart_listHand;
    Uart_listHand = uart_p;
#endif
    if (Uart_listHand == NULL)
        return NULL;
    QueueHandle_t* uart_queue_p;
    while (uart_p) //init all uart
    {
        uart_queue_p = malloc(sizeof(QueueHandle_t));
        uart_p->pUart_queue = uart_queue_p;
        ESP_ERROR_CHECK(uart_param_config(uart_p->uart_num, &(uart_p->Uart_config)));
        ESP_ERROR_CHECK(uart_set_pin(uart_p->uart_num, uart_p->tx_io_num, uart_p->rx_io_num, 
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK(uart_driver_install(uart_p->uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 20, 
                                uart_p->pUart_queue, 0));
        ESP_ERROR_CHECK(uart_pattern_queue_reset(uart_p->uart_num, 20));
        ESP_LOGI(TAG, "[UART%d Init done!]", uart_p->uart_num);

        uart_p = uart_p->ptNext;
    }
    return Uart_listHand;
}

/**
 * @brief       Exit UART
 * @param       uart_num : uart number
 * 
 * @retval      ESP_ERR_NOT_FOUND   : uart number is invalid
 * @retval      ESP_OK              : success
 */
static esp_err_t vRadar_UART_Device_Exit(const uart_port_t uart_num)
{
    xRadar_UART_t* pfirst_note = g_Uart_listHand;
    xRadar_UART_t* pnote = NULL;
    if (pfirst_note->uart_num == uart_num)
    {
        g_Uart_listHand = pfirst_note->ptNext;
        vTaskDelete(pfirst_note->handle_receive_task);
        ESP_ERROR_CHECK(uart_driver_delete(pfirst_note->uart_num));
        free(pfirst_note->pUart_queue);
        free(pfirst_note);

        return ESP_OK;
    } else {
        pnote = pfirst_note;
        pfirst_note = pfirst_note->ptNext;
    }

    while (pfirst_note)
    {
        if (pfirst_note->uart_num == uart_num)
        {
            pnote->ptNext = pfirst_note->ptNext;
            vTaskDelete(pfirst_note->handle_receive_task);
            ESP_ERROR_CHECK(uart_driver_delete(pfirst_note->uart_num));
            free(pfirst_note->pUart_queue);
            free(pfirst_note);

            return ESP_OK;
        }
        pnote = pfirst_note;
        pfirst_note = pfirst_note->ptNext;
    }

    return ESP_ERR_NOT_FOUND;
}

/**
 * @brief       default processing function after receiving valid data, 
 * @param       uart_num : uart number
 * @param       dtmp     : received data
 * @param       size     : data size
 * 
 * @retval      void
 */
static void vpRadar_UART_default_DataHand(const uart_port_t uart_num, uint8_t* dtmp, size_t size)
{
    /* The radar is not activated, no operation will be performed */
    uart_write_bytes(uart_num, "Radar not activated", sizeof("Radar not activated"));
}

/**
 * @brief       Start UART
 * @param       UART_receive_task      : provide uart task, in NULL using default function, Task priority is 12
 * @param       Radar_UART_DataHand    : processing function after receiving valid data, in NULL using default function
 * 
 * @retval      NULL  : init failed
 * @retval      other : link header connecting all UART items
 */
xRadar_UART_t* radar_UART_Run(UBaseType_t uxPriority,
                              TaskFunction_t UART_receive_task, 
                              pRadar_UART_DataHand_t Radar_UART_DataHand)
{
    if (g_Uart_listHand)
        return g_Uart_listHand;
    else
        g_Uart_listHand = vRadar_UART_Device_Init();
    if (g_Uart_listHand == NULL)
        return NULL;

    xRadar_UART_t* pfirse_note = g_Uart_listHand;
    uint32_t name_len= strlen("radar_uart$_event_task");
    char task_name[name_len + 1];

    while (pfirse_note)
    {
        // can provide your own receive function,  if provide NULL, use the default function
        if (Radar_UART_DataHand)
            pfirse_note->DateHand_fun = Radar_UART_DataHand;
        else
            pfirse_note->DateHand_fun = vpRadar_UART_default_DataHand;
        // build task name
        strcpy(task_name, "radar_uart$_event_task\0");
        for (int i = 0; i < name_len; i++)
        {
            if (task_name[i] == '$')
            {    
                task_name[i] = 0x30 + pfirse_note->uart_num;//Determine the task name for each UART
                break;
            }
        }
        // can provide your own receive task,  if provide NULL, use the default task
        if (UART_receive_task)
            xTaskCreatePinnedToCore(UART_receive_task, task_name, pfirse_note->usStackDepth, 
                            pfirse_note, uxPriority, &(pfirse_note->handle_receive_task), 0);
        else
            xTaskCreatePinnedToCore(Radar_uart_default_receive_task, task_name, 10000,//pfirse_note->usStackDepth, 
                            pfirse_note, uxPriority, &(pfirse_note->handle_receive_task), 0);
        ESP_LOGI(TAG, "[%s is running!]", task_name);
        pfirse_note = pfirse_note->ptNext;
    }
    ESP_LOGI(TAG, "[runing!]");

    return g_Uart_listHand;
}

/**
 * @brief       find UART structures by uart_num
 * @param       uart_num      : uart number
 * 
 * @retval      NULL  : no find
 * @retval      other : Structure found
 */
xRadar_UART_t* radar_UART_Find_by_Num(const uart_port_t uart_num)
{
    xRadar_UART_t* pfirse_note = g_Uart_listHand;
    while (pfirse_note)
    {
        if(pfirse_note->uart_num == uart_num)
            return pfirse_note;
        else
            pfirse_note = pfirse_note->ptNext;
    }
    return NULL;
}

/**
 * @brief       Change the processing function
 * @param       uart_num      : uart number that needs to be changed
 * @param       UART_DataHand : processing function
 * 
 * @retval      ESP_OK  : success
 * @retval      ESP_FAIL: failure
 */
esp_err_t radar_UART_ChangeFunbyNum(const uart_port_t uart_num, pRadar_UART_DataHand_t UART_DataHand)
{
    xRadar_UART_t* UART_note;

    if (!uart_is_driver_installed(uart_num))
    {
        ESP_LOGW(TAG,"UART %d no install!", uart_num);
        return ESP_FAIL;
    }

    UART_note = radar_UART_Find_by_Num(uart_num);
    if (!UART_note)
        return ESP_FAIL;
    else
    {
        uart_flush_input(UART_note->uart_num);
        UART_note->DateHand_fun = UART_DataHand;
        return ESP_OK;
    }
}