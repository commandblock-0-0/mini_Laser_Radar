#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "radar_manager.h"
#include "uart.h"
#include "steering_control.h"


static const char *TAG = "RadarUART";

static esp_err_t vRadar_UART_Device_Exit(uart_port_t uart_num);

static xRadar_UART_t* g_Uart_listHand = NULL;

//init function, according to the kconfig file
//app is not visible
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

//exit function
//app is not visible
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

//Convert commands to numeric strings
static void vCommandDecode(char* command, int32_t* command_code)
{
    /* All non-numbers parameters are converted to zero */
    uint32_t count = 1;
    char *token;
    char *rest = command;
    while ((token = strtok_r(NULL, " ", &rest)))
    {
        command_code[count] = atoi(token);
        count++;
        if (count == (RADAR_MAX_COMMAND_LEN + 1))
            break;
    }
    command_code[0] = count - 1;// Number of instructions for this command
}

static void vpRadar_UART_default_DataHand(const uart_port_t uart_num, char* command, size_t size)
{
    /*
     *default function to process data
     *the data sent should start with a command, followed by several data, separated by spaces :
     *<command> [data 0] [data 1] ...
     */
    char* str;
    int32_t Command_code[RADAR_MAX_COMMAND_LEN + 1];
    vCommandDecode(command, Command_code);//convert commands to numeric strings

    switch(Command_code[1]) { //Parse the first substring
        // stop command
        case RADAR_SUSPEND:
            ESP_LOGI(TAG, "Radar Suspend.");
            str = "Radar Suspend !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            // To suspend runningtask
            vRadarManager_Task_Suspend();
            break;
        // run command
        case RADAR_RUN:
            ESP_LOGI(TAG, "Radar start.");
            str = "Radar start !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            // To ready runningtask
            vRadarManager_Task_run();
            break;
        // choose mode command
        case RADAR_MOD: 
            ESP_LOGI(TAG, "Choose radar mode.");
            str = "Choose radar mode !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            // To suspend runningtask
            break;
        // start angle calibration command
        case RADAR_ANGLE_CALIBRATION: 
            // <RADAR_MOD> <sreeringNum> <timeNum> <H/L>
            ESP_LOGI(TAG, "Radar calibration mode.");
            vRadarManager_enable_calibration(uart_num, Command_code);
            break;
        //reset the status and stops
        case RADAR_RESET:
            ESP_LOGI(TAG, "Radar stop and reset action.");
            str = "Radar stop and reset action !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            vRadarManager_Task_Stop();
            break;
        //specify the angle of a single steering gear
        case SPECIFY_ANGLE:
            // <RADAR_MOD> <sreeringNum> <angle>
            ESP_LOGI(TAG, "Specify Angle.");
            str = "Specify Angle !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            vRadarManager_Specify_Angle(uart_num, Command_code);
            break;
        // other
        default:
            ESP_LOGW(TAG, "Command is not available !");
            str = "Command is not available !\n";
            uart_write_bytes(uart_num, str, strlen(str));
            break;
    }
}

// Used by app 
// Run function 
xRadar_UART_t* radar_UART_Run(TaskFunction_t UART_receive_task, 
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
        strcpy(task_name, "radar_uart$_event_task");
        for (int i = 0; i < name_len; i++)
        {
            if (task_name[i] == '$')
            {    
                task_name[i] = 0x30 + pfirse_note->uart_num;
                break;
            }
        }
        // can provide your own receive task,  if provide NULL, use the default task
        if (UART_receive_task)
            xTaskCreatePinnedToCore(UART_receive_task, "radar_uart_event_task", pfirse_note->usStackDepth, 
                            pfirse_note, 12, &(pfirse_note->handle_receive_task), 0);
        else
            xTaskCreatePinnedToCore(Radar_uart_default_receive_task, "radar_uart_event_task", pfirse_note->usStackDepth, 
                            pfirse_note, 12, &(pfirse_note->handle_receive_task), 0);
        ESP_LOGI(TAG, "[%s is running!]", task_name);
        pfirse_note = pfirse_note->ptNext;
    }
    ESP_LOGI(TAG, "[runing!]");

    return g_Uart_listHand;
}
