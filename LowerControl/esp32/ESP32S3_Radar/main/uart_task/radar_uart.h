#ifndef _RADAR_UART_H_
#define _RADAR_UART_H_

#include "freertos/queue.h"
#include "driver/uart.h"

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024
#define RADAR_MAX_COMMAND_LEN 10

typedef void(* pRadar_UART_DataHand_t)(const uart_port_t uart_num, uint8_t* dtmp, size_t size);

typedef enum {
    RADAR_SUSPEND = 1,//radar does not reset the status and stops working 
    RADAR_RUN,
    RADAR_MOD,  //change radar work mode
    RADAR_ANGLE_CALIBRATION,// The angle of each steering gear may be different. This command opens the calibration mode
    RADAR_RESET, //stops and status
    SPECIFY_ANGLE,//specify the angle of a single steering gear
} Radar_uart_command_t;

//defined UART itself 
typedef struct xRadar_UART_t{
    uart_port_t uart_num;        //uart num
    uart_config_t Uart_config; //uart config, most configurations come from Kconfig
    QueueHandle_t* pUart_queue;    //Interrupt event queue
    uint32_t usStackDepth;     //task stack depth
    int tx_io_num;
    int rx_io_num;

    esp_err_t(* UART_Exit)(uart_port_t); //exit function

    pRadar_UART_DataHand_t DateHand_fun; //
    TaskHandle_t handle_receive_task;//uart event task handle

    struct xRadar_UART_t* ptNext;
} xRadar_UART_t;


xRadar_UART_t* radar_UART_Run(UBaseType_t uxPriority, TaskFunction_t UART_receive_task, pRadar_UART_DataHand_t Radar_UART_DataHand);/* start uart */
xRadar_UART_t* radar_UART_Find_by_Num(uart_port_t uart_num);/* find UART structures by uart_num */
esp_err_t radar_UART_ChangeFunbyNum(uart_port_t uart_num, pRadar_UART_DataHand_t UART_DataHand);/* Change the processing function */

void Radar_uart_default_receive_task(void *pvParameters);

#endif