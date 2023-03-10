#ifndef _RADAR_UART_H_
#define _RADAR_UART_H_

#include "freertos/queue.h"
#include "driver/uart.h"

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

typedef void(* pRadar_UART_DataHand_t)(uint8_t* dtmp, size_t size);

typedef struct {
    uart_port_t uart_num;        //uart num
    uart_config_t *pUart_config; //uart config, most configurations come from Kconfig
    QueueHandle_t uart_queue;    //Interrupt event queue

    void(* UART_Init)(void); //init function
    void(* UART_Exit)(void); //exit function

    TaskHandle_t handle_receive_task;//uart event task handle
} xRadar_UART_t;//defined UART itself 

xRadar_UART_t* radar_UART_Run(TaskFunction_t UART_receive_task, 
                              pRadar_UART_DataHand_t Radar_UART_DataHand);

#endif