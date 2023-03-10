#ifndef _RADAR_UART_H_
#define _RADAR_UART_H_

#include "freertos/queue.h"
#include "driver/uart.h"

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

typedef enum {
    RADAR_STOP = 1,
    RADAR_RUN,
    RADAR_MOD,
    RADAR_ANGLE_CALIBRATION,// The angle of each steering gear may be different. This command opens the calibration mode
} Radar_uart_command_t;

typedef struct {
    uart_port_t uart_num;        //uart num
    uart_config_t *pUart_config; //uart config, most configurations come from Kconfig
    QueueHandle_t uart_queue;    //Interrupt event queue

    void(* UART_Init)(void); //init function
    void(* UART_Exit)(void); //exit function

    TaskHandle_t handle_receive_task;//uart event task handle
} xRadar_UART_t;//defined UART itself 

typedef void(* pRadar_UART_DataHand_t)(char* dtmp, size_t size);


xRadar_UART_t* radar_UART_Run(TaskFunction_t UART_receive_task, 
                              pRadar_UART_DataHand_t Radar_UART_DataHand);

#endif