#ifndef _MOD_BUS_H_
#define _MOD_BUS_H_

#include "radar_uart.h"
/* fun_code */
enum
{
    MODBUS_FUNCODE_SYS              = 0x00, /* System settings */
    MODBUS_FUNCODE_SCANRATE         = 0x01, /* Automatic scanning rate setting */
    MODBUS_FUNCODE_BAUDRATE         = 0x02, /* BPS rate setting */
    MODBUS_FUNCODE_IDSET            = 0x03, /* Device Address Settings */
    MODBUS_FUNCODE_APPOINTDATA      = 0x05, /* Obtain specified azimuth data */
    MODBUS_FUNCODE_WORKMODE         = 0x06, /* Work mode */
    MODBUS_FUNCODE_MEASUREMODE      = 0x07, /* Measurement mode settings */
    MODBUS_FUNCODE_CALIMODE         = 0x08, /* Calibration Mode */
};

/* Work status code */
enum
{
    MODBUS_STATUSCODE_NORMAL        = 0x00, /* Normal operation */
    MODBUS_STATUSCODE_NO_SENSOR     = 0x01, /* Sensor not present */
    MODBUS_STATUSCODE_ERR_ADDRESS   = 0x02, /* The sensor address does not exist */
    MODBUS_STATUSCODE_ERR_OPR       = 0x03, /* Operation error */
    MODBUS_STATUSCODE_ERR_FUNCODE   = 0x04, /* Function code does not exist */
    MODBUS_STATUSCODE_ERR_LEN       = 0x05, /* Length error */
    MODBUS_STATUSCODE_ERR_CRC       = 0x06, /* Error in check code */
    MODBUS_STATUSCODE_ERR_FRAME     = 0x07, /* Incorrect frame format */
    MODBUS_STATUSCODE_ERR_BUSY      = 0x08, /* Device busy */
    MODBUS_STATUSCODE_ERR_DEVICE    = 0x09, /* Device abnormality */
    MODBUS_STATUSCODE_ERR_DATA      = 0x0A, /* Error writing data */
};

/* System settings parameters */
enum
{
    MODBUS_SYS_RUN                 = 0x00, /* System run */
    MODBUS_SYS_PARAM_RESET         = 0x01, /* System Recovery Settings */
    MODBUS_SYS_RESET               = 0x02, /* System reset */
    MODBUS_SYS_SUSPEND             = 0x03, /* System suspend */
};

/* Automatic scanning rate setting parameters */
enum
{
    MODBUS_BACKRATE_01HZ           = 0x00, /* 0.1Hz */
    MODBUS_BACKRATE_02HZ           = 0x01, /* 0.2Hz */
    MODBUS_BACKRATE_05HZ           = 0x02, /* 0.5Hz */
    MODBUS_BACKRATE_1HZ            = 0x03, /* 1Hz */
    MODBUS_BACKRATE_2HZ            = 0x04, /* 2Hz */
    MODBUS_BACKRATE_5HZ            = 0x05, /* 5Hz */
    MODBUS_BACKRATE_10HZ           = 0x06, /* 10Hz */
    MODBUS_BACKRATE_20HZ           = 0x07, /* 20Hz */
    MODBUS_BACKRATE_50HZ           = 0x08, /* 50Hz */
    MODBUS_BACKRATE_100HZ          = 0x09, /* 100Hz */
};

/* BPS settings parameters */
enum
{
    MODBUS_BAUDRATE_2400           = 0x00, /* 2400bps */
    MODBUS_BAUDRATE_4800           = 0x01, /* 4800bps */
    MODBUS_BAUDRATE_9600           = 0x02, /* 9600bps */
    MODBUS_BAUDRATE_19200          = 0x03, /* 19200bps */
    MODBUS_BAUDRATE_38400          = 0x04, /* 38400bps */
    MODBUS_BAUDRATE_57600          = 0x05, /* 57600bps */
    MODBUS_BAUDRATE_115200         = 0x06, /* 115200bps */
    MODBUS_BAUDRATE_230400         = 0x07, /* 230400bps */
    MODBUS_BAUDRATE_460800         = 0x08, /* 460800bps */
    MODBUS_BAUDRATE_921600         = 0x09, /* 921600bps */
};

/* Measurement mode settings parameters */
enum
{
    MODBUS_MEAUMODE_GENERAL        = 0x00, /* balance */
    MODBUS_MEAUMODE_HIPRECI        = 0x01, /* high-precision */
    MODBUS_MEAUMODE_LONG           = 0x02, /* long-distance */
    MODBUS_MEAUMODE_HISPEED        = 0x03, /* high speed */
};

/* Working mode settings parameters */
enum
{
    MODBUS_WORKMODE_NORMAL         = 0x00, /* Normal */

};


/* Definitions for error constants */
#define MODBUS_EOK         0   /* no error */
#define MODBUS_ERROR       1   /* error */
#define MODBUS_ETIMEOUT    2   /* time out */
#define MODBUS_EFRAME      3   /* frame error */
#define MODBUS_ECRC        4   /* CRC check error */
#define MODBUS_EOPT        5   /* operation error */

/* MODBUS settings */
#define MODBUS_MASTER_FRAME_HEAD   0x51    /* Host request frame header */
#define MODBUS_SLAVE_FRAME_HEAD    0x55    /* Slave response frame header */
#define MODBUS_SENSOR_TYPE         0x0B    /* Type code */
#define MODBUS_INITIAL_DEVICE_ADDRESS 0x0001 /* Device Address, Address Range：0x0001~0xFFFE */

#define MODBUS_FRAME_LEN_MAX       270     /* Maximum length of received frame */
#define MODBUS_FRAME_LEN_MIN       9       /* Minimum length of received frame */

#define MODBUS_OPT_READ            0x00    /* Read operation */
#define MODBUS_OPT_WRITE           0x01    /* Write operation */
#define MODBUS_OPT_ERROR           0xFF    /* Abnormal */

#define MODBUS_WAITTIME (100 / portTICK_PERIOD_MS) /* Maximum waiting time */

typedef struct {
    bool status;                        /* Structure state, true represents completion of data processing */
    uint16_t device_address;            /* Device address, Address Range：0x0001~0xFFFE */
    uart_port_t uart_num;               /* UART port number */
    SemaphoreHandle_t xBinarySemaphore; /* Binary semaphores are used to determine the arrival of data  */
    uint8_t opt_type;                   /* Operation type: 0x00 = read, 0x01 = write, 0xFF = error */
    uint8_t fun_code;                   /* Modebus funcation code */
    size_t len;                         /* Frame receive data Len */
    uint8_t buf[RX_BUF_SIZE];           /* Frame receive data buffer */
} Modbus_uart_rx_data;


uint8_t Modbus_init(uart_port_t uart_num); /* init Modbus */
Modbus_uart_rx_data* Modbus_Get_rx_Data_Address(void); /* get data address */
void Modbus_transmit_ErrCode(uint8_t work_code);   /* transmit in abnormal message */
void Modbus_back_read_message(uint8_t fun_code, uint8_t len, uint16_t data); /* Return read message to the host(include 2 byte data) */
void Modbus_back_write_message(uint8_t fun_code); /* Return write message to the host */

#endif