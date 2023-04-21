#ifndef _ATK_MS53L0M_H_
#define _ATK_MS53L0M_H_

#include "freertos/FreeRTOS.h"

/* ATK-MS53L0M模块功能码 */
enum
{
    ATK_MS53L0M_FUNCODE_SYS             = 0x00, /* 系统设置 */
    ATK_MS53L0M_FUNCODE_BACKRATE        = 0x01, /* 回传速率设置 */
    ATK_MS53L0M_FUNCODE_BAUDRATE        = 0x02, /* 波特率设置 */
    ATK_MS53L0M_FUNCODE_IDSET           = 0x03, /* 设备地址设置 */
    ATK_MS53L0M_FUNCODE_MEAUDATA        = 0x05, /* 测量数据获取 */
    ATK_MS53L0M_FUNCODE_OUTPUTSTATUS    = 0x07, /* 测量状态 */
    ATK_MS53L0M_FUNCODE_MEAUMODE        = 0x08, /* 测量模式设置 */
    ATK_MS53L0M_FUNCODE_CALIMODE        = 0x09, /* 校准模式 */
    ATK_MS53L0M_FUNCODE_WORKMODE        = 0x0A, /* 工作模式 */
    ATK_MS53L0M_FUNCODE_ERRORFRAM       = 0x0B, /* 错误帧信息 */
    ATK_MS53L0M_FUNCODE_VERSION         = 0x0C, /* 版本信息 */
};

/* 系统设置参数 */
enum
{
    ATK_MS53L0M_SYS_PARAM_RESET         = 0x01, /* 系统恢复设置 */
    ATK_MS53L0M_SYS_RESET               = 0x02, /* 系统复位 */
};

/* 回传速率设置参数 */
enum
{
    ATK_MS53L0M_BACKRATE_01HZ           = 0x00, /* 0.1Hz */
    ATK_MS53L0M_BACKRATE_02HZ           = 0x01, /* 0.2Hz */
    ATK_MS53L0M_BACKRATE_05HZ           = 0x02, /* 0.5Hz */
    ATK_MS53L0M_BACKRATE_1HZ            = 0x03, /* 1Hz */
    ATK_MS53L0M_BACKRATE_2HZ            = 0x04, /* 2Hz */
    ATK_MS53L0M_BACKRATE_5HZ            = 0x05, /* 5Hz */
    ATK_MS53L0M_BACKRATE_10HZ           = 0x06, /* 10Hz */
    ATK_MS53L0M_BACKRATE_20HZ           = 0x07, /* 20Hz */
    ATK_MS53L0M_BACKRATE_50HZ           = 0x08, /* 50Hz */
    ATK_MS53L0M_BACKRATE_100HZ          = 0x09, /* 100Hz */
};

/* 串口波特率设置参数 */
enum
{
    ATK_MS53L0M_BAUDRATE_2400           = 0x00, /* 2400bps */
    ATK_MS53L0M_BAUDRATE_4800           = 0x01, /* 4800bps */
    ATK_MS53L0M_BAUDRATE_9600           = 0x02, /* 9600bps */
    ATK_MS53L0M_BAUDRATE_19200          = 0x03, /* 19200bps */
    ATK_MS53L0M_BAUDRATE_38400          = 0x04, /* 38400bps */
    ATK_MS53L0M_BAUDRATE_57600          = 0x05, /* 57600bps */
    ATK_MS53L0M_BAUDRATE_115200         = 0x06, /* 115200bps */
    ATK_MS53L0M_BAUDRATE_230400         = 0x07, /* 230400bps */
    ATK_MS53L0M_BAUDRATE_460800         = 0x08, /* 460800bps */
    ATK_MS53L0M_BAUDRATE_921600         = 0x09, /* 921600bps */
};

/* 测量模式设置参数 */
enum
{
    ATK_MS53L0M_MEAUMODE_GENERAL        = 0x00, /* 一般 */
    ATK_MS53L0M_MEAUMODE_HIPRECI        = 0x01, /* 高精度 */
    ATK_MS53L0M_MEAUMODE_LONG           = 0x02, /* 长距离 */
    ATK_MS53L0M_MEAUMODE_HISPEED        = 0x03, /* 高速 */
};

/* 工作模式设置参数 */
enum
{
    ATK_MS53L0M_WORKMODE_NORMAL         = 0x00, /* Normal模式 */
    ATK_MS53L0M_WORKMODE_MODBUS         = 0x01, /* Modbus模式 */
    ATK_MS53L0M_WORKMODE_IIC            = 0x02, /* IIC模式 */
};

/* 错误帧输出设置参数 */
enum
{
    ATK_MS53L0M_ERRORFRAM_OFF           = 0x00, /* 关闭 */
    ATK_MS53L0M_ERRORFRAM_ON            = 0x01, /* 开启 */
};

/* 错误代码 */
#define ATK_MS53L0M_EOK         0   /* 没有错误 */
#define ATK_MS53L0M_ERROR       1   /* 错误 */
#define ATK_MS53L0M_ETIMEOUT    2   /* 超时错误 */
#define ATK_MS53L0M_EFRAME      3   /* 帧错误 */
#define ATK_MS53L0M_ECRC        4   /* CRC校验错误 */
#define ATK_MS53L0M_EOPT        5   /* 操作错误 */

/* MODBUS设置 */
#define ATK_MS53L0M_MASTER_FRAME_HEAD   0x51    /* 主机请求帧头 */
#define ATK_MS53L0M_SLAVE_FRAME_HEAD    0x55    /* 从机应答帧头 */
#define ATK_MS53L0M_SENSOR_TYPE         0x0B    /* ATK-MS53L0M传感器类型 */

#define ATK_MS53L0M_FRAME_LEN_MAX       270     /* 接收帧最大长度 */
#define ATK_MS53L0M_FRAME_LEN_MIN       8       /* 接收帧最小长度 */

#define ATK_MS53L0M_OPT_READ            0x00    /* 读操作 */
#define ATK_MS53L0M_OPT_WRITE           0x01    /* 写操作 */
#define ATK_MS53L0M_OPT_ERROR           0xFF    /* 异常报文 */

#define ATK_MS53L0M_WAITTIME (1000 / portTICK_PERIOD_MS) /* 超时前的等待时间 */

uint8_t atk_ms53l0m_init(uart_port_t uart_num, uint16_t *id);/* ATK-MS53L0M初始化 */
uint8_t atk_ms53l0m_read_data(uint16_t addr, uint8_t fun_code, uint8_t len, uint16_t *dat);/* 根据模块功能码读取数据 */
uint8_t atk_ms53l0m_write_data(uint16_t addr, uint8_t fun_code, uint8_t dat);/* 根据模块功能码写入1字节数据 */
uint8_t atk_ms53l0m_modbus_get_data(uint16_t addr, uint16_t *dat);/* ATK-MS53L0M Modbus工作模式获取测量值 */
/* ATK-MS53L0M Normal工作模式获取测量值 */

#endif