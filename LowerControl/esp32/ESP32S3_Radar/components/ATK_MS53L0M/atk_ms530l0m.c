#include <string.h>

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "radar_uart.h"
#include "atk_ms53l0m.h"

static const char *TAG = "atk_ms53l0m";

static uart_port_t g_uart_num;
static const TickType_t WAITTIME = ATK_MS53L0M_WAITTIME;

static struct
{
    uart_port_t uart_num;               /* UART端口号 */
    uint8_t buf[RX_BUF_SIZE];           /* 帧接收缓冲 */
    size_t len;                         /* 帧接收长度 */
    SemaphoreHandle_t xBinarySemaphore; /* 帧接收完成标志，二进制信号量 */
} g_uart_rx_atk_ms53l0m_frame = {0};                /* ATK-MS53L0M UART接收帧缓冲信息结构体 */

/**
 * @brief       计算CRC校验和
 * @param       buf: 数据缓冲
 * @param       len: 数据长度
 * 
 * @retval      CRC校验和值
 */
static inline uint16_t atk_ms53l0m_crc_check_sum(uint8_t *buf, uint16_t len)
{
    uint16_t check_sum = 0;
    uint16_t i;
    
    for (i=0; i<len; i++)
    {
        check_sum += buf[i];
    }
    
    return check_sum;
}

/**
 * @brief       解析接收到的数据包
 * @param       dat     : 读操作时，读取到的数据要存入的地址
 * @param       recv_dat: 需要解析的数据包地址
 * @param       recv_len: 需要解析的数据包长度
 * 
 * @retval      ATK_MS53L0M_EOK     : 没有错误
 * @retval      ATK_MS53L0M_ETIMEOUT: 接收数据超时
 * @retval      ATK_MS53L0M_EFRAME  : 帧错误
 * @retval      ATK_MS53L0M_ECRC    : CRC校验错误
 * @retval      ATK_MS53L0M_EOPT    : 操作错误
 */
static uint8_t atk_ms53l0m_unpack_recv_data(uint16_t *const dat)
{
    uint8_t* recv_dat;
    size_t recv_len;
    uint16_t frame_loop = 0;
    int frame_head_index;
    uint16_t frame_len;
    uint8_t opt_type;
    uint16_t dat_len;
    uint16_t frame_check_sum;
    uint16_t check_sum;

    /* 等待数据信号量 */
    if (! xSemaphoreTake(g_uart_rx_atk_ms53l0m_frame.xBinarySemaphore, WAITTIME))
        return ATK_MS53L0M_ETIMEOUT;
    else
    {
        recv_dat = g_uart_rx_atk_ms53l0m_frame.buf;
        recv_len = g_uart_rx_atk_ms53l0m_frame.len;
    }
    
    /* 获取接收数据的长度 */
    if ((recv_len < ATK_MS53L0M_FRAME_LEN_MIN) || (recv_len > ATK_MS53L0M_FRAME_LEN_MAX))
    {
        /* 接收帧长度异常错误 */
        return ATK_MS53L0M_EFRAME;
    }

    /* 查找帧头 */
    do
    {
        if ((recv_dat[frame_loop] == ATK_MS53L0M_SLAVE_FRAME_HEAD) && 
            (recv_dat[frame_loop + 1] == ATK_MS53L0M_SENSOR_TYPE))
        {
            break;
        }
        
        if (frame_loop != (recv_len - 2))
        {
            frame_loop++;
        }
        else
        {
            /* 帧异常 */
            return ATK_MS53L0M_EFRAME;
        }
    } while (1);
    
    frame_head_index = frame_loop;              /* 记录帧头位置 */
    frame_len = recv_len - frame_head_index;    /* 计算帧长度 */
    
    if ((frame_len < ATK_MS53L0M_FRAME_LEN_MIN) || (frame_len > ATK_MS53L0M_FRAME_LEN_MAX))
    {
        /* 接收帧长度异常错误 */
        return ATK_MS53L0M_EFRAME;
    }
    
    opt_type = recv_dat[frame_head_index + 4]; /* 获取操作类型 */
    
    
    if (opt_type == ATK_MS53L0M_OPT_READ)
    {
        dat_len = (uint16_t)recv_dat[frame_head_index + 7];  /* 获取数据长度 */
        if ((dat_len + 10) > frame_len)
        {
            /* 帧异常 */
            return ATK_MS53L0M_EFRAME;
        }
        
        frame_check_sum = atk_ms53l0m_crc_check_sum(&recv_dat[frame_head_index], dat_len + 8);   /* 计算CRC校验和 */
        check_sum = ((uint16_t)recv_dat[frame_head_index + dat_len + 8] << 8) + (uint16_t)recv_dat[frame_head_index + dat_len + 9]; /* 获取帧的CRC校验和 */
        if (frame_check_sum == check_sum)
        {
            if (recv_dat[frame_head_index + 7] == 1)
            {
                *dat = recv_dat[frame_head_index + 8];
            }
            else if (recv_dat[frame_head_index + 7] == 2)
            {
                *dat = ((uint16_t)recv_dat[frame_head_index + 8] << 8) + recv_dat[frame_head_index + 9];
            }
            else
            {
                /* 帧错误 */
                return ATK_MS53L0M_EFRAME;
            }
            
            return ATK_MS53L0M_EOK;
        }
        else
        {
            /* CRC错误 */
            return ATK_MS53L0M_ECRC;
        }
    }
    else if (opt_type == ATK_MS53L0M_OPT_WRITE)
    {
        frame_check_sum = atk_ms53l0m_crc_check_sum(&recv_dat[frame_head_index], 6);    /* 计算CRC校验和 */
        check_sum = ((uint16_t)recv_dat[frame_head_index + 6] << 8) + recv_dat[frame_head_index + 7];
        if (frame_check_sum == check_sum)
        {
            return ATK_MS53L0M_EOK;
        }
        else
        {
            /* CRC错误 */
            return ATK_MS53L0M_ECRC;
        }
    }
    else if (opt_type == ATK_MS53L0M_OPT_ERROR)
    {

        if ((recv_dat[frame_head_index + 2] == ATK_MS53L0M_OPT_ERROR) && (recv_dat[frame_head_index + 3] == ATK_MS53L0M_OPT_ERROR))
        {
            frame_check_sum = atk_ms53l0m_crc_check_sum(&recv_dat[frame_head_index], 6);   /* 计算CRC校验和 */
            check_sum = ((uint16_t)recv_dat[frame_head_index + 6] << 8) + recv_dat[frame_head_index + 7];
            if (frame_check_sum == check_sum)
            {
                /* 异常操作 */
                ESP_LOGW(TAG,"error number:%x",recv_dat[frame_head_index + 5]);
                return ATK_MS53L0M_EOPT;
            }
            else
            {
                /* CRC错误 */
                return ATK_MS53L0M_ECRC;
            }
        }
        else
        {
            /* 帧异常 */
            return ATK_MS53L0M_EFRAME;
        }
    }
    else
    {
        /* 帧异常 */
        return ATK_MS53L0M_EFRAME;
    }
}

/**
 * @brief       将UART接收到的数据拷贝到全局接收帧缓冲中
 * @param       uart_num: ATK-MS53L0M连接的UART端口号
 * @param       dat     : 接收到的数据地址
 * @param       Len     ：数据长度
 * 
 * @retval      ESP_OK  : ATK-MS53L0M初始化成功，函数执行成功
 * @retval      ESP_FAIL: ATK-MS53L0M初始化失败，函数执行失败
 */
static void atk_ms53l0m_DataHand(const uart_port_t uart_num, uint8_t* dat, size_t Len)
{
    g_uart_rx_atk_ms53l0m_frame.len = Len;              /* 接收数据的长度 */
    g_uart_rx_atk_ms53l0m_frame.uart_num = uart_num;    /* 接收的UART端口号 */
    /* 获得UART接收缓冲区的数据 */
    for (int i = 0;i < Len; i++)
        g_uart_rx_atk_ms53l0m_frame.buf[i] = dat[i];
    
    /* 释放信号量 */
    xSemaphoreGive(g_uart_rx_atk_ms53l0m_frame.xBinarySemaphore);
}

/**
 * @brief       根据模块功能码读取数据
 * @param       addr: 设备地址
 * @param       fun_code : 功能码
 * @param       len : 数据长度，取值范围：1或2
 * @param       dat : 读取到的数据
 * 
 * @retval      ATK_MS53L0M_EOK     : 没有错误
 * @retval      ATK_MS53L0M_ETIMEOUT: 接收数据超时
 * @retval      ATK_MS53L0M_EFRAME  : 帧错误
 * @retval      ATK_MS53L0M_ECRC    : CRC校验错误
 * @retval      ATK_MS53L0M_EOPT    : 操作错误
 */
uint8_t atk_ms53l0m_read_data(uint16_t addr, uint8_t fun_code, uint8_t len, uint16_t *dat)
{
    uint16_t check_sum;
    uint8_t buf[9];
    
    buf[0] = ATK_MS53L0M_MASTER_FRAME_HEAD;              /* 标志头 */
    buf[1] = ATK_MS53L0M_SENSOR_TYPE;                    /* 传感器类型 */
    buf[2] = (uint8_t)(addr >> 8);                       /* 传感器地址，高8位 */
    buf[3] = (uint8_t)(addr & 0xFF);                     /* 传感器地址，低8位 */
    buf[4] = ATK_MS53L0M_OPT_READ;                       /* 读操作 */
    buf[5] = fun_code;                                   /* 功能码 */
    buf[6] = len;                                        /* 数据长度 */
    
    check_sum = atk_ms53l0m_crc_check_sum(buf, 7);       /* 计算CRC校验和 */
     
    buf[7] = (uint8_t)(check_sum >> 8);                  /* CRC校验码，高8位 */
    buf[8] = (uint8_t)(check_sum & 0xFF);                /* CRC校验码，低8位 */

    uart_write_bytes(g_uart_num, buf, 9);                /* 发送数据 */
    
    xSemaphoreTake(g_uart_rx_atk_ms53l0m_frame.xBinarySemaphore, 0); /* 清空信号量准备接收数据 */
    return atk_ms53l0m_unpack_recv_data(dat);            /* 解析应答数据 */
}

/**
 * @brief       根据模块功能码写入1字节数据
 * @param       addr     : 设备地址
 *              fun_code : 功能码
 *              dat      : 待写入的1字节数据
 * @retval      ATK_MS53L0M_EOK     : 没有错误
 *              ATK_MS53L0M_ETIMEOUT: 接收数据超时
 *              ATK_MS53L0M_EFRAME  : 帧错误
 *              ATK_MS53L0M_ECRC    : CRC校验错误
 *              ATK_MS53L0M_EOPT    : 操作错误
 */
uint8_t atk_ms53l0m_write_data(uint16_t addr, uint8_t fun_code, uint8_t dat)
{
    uint8_t buf[10];
    uint16_t check_sum;
    
    buf[0] = ATK_MS53L0M_MASTER_FRAME_HEAD;         /* 标志头 */
    buf[1] = ATK_MS53L0M_SENSOR_TYPE;               /* 传感器类型 */
    buf[2] = (uint8_t)(addr >> 8);                  /* 传感器地址，高8位 */
    buf[3] = (uint8_t)(addr & 0xFF);                /* 传感器地址，低8位 */
    buf[4] = ATK_MS53L0M_OPT_WRITE;                 /* 写操作 */
    buf[5] = fun_code;                              /* 功能码 */
    buf[6] = 0x01;                                  /* 数据长度 */
    buf[7] = dat;                                   /* 数据 */
    
    check_sum = atk_ms53l0m_crc_check_sum(buf, 8);  /* 计算CRC校验和 */
    
    buf[8] = (uint8_t)(check_sum >> 8);             /* CRC校验码，高8位 */
    buf[9] = (uint8_t)(check_sum & 0xFF);           /* CRC校验码，低8位 */
    
    uart_write_bytes(g_uart_num, buf, 10);               /* 发送数据 */

    xSemaphoreTake(g_uart_rx_atk_ms53l0m_frame.xBinarySemaphore, 0); /* 清空信号量准备接收数据 */
    return atk_ms53l0m_unpack_recv_data(NULL);     /* 解析应答数据 */
}

/**
 * @brief       ATK-MS53L0M初始化
 * @param       uart_num: ATK-MS53L0M要连接的UART端口号
 * @param       id      : ATK-MS53L0M应答返回的设备ID
 * 
 * @retval      ATK_MS53L0M_EOK     : 没有错误
 * @retval      ATK_MS53L0M_ETIMEOUT: 接收数据超时
 * @retval      ATK_MS53L0M_EFRAME  : 帧错误
 * @retval      ATK_MS53L0M_ECRC    : CRC校验错误
 * @retval      ATK_MS53L0M_EOPT    : 操作错误
 */
uint8_t atk_ms53l0m_init(uart_port_t uart_num, uint16_t *id)
{
    esp_err_t ret;
    /* 创建信号量 */
    g_uart_rx_atk_ms53l0m_frame.xBinarySemaphore = xSemaphoreCreateBinary();
    /* 更换对应UART端口任务的数据处理函数 */
    ret = radar_UART_ChangeFunbyNum(uart_num, atk_ms53l0m_DataHand);
    if (ret == ESP_FAIL)
        return ATK_MS53L0M_EOPT;/* 此处失败则为UART未初始化或使用了错误的端口号 */
    else
        g_uart_num = uart_num; /* 记录建立连接的UART端口号 */
    ESP_LOGI(TAG,"connent UART%d",g_uart_num);
    /* 获取设备地址 */
    ret = atk_ms53l0m_read_data(0xFFFF, ATK_MS53L0M_FUNCODE_IDSET, 2, id);
    if (ret != ATK_MS53L0M_EOK)
        return ret;
    /* 设置ATK-MS53L0M模块的工作模式为Modbus模式 */
    ret = atk_ms53l0m_write_data(*id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_MODBUS);
    if (ret != ATK_MS53L0M_EOK)
        return ret;
    
    ESP_LOGI(TAG, "init done!");
    return ATK_MS53L0M_EOK;
}

/**
 * @brief       ATK-MS53L0M Modbus工作模式获取测量值
 * @param       dat: 获取到的测量值
 * 
 * @retval      ATK_MS53L0M_EOK  : 获取测量值成功
 * @retval      ATK_MS53L0M_ETIMEOUT: 接收数据超时
 * @retval      ATK_MS53L0M_EFRAME  : 帧错误
 * @retval      ATK_MS53L0M_ECRC    : CRC校验错误
 * @retval      ATK_MS53L0M_EOPT    : 操作错误
 */
uint8_t atk_ms53l0m_modbus_get_data(uint16_t addr, uint16_t *dat)
{
    uint8_t ret;
    uint16_t dat_tmp;
    
    ret = atk_ms53l0m_read_data(addr, ATK_MS53L0M_FUNCODE_MEAUDATA, 2, &dat_tmp);
    if (ret != ATK_MS53L0M_EOK)
    {
        *dat = 0;
        return ret;
    }
    else
    {
        *dat = dat_tmp;
        return ret;
    }
}
