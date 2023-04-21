#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "mod_bus.h"

static const char *TAG = "mod_bus";

static Modbus_uart_rx_data g_uart_rx_modbus_frame = {0};    /* Data in valid frames */

static uint8_t g_abnormal_message[8] = {    /* Abnormal message format */
    MODBUS_SLAVE_FRAME_HEAD,
    MODBUS_SENSOR_TYPE,
    MODBUS_OPT_ERROR,
    MODBUS_OPT_ERROR,
    MODBUS_OPT_ERROR,
    /* Fill in the remaining three bytes according to the actual situation */
};

/**
 * @brief       Copy String to Frame receive buffer
 * @param       destinin: Destination Address
 * @param       source  : Source Address
 * @param       Len     : Copy Len
 * 
 * @retval      void
*/
static inline void Modbus_copy_to_Framebuffer(uint8_t* destinin, const uint8_t *source, size_t Len)
{
    for (int i = Len - 1; i >= 0; i--) /* Bid Endian */
       destinin[i] = source[i];
}

/**
 * @brief       Calculate CRC checksum
 * @param       buf: data buff
 * @param       len: data len
 * 
 * @retval      checksum value
 */
static inline uint16_t Modbus_crc_check_sum(const uint8_t *buf, const uint16_t len)
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
 * @brief       unpack receive data and
 *              copy the section of the frame from the start of the read/write operation code 
                to the end of the data to the provided address
 * @param       pvalid_dat  : Address where the parsed data is stored
 * @param       recv_dat    : receive data
 * @param       recv_len    : receive data len
 * 
 * 
 * @retval      MODBUS_EOK    : No error
 * @retval      MODBUS_EFRAME : Frame format error
*/
static uint8_t Modbus_judgment_recv_data(const uint8_t* recv_dat, const size_t recv_len)
{
    uint16_t frame_loop = 0;
    uint16_t frame_head_index;
    size_t frame_len;
    uint8_t opt_type;
    uint16_t frame_check_sum;
    uint16_t check_sum;
    if ((recv_len < MODBUS_FRAME_LEN_MIN) || (recv_len > MODBUS_FRAME_LEN_MAX))
    {
        /* received data length error */
        return MODBUS_EFRAME;
    }

    /* Find Frame Headers */
    do
    {
        if ((recv_dat[frame_loop] == MODBUS_MASTER_FRAME_HEAD) && 
            (recv_dat[frame_loop + 1] == MODBUS_SENSOR_TYPE))
        {
            break;
        }
        
        if (frame_loop != (recv_len - 2))
        {
            frame_loop++;
        }
        else
        {
            /* Frame error */
            printf("MODBUS HERE!");
            return MODBUS_EFRAME;
        }
    } while (1);

    frame_head_index = frame_loop;              /* Actual frame header position */
    frame_len = recv_len - frame_head_index;    /* Actual frame len */
    if ((frame_len < MODBUS_FRAME_LEN_MIN) || (frame_len > MODBUS_FRAME_LEN_MAX))
    {
        /* received data length error */
        return MODBUS_EFRAME;
    }

    opt_type = recv_dat[frame_head_index + 4]; /* Get Operation Type */

    if (opt_type == MODBUS_OPT_READ)
    {
        frame_check_sum = Modbus_crc_check_sum(&recv_dat[frame_head_index], 7);   /* Calculate CRC checksum */
        check_sum = ((uint16_t)recv_dat[frame_head_index + 7] << 8) + (uint16_t)recv_dat[frame_head_index + 8]; /* Frame CRC checksum */
        if (frame_check_sum != check_sum)
            return MODBUS_EFRAME;/* Frame error */
        else
        {
            /* Copy the section of the frame from the start of the read/write operation code 
            to the end of the data to the provided address */
            g_uart_rx_modbus_frame.opt_type = opt_type;   /* Record Operation Type */
            g_uart_rx_modbus_frame.fun_code = recv_dat[frame_head_index + 5];   /* Record Modebus funcation code */
            g_uart_rx_modbus_frame.len      = recv_dat[frame_head_index + 6];   /* in read operation, Record number of bytes requested by the host */

            return MODBUS_EOK;
        }
    } else if (opt_type == MODBUS_OPT_WRITE) {
        uint16_t dat_len = (uint16_t)recv_dat[frame_head_index + 6];  /* Host write data length */
        if ((dat_len + 9) > frame_len)
        {
            /* Frame error */
            return MODBUS_EFRAME;
        }
        frame_check_sum = Modbus_crc_check_sum(&recv_dat[frame_head_index], dat_len + 7);   /* Calculate CRC checksum */
        check_sum = ((uint16_t)recv_dat[frame_head_index + dat_len + 7] << 8) + (uint16_t)recv_dat[frame_head_index + dat_len + 8]; /* Frame CRC checksum */
        if (frame_check_sum != check_sum)
            return MODBUS_EFRAME;/* Frame error */
        else
        {
            /* Copy the section of the frame from the start of the read/write operation code 
            to the end of the data to the provided address */
            g_uart_rx_modbus_frame.opt_type = opt_type;     /* Record Operation Type */
            g_uart_rx_modbus_frame.fun_code = recv_dat[frame_head_index + 5];   /* Record Modebus funcation code */
            g_uart_rx_modbus_frame.len      = recv_dat[frame_head_index + 6];   /* in write operation, Record the length of data sent by the host */
            Modbus_copy_to_Framebuffer(g_uart_rx_modbus_frame.buf, 
                                       &recv_dat[frame_head_index + 7], 
                                       recv_dat[frame_head_index + 6]); /* copy receive data */

            return MODBUS_EOK;
        }
    } else {
        /* Frame error */
        return MODBUS_EFRAME;
    }
}

/**
 * @brief       Processing data received by UART
 * @param       uart_num    UART port number
 * @param       dat         Received data address
 * @param       Len         Received data Len
 * 
 * @retval      void
*/
static void Modbus_uart_DataHand(const uart_port_t uart_num, uint8_t* dat, size_t Len)
{
    g_uart_rx_modbus_frame.uart_num = uart_num;    /* Received UART port number */
    ESP_LOGI(TAG, "[receive data]");
    /* unpack receive data */
    if (Modbus_judgment_recv_data(dat, Len))
    {
        ESP_LOGI(TAG, "[frame err!]");
        Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_FRAME);  /* Illegal data frame */
    } else {
        /* give semaphore */
        if ( (pdTRUE != xSemaphoreGive(g_uart_rx_modbus_frame.xBinarySemaphore)) && g_uart_rx_modbus_frame.status )
        {
            ESP_LOGI(TAG, "[busy!]");
            Modbus_transmit_ErrCode(MODBUS_STATUSCODE_ERR_BUSY);   /* busy */
        }
    }
}

/**
 * @brief       init Modbus
 * @param       uart_num : The UART port number of the host connecting to this device
 * 
 * @retval      MODBUS_ERROR : error
 * @retval      MODBUS_EOK : ok
*/
uint8_t Modbus_init(uart_port_t uart_num)
{
    if ( g_uart_rx_modbus_frame.device_address )
        return MODBUS_EOK;
    esp_err_t err;
    nvs_handle_t DeviceAddress_nvs_handle;
    /* Creating semaphores */
    g_uart_rx_modbus_frame.xBinarySemaphore = xSemaphoreCreateBinary();
    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    err = nvs_open("MODEBUS", NVS_READWRITE, &DeviceAddress_nvs_handle);
    if (err != ESP_OK) {
        g_uart_rx_modbus_frame.device_address = MODBUS_INITIAL_DEVICE_ADDRESS;
    } else {
        uint16_t DeviceAddress; 
        err = nvs_get_u16(DeviceAddress_nvs_handle, "DeviceAddress", &DeviceAddress); // get stored address
        if (err != ESP_OK) {
            /*  address will default to 0x0001, if not set yet in NVS */
            g_uart_rx_modbus_frame.device_address = MODBUS_INITIAL_DEVICE_ADDRESS; // no stored values
        } else {
            g_uart_rx_modbus_frame.device_address = DeviceAddress;  
        }
    }
    nvs_close(DeviceAddress_nvs_handle);
    /* init data processing status */
    g_uart_rx_modbus_frame.status = true;
    /* Register the processing function corresponding to the port number */
    err = radar_UART_ChangeFunbyNum(uart_num, Modbus_uart_DataHand);
    if (err == ESP_FAIL)
        return MODBUS_ERROR;    /* port number error */
    else
        /* Record the UART port number for establishing the connection */
        g_uart_rx_modbus_frame.uart_num = uart_num; 
    ESP_LOGI(TAG, "connent UART%d", uart_num);

    return MODBUS_EOK;
}

/**
 * @brief       get data address
 * @param       void
 * 
 * @retval      NULL : uninitialized
 * @retval      other : address
*/
Modbus_uart_rx_data* Modbus_Get_rx_Data_Address(void)
{
    if ( g_uart_rx_modbus_frame.device_address )
        return &g_uart_rx_modbus_frame;
    else
        return NULL;
}

/**
 * @brief       transmit in abnormal message
 * @param       err_code The working status code contained in the slave return message
*/
void Modbus_transmit_ErrCode(uint8_t err_code)
{
    uint16_t check_sum;
    g_abnormal_message[5] = err_code;
    check_sum = Modbus_crc_check_sum(g_abnormal_message, 6);
    g_abnormal_message[6] = (uint8_t)(check_sum >> 8);
    g_abnormal_message[7] = (uint8_t)(check_sum & 0xFF);
    uart_write_bytes(g_uart_rx_modbus_frame.uart_num, g_abnormal_message, 8);
}

/**
 * @brief       Return read message to the host(Up to two bytes of data)
 * 
 * @param       fun_code: function code    
 * 
 * @retval      void
*/
void Modbus_back_read_message(uint8_t fun_code, uint8_t len, uint16_t data)
{
    uint16_t check_sum;
    if ( len == 0x01 ) /* 1 byte data */
    {
        uint8_t buf[11];
        
        buf[0] = MODBUS_SLAVE_FRAME_HEAD;                   /* Slave response frame header */
        buf[1] = MODBUS_SENSOR_TYPE;                        /* Type code */
        buf[2] = (uint8_t)(g_uart_rx_modbus_frame.device_address >> 8);     /* Device address，High 8 bits */
        buf[3] = (uint8_t)(g_uart_rx_modbus_frame.device_address & 0xFF);   /* Device address，low 8 bits */
        buf[4] = MODBUS_OPT_READ;                            /* read operation */
        buf[5] = MODBUS_STATUSCODE_NORMAL;                   /* Work status code */
        buf[6] = fun_code;                                   /* function code */
        buf[7] = len;                                        /* data len */
    
        buf[8] = (uint8_t)(data & 0xFF);                    /* data, low 8 bits */
    
        check_sum = Modbus_crc_check_sum(buf, 7);      /* Calculate CRC checksum */
         
        buf[9] = (uint8_t)(check_sum >> 8);                /* CRC check code, high 8 bits */
        buf[10] = (uint8_t)(check_sum & 0xFF);              /* CRC check code, low 8 bits */
    
        uart_write_bytes(g_uart_rx_modbus_frame.uart_num, buf, 11); /* send data */
    } else if ( len == 0x02 ) /* 2 byte data */
    {
        uint8_t buf[12];

        buf[0] = MODBUS_SLAVE_FRAME_HEAD;                   /* Slave response frame header */
        buf[1] = MODBUS_SENSOR_TYPE;                        /* Type code */
        buf[2] = (uint8_t)(g_uart_rx_modbus_frame.device_address >> 8);     /* Device address，High 8 bits */
        buf[3] = (uint8_t)(g_uart_rx_modbus_frame.device_address & 0xFF);   /* Device address，low 8 bits */
        buf[4] = MODBUS_OPT_READ;                            /* read operation */
        buf[5] = MODBUS_STATUSCODE_NORMAL;                   /* Work status code */
        buf[6] = fun_code;                                   /* function code */
        buf[7] = len;                                        /* data len */

        buf[8] = (uint8_t)(data >> 8 );                     /* data, high 8 bits */
        buf[9] = (uint8_t)(data & 0xFF);                    /* data, low 8 bits */

        check_sum = Modbus_crc_check_sum(buf, 7);      /* Calculate CRC checksum */

        buf[10] = (uint8_t)(check_sum >> 8);                /* CRC check code, high 8 bits */
        buf[11] = (uint8_t)(check_sum & 0xFF);              /* CRC check code, low 8 bits */

        uart_write_bytes(g_uart_rx_modbus_frame.uart_num, buf, 12); /* send data */
    }
}

/**
 * @brief       Return write message to the host
 * 
 * @param       fun_code: function code    
 * 
 * @retval      void
*/
void Modbus_back_write_message(uint8_t fun_code)
{
    uint16_t check_sum;
    uint8_t buf[8];
    
    buf[0] = MODBUS_SLAVE_FRAME_HEAD;                                   /* Slave response frame header */
    buf[1] = MODBUS_SENSOR_TYPE;                                        /* Type code */
    buf[2] = (uint8_t)(g_uart_rx_modbus_frame.device_address >> 8);     /* Device address，High 8 bits */
    buf[3] = (uint8_t)(g_uart_rx_modbus_frame.device_address & 0xFF);   /* Device address，low 8 bits */
    buf[4] = MODBUS_OPT_WRITE;                                          /* Write operation */
    buf[5] = fun_code;                                                  /* function code */
    
    check_sum = Modbus_crc_check_sum(buf, 7);           /* Calculate CRC checksum */
     
    buf[6] = (uint8_t)(check_sum >> 8);                  /* CRC check code, high 8 bits */
    buf[7] = (uint8_t)(check_sum & 0xFF);                /* CRC check code, low 8 bits */

    uart_write_bytes(g_uart_rx_modbus_frame.uart_num, buf, 9); /* send data */
}