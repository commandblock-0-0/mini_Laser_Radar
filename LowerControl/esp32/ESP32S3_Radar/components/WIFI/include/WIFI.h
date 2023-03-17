#ifndef _WIFI_H_
#define _WIFI_H_

#include "esp_err.h"

esp_err_t wifi_init_sta(void);

void udp_client_task(void *pvParameters);

#endif