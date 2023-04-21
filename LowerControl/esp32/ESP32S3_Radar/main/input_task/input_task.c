#include "esp_log.h"

#include "radar_manager.h"
#include "atk_ms53l0m.h"

static Radar_status* g_pRadar_status;

void Radar_input_Execution_Task(void* pvParameters)
{
    while (1)
    {
        Radar_manager_Modbus_carry_out(portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

void Radar_input_measure_Task(void* pRadar_status)
{
    g_pRadar_status = (Radar_status*)pRadar_status;

    while (1)
    {
        /* wait steering Task */
        xEventGroupWaitBits(g_pRadar_status->Task_EventGroup, 0x1F, pdTRUE, pdTRUE, portMAX_DELAY); 
        /* get measure data */
        atk_ms53l0m_modbus_get_data(g_pRadar_status->Measurement_sensor_address, &g_pRadar_status->measure_data); 
        ESP_LOGI("measure Task", "distance: %d", g_pRadar_status->measure_data);
        /* Use EventGroup to inform measurement completion */
        xEventGroupSetBits(g_pRadar_status->Task_EventGroup, 0x20); /* event group 0~4 bits is steering, 5 bits is Distance Sensor */
    }
}