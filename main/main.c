#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <lfw_esp/line_sensor.h>


static const char TAG[] = "main";


void app_main(void)
{
    ESP_LOGW(TAG, "LFW starting");

    if (line_sensor_init())
    {
        ESP_LOGE(TAG, "Line sensor init failed");
        return;
    }

    ESP_LOGW(TAG, "LFW running");
}
