#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <lfw_esp/line_sensor.h>
#include <lfw_esp/motors.h>
#include <lfw_esp/control_loop.h>


static const char TAG[] = "main";


void app_main(void)
{
    ESP_LOGW(TAG, "LFW starting");

    line_sensor_init();
    motors_init();

    control_loop_start();

    ESP_LOGW(TAG, "LFW running");
}
