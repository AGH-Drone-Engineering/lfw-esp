#include <lfw_esp/control_loop.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include <lfw_esp/motors.h>
#include <lfw_esp/line_sensor.h>
#include <lfw_esp/turn_pid.h>
#include <lfw_esp/tcp_server.h>


#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define FORWARD_SPEED 0.5f


static const char TAG[] = "control_loop";


static void control_loop_task(void *pv)
{
    float line_position = 0.0f;    

    int last_time = 0;

    for (;;)
    {
        line_sensor_measurement(&line_position);

        int time = esp_timer_get_time();
        float output = turn_pid_update(line_position, (time - last_time) / 1000.0f);
        last_time = time;

        float left_speed = FORWARD_SPEED + output;
        float right_speed = FORWARD_SPEED - output;

        int left = (int) CLAMP(left_speed * 2000, -2000, 2000);
        int right = (int) CLAMP(right_speed * 2000, -2000, 2000);

        motors_set_speed(left, right);

        tcp_server_send_angle(line_position * 1000);
        tcp_server_send_motors(left, right);
    }
}

void control_loop_start(void)
{
    xTaskCreate(control_loop_task, "control_loop", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "started");
}
