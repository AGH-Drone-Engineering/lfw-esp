#include <lfw_esp/control_loop.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>

#include <lfw_esp/motors.h>
#include <lfw_esp/line_sensor.h>
#include <lfw_esp/pid.h>


#define FORWARD_SPEED 0.5f


static void control_loop_task(void *pv)
{
    float line_position = 0.0f;
    pid_state_t pid = {
        .kp = 0.5f,
        .kd = 0.2f,
    };

    for (;;)
    {
        line_sensor_measurement(&line_position);
        float output = pid_update(&pid, line_position, 1.0f);
        float left_speed = FORWARD_SPEED + output;
        float right_speed = FORWARD_SPEED - output;
        motors_set_speed(left_speed, right_speed);
    }
}

void control_loop_start(void)
{
    xTaskCreate(control_loop_task, "control_loop", 4096, NULL, 5, NULL);
}
