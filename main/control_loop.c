#include <lfw_esp/control_loop.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>

#include <lfw_esp/motors.h>
#include <lfw_esp/line_sensor.h>
#include <lfw_esp/turn_pid.h>
#include <lfw_esp/tcp_server.h>


#define CALIBRATION_SWEEP_TIME_US (1000 * 1000)

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


static const char TAG[] = "control_loop";

static QueueHandle_t g_forward_speed;
static QueueHandle_t g_enable;


static void control_loop_task(void *pv)
{
    uint32_t line_position = 0;

    for (;;)
    {
        line_sensor_measurement(&line_position);

        // 0 centered, 1.0 per sensor
        float error = line_position / 1024.f - ((LINE_SENSOR_N - 1) / 2.f);

        float output = turn_pid_update(error);

        float forward_speed = 0.f;
        int enable = 0;
        xQueuePeek(g_enable, &enable, 0);
        if (enable) xQueuePeek(g_forward_speed, &forward_speed, 0);

        float left_speed = forward_speed + output;
        float right_speed = forward_speed - output;

        int left = CLAMP((int) (left_speed * 4000), -8191, 8191);
        int right = CLAMP((int) (right_speed * 4000), -8191, 8191);

        motors_set_speed(left, right);

        tcp_server_send_angle(line_position * 1000);
        tcp_server_send_turn(output * 1000);
        tcp_server_send_motors(left, right);
    }
}

void control_loop_init(void)
{
    g_forward_speed = xQueueCreate(1, sizeof(float));
    g_enable = xQueueCreate(1, sizeof(int));
}

void control_loop_calibrate(void)
{
    ESP_LOGI(TAG, "Running calibration");

    motors_set_speed(-3000, 3000);

    while (esp_timer_get_time() < CALIBRATION_SWEEP_TIME_US)
    {
        line_sensor_calibrate();
    }

    motors_set_speed(3000, -3000);

    while (esp_timer_get_time() < 2 * CALIBRATION_SWEEP_TIME_US)
    {
        line_sensor_calibrate();
    }

    motors_set_speed(0, 0);

    ESP_LOGI(TAG, "Calibrated");
}

void control_loop_start(void)
{
    xTaskCreatePinnedToCore(control_loop_task, "control_loop", 4096, NULL, 4, NULL, APP_CPU_NUM);
}

void control_loop_set_forward(float speed)
{
    xQueueOverwrite(g_forward_speed, &speed);
}

void control_loop_set_enable(int enable)
{
    xQueueOverwrite(g_enable, &enable);
}
