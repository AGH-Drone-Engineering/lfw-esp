#include <lfw_esp/control_loop.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>

#include <lfw_esp/motors.h>
#include <lfw_esp/line_sensor.h>
#include <lfw_esp/turn_pid.h>
#include <lfw_esp/tcp_server.h>


#define CALIBRATION_SWEEP_TIME_US (1500 * 1000)
#define CALIBRATION_SWEEP_SPEED (600)

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


static const char TAG[] = "control_loop";

static QueueHandle_t g_forward_speed;
static QueueHandle_t g_enable;


static void control_loop_task(void *pv)
{
    uint32_t line_position = 0;

    float error = 0.f;
    float new_error;
    bool off_line;

    int enable;
    float forward_speed;

    float output;
    float left_speed;
    float right_speed;

    int left;
    int right;

    for (;;)
    {
        line_sensor_measurement(&line_position);

        // 0 centered, 1.0 max left/right
        new_error = line_position / ((LINE_SENSOR_N - 1) * 512.f) - 1.f;
        error = 0.6f * error + 0.4f * new_error;
        off_line = error < -0.9f || error > 0.9f;

        // ESP_LOGI(TAG, "Line: %f", error);

        output = turn_pid_update(error);

        if (off_line)
        {
            if (error < 0)
            {
                left = -8191;
                right = 0;
            }
            else
            {
                left = 0;
                right = -8191;
            }
        }
        else
        {
            forward_speed = 0.f;
            enable = 0;
            xQueuePeek(g_enable, &enable, 0);
            if (enable) xQueuePeek(g_forward_speed, &forward_speed, 0);

            left_speed = forward_speed + output;
            right_speed = forward_speed - output;

            left = CLAMP((int) (left_speed * 4000), -8191, 8191);
            right = CLAMP((int) (right_speed * 4000), -8191, 8191);
        }

        motors_set_speed(left, right);

        tcp_server_send_angle(error * 1000);
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
    line_sensor_calibrate();
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
