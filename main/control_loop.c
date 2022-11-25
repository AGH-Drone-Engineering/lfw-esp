#include <lfw_esp/control_loop.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>

#include <lfw_esp/motors.h>
#include <lfw_esp/line_sensor.h>
#include <lfw_esp/turn_pid.h>
#include <lfw_esp/tcp_server.h>


#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


static const char TAG[] = "control_loop";

static QueueHandle_t g_forward_speed;
static QueueHandle_t g_enable;


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

        float forward_speed = 0.f;
        int enable = 0;
        xQueuePeek(g_enable, &enable, 0);
        if (enable) xQueuePeek(g_forward_speed, &forward_speed, 0);

        float left_speed = forward_speed + output;
        float right_speed = forward_speed - output;

        int left = (int) CLAMP(left_speed * 2000, -2000, 2000);
        int right = (int) CLAMP(right_speed * 2000, -2000, 2000);

        motors_set_speed(left, right);

        tcp_server_send_angle(line_position * 1000);
        tcp_server_send_turn(output * 1000);
        tcp_server_send_motors(left, right);

        // TODO remove
        vTaskDelay(1);
    }
}

void control_loop_init(void)
{
    g_forward_speed = xQueueCreate(1, sizeof(float));
    g_enable = xQueueCreate(1, sizeof(int));
}

void control_loop_start(void)
{
    xTaskCreatePinnedToCore(control_loop_task, "control_loop", 4096, NULL, 4, NULL, PRO_CPU_NUM);
    ESP_LOGI(TAG, "started");
}

void control_loop_set_forward(float speed)
{
    xQueueOverwrite(g_forward_speed, &speed);
}

void control_loop_set_enable(int enable)
{
    xQueueOverwrite(g_enable, &enable);
}
