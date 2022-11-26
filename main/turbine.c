#include <lfw_esp/turbine.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <driver/ledc.h>


#define TURBINE_GPIO GPIO_NUM_4
#define DUTY_MIN (410)
#define DUTY_MAX (819)


static QueueHandle_t g_requested_speed;


static void set_speed(int speed)
{
    // speed = 0 - 255
    int duty = speed * (DUTY_MAX - DUTY_MIN) / 255 + DUTY_MIN;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}

static void turbine_task(void *pv)
{
    int speed;

    // idle
    set_speed(0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // arm
    set_speed(100);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // idle
    set_speed(0);

    for (;;)
    {
        if (xQueueReceive(g_requested_speed, &speed, portMAX_DELAY) == pdTRUE)
        {
            set_speed(speed);
        }
    }
}

void turbine_init(void)
{
    g_requested_speed = xQueueCreate(1, sizeof(int));

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_APB_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = TURBINE_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

void turbine_start(void)
{
    xTaskCreatePinnedToCore(turbine_task, "turbine", 2048, 0, 8, 0, PRO_CPU_NUM);
}

void turbine_set_speed(int speed)
{
    xQueueOverwrite(g_requested_speed, &speed);
}
