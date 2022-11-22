#include <lfw_esp/motors.h>

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/ledc.h>


#define MOTOR_1_EN GPIO_NUM_17
#define MOTOR_1_A GPIO_NUM_18
#define MOTOR_1_B GPIO_NUM_19

#define MOTOR_2_EN GPIO_NUM_21
#define MOTOR_2_A GPIO_NUM_22
#define MOTOR_2_B GPIO_NUM_23


void motors_init(void)
{
    // configure enable pins

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_APB_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_1_EN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    channel_conf.gpio_num = MOTOR_2_EN;
    channel_conf.channel = LEDC_CHANNEL_1;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    // configure direction pins

    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << MOTOR_1_A) |
            (1ULL << MOTOR_1_B) |
            (1ULL << MOTOR_2_A) |
            (1ULL << MOTOR_2_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_1_A, 0));
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_2_A, 0));
}

void motors_set_speed(int left, int right)
{
    if (left == 0)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_1_B, 0);
    }
    else if (left > 0)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(MOTOR_1_A, 1);
        gpio_set_level(MOTOR_1_B, 0);
    }
    else
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, -left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(MOTOR_1_A, 0);
        gpio_set_level(MOTOR_1_B, 1);
    }

    if (right == 0)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_2_B, 0);
    }
    else if (right > 0)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        gpio_set_level(MOTOR_2_A, 1);
        gpio_set_level(MOTOR_2_B, 0);
    }
    else
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, -right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        gpio_set_level(MOTOR_2_A, 0);
        gpio_set_level(MOTOR_2_B, 1);
    }
}
