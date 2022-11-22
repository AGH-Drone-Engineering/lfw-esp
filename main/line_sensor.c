#include <lfw_esp/line_sensor.h>

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/gpio.h>


#define SENSOR_SETTLE_DELAY_US (10)
#define SENSOR_TIMEOUT_US (1000)


static const char TAG[] = "line_sensor";

static const gpio_num_t g_gpios[LINE_SENSOR_N] = {
    GPIO_NUM_13,
    GPIO_NUM_14,
    GPIO_NUM_27,
    GPIO_NUM_26,
    GPIO_NUM_25,
    GPIO_NUM_33,
    GPIO_NUM_32,
    GPIO_NUM_16,
};

static struct {
    portMUX_TYPE gpio_mux;
    uint32_t read_start;
    uint32_t read_starts[LINE_SENSOR_N];
    volatile uint32_t pulse_lengths[LINE_SENSOR_N];
    volatile int n_readings;
} g_state = {
    .gpio_mux = portMUX_INITIALIZER_UNLOCKED,
    .read_start = 0,
    .read_starts = {0},
    .pulse_lengths = {0},
    .n_readings = 0,
};


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t now = portGET_RUN_TIME_COUNTER_VALUE();
    uint32_t sensor_id = (uint32_t) arg;
    g_state.pulse_lengths[sensor_id] = now - g_state.read_starts[sensor_id];
    g_state.n_readings++;
}

void line_sensor_init(void)
{
    if (!portGET_RUN_TIME_COUNTER_VALUE())
    {
        ESP_LOGE(TAG, "ccount not available");
        ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        io_conf.pin_bit_mask |= 1ULL << g_gpios[i];
        ESP_ERROR_CHECK(gpio_set_level(g_gpios[i], 1));
    }
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        ESP_ERROR_CHECK(gpio_isr_handler_add(
            g_gpios[i],
            gpio_isr_handler,
            (void*) i
        ));
    }
}

void line_sensor_measurement(line_sensor_measurement_t *measurement)
{
    while (g_state.n_readings < LINE_SENSOR_N && (uint32_t) esp_timer_get_time() - g_state.read_start < SENSOR_TIMEOUT_US);

    portENTER_CRITICAL(&g_state.gpio_mux);
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        gpio_set_direction(g_gpios[i], GPIO_MODE_OUTPUT);
    }
    portEXIT_CRITICAL(&g_state.gpio_mux);

    ets_delay_us(SENSOR_SETTLE_DELAY_US);

    memcpy((void*) measurement->values, (void*) g_state.pulse_lengths, sizeof(g_state.pulse_lengths));

    portENTER_CRITICAL(&g_state.gpio_mux);
    g_state.read_start = (uint32_t) esp_timer_get_time();
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        g_state.read_starts[i] = portGET_RUN_TIME_COUNTER_VALUE();
        gpio_set_direction(g_gpios[i], GPIO_MODE_INPUT);
    }
    portEXIT_CRITICAL(&g_state.gpio_mux);
}
