#include <lfw_esp/line_sensor.h>

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/gpio.h>


#define LINE_SENSOR_N (8)
#define SENSOR_SETTLE_DELAY_US (10)
#define SENSOR_TIMEOUT_US (1000)
#define ON_LINE_THRESHOLD (1000000)
#define NOISE_THRESHOLD (ON_LINE_THRESHOLD / 4)


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

static uint32_t g_read_start = 0;
static uint32_t g_read_starts[LINE_SENSOR_N] = {0};
static volatile uint32_t g_pulse_lengths[LINE_SENSOR_N] = {0};
static volatile int g_n_readings = 0;
static SemaphoreHandle_t g_sem;


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t now = portGET_RUN_TIME_COUNTER_VALUE();
    uint32_t sensor_id = (uint32_t) arg;
    g_pulse_lengths[sensor_id] = now - g_read_starts[sensor_id];
    g_n_readings++;
    if (g_n_readings == LINE_SENSOR_N)
    {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(g_sem, &woken);
        if (woken) portYIELD_FROM_ISR();
    }
}

static void update_line_position(float *position)
{
    float avg = 0.0f;
    float sum = 0.0f;
    bool onLine = false;

    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        float v = g_pulse_lengths[i] / (float) ON_LINE_THRESHOLD;

        if (v > 1.0f)
        {
            onLine = true;
        }

        if (v > 0.25f)
        {
            avg += i * v;
            sum += v;
        }
    }

    if (!onLine)
    {
        *position = *position < 0 ? -1.0f : 1.0f;
    }
    else
    {
        *position = avg / sum - (LINE_SENSOR_N - 1) / 2.0f;
    }
}

void line_sensor_init(void)
{
    if (!portGET_RUN_TIME_COUNTER_VALUE())
    {
        ESP_LOGE(TAG, "ccount not available");
        ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
    }

    g_sem = xSemaphoreCreateBinary();

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

void line_sensor_measurement(float *line_position)
{
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    xSemaphoreTake(g_sem, 1);
    // while (g_state.n_readings < LINE_SENSOR_N && (uint32_t) esp_timer_get_time() - g_state.read_start < SENSOR_TIMEOUT_US)
        // portYIELD();

    portENTER_CRITICAL(&mux);
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        gpio_set_direction(g_gpios[i], GPIO_MODE_OUTPUT);
    }
    portEXIT_CRITICAL(&mux);

    ets_delay_us(SENSOR_SETTLE_DELAY_US);

    update_line_position(line_position);

    portENTER_CRITICAL(&mux);
    g_read_start = (uint32_t) esp_timer_get_time();
    g_n_readings = 0;
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        g_read_starts[i] = portGET_RUN_TIME_COUNTER_VALUE();
        gpio_set_direction(g_gpios[i], GPIO_MODE_INPUT);
    }
    portEXIT_CRITICAL(&mux);
}
