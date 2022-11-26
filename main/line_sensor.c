#include <lfw_esp/line_sensor.h>

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <driver/gpio.h>


#define SENSOR_SETTLE_DELAY_US (10)
#define SENSOR_TIMEOUT_US (2500)

#define ON_LINE_THRESHOLD (500)
#define NOISE_THRESHOLD (200)

#define CALIBRATION_SAMPLES (10)

#define GET_HIGH_RES_TIME() ((uint32_t) portGET_RUN_TIME_COUNTER_VALUE())


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

static uint32_t g_read_start_us = 0;
static uint32_t g_read_start = 0;
static uint32_t g_pulse_lengths[LINE_SENSOR_N] = {0};
static volatile int g_n_readings = LINE_SENSOR_N;

static uint32_t g_calibration_min[LINE_SENSOR_N];
static uint32_t g_calibration_max[LINE_SENSOR_N];
static bool g_calibrated = false;


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t now = GET_HIGH_RES_TIME();
    uint32_t sensor_id = (uint32_t) arg;
    g_pulse_lengths[sensor_id] = now - g_read_start;
    g_n_readings++;
}

static void update_line_position(uint32_t *position, const uint32_t values[])
{
    uint32_t avg = 0;
    uint32_t sum = 0;
    bool onLine = false;

    for (int i = 0; i < LINE_SENSOR_N * 1024; i += 1024)
    {
        uint32_t v = values[i];

        if (v > ON_LINE_THRESHOLD) onLine = true;

        if (v > NOISE_THRESHOLD)
        {
            avg += i * v;
            sum += v;
        }
    }

    if (!onLine)
    {
        // line position from 0 to 1024 * N (1024 per sensor)
        if (*position < (LINE_SENSOR_N - 1) << 9) *position = 0;
        else *position = (LINE_SENSOR_N - 1) << 10;
    }
    else
    {
        *position = avg / sum;
    }
}

static void sensor_read(uint32_t values[])
{
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // spin until all readings complete or timeout
    while (g_n_readings < LINE_SENSOR_N && (uint32_t) esp_timer_get_time() - g_read_start_us < SENSOR_TIMEOUT_US);

    uint32_t now = GET_HIGH_RES_TIME();

    // raise all pins and wait for remaining interrupts to settle
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        gpio_intr_disable(g_gpios[i]);
        gpio_set_direction(g_gpios[i], GPIO_MODE_OUTPUT);
        if (g_pulse_lengths[i] == -1) g_pulse_lengths[i] = now - g_read_start;
    }

    ets_delay_us(SENSOR_SETTLE_DELAY_US);

    // no more interrupts will trigger, safe to read
    memcpy(values, g_pulse_lengths, sizeof(g_pulse_lengths));

    // prepare interrupts
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        gpio_intr_enable(g_gpios[i]);
    }

    // begin next reading
    portENTER_CRITICAL(&mux);
    g_read_start_us = (uint32_t) esp_timer_get_time();
    g_read_start = GET_HIGH_RES_TIME();
    g_n_readings = 0;
    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        // set max value in case of timeout
        g_pulse_lengths[i] = -1;
        gpio_set_direction(g_gpios[i], GPIO_MODE_INPUT);
    }
    portEXIT_CRITICAL(&mux);
}

static void calibrate_reading(uint32_t values[])
{
    if (!g_calibrated)
    {
        ESP_LOGE(TAG, "calibrate_reading() called before calibration");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    else
    {
        for (int i = 0; i < LINE_SENSOR_N; i++)
        {
            if (values[i] <= g_calibration_min[i]) values[i] = 0;
            else if (values[i] >= g_calibration_max[i]) values[i] = 1024;
            else values[i] = ((values[i] - g_calibration_min[i]) << 10) / (g_calibration_max[i] - g_calibration_min[i]);
        }
    }
}

void line_sensor_init(void)
{
    if (!portGET_RUN_TIME_COUNTER_VALUE())
    {
        ESP_LOGE(TAG, "ccount not available");
        ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
    }

    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        g_calibration_min[i] = -1;
        g_calibration_max[i] = 0;
    }

    gpio_config_t io_conf = {};
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLDOWN_DISABLE;
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
        ESP_ERROR_CHECK(gpio_intr_disable(g_gpios[i]));
    }
}

void line_sensor_measurement(uint32_t *line_position)
{
    uint32_t values[LINE_SENSOR_N];

    sensor_read(values);
    
    // ESP_LOGI(TAG, "Raw: %u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
    //     values[0],
    //     values[1],
    //     values[2],
    //     values[3],
    //     values[4],
    //     values[5],
    //     values[6],
    //     values[7]
    // );

    calibrate_reading(values);
    
    // ESP_LOGI(TAG, "Calib: %u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
    //     values[0],
    //     values[1],
    //     values[2],
    //     values[3],
    //     values[4],
    //     values[5],
    //     values[6],
    //     values[7]
    // );
    
    update_line_position(line_position, values);
}

void line_sensor_calibrate(void)
{
    uint32_t min[LINE_SENSOR_N];
    uint32_t max[LINE_SENSOR_N];
    uint32_t values[LINE_SENSOR_N];

    for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++)
    {
        sensor_read(values);

        for (int i = 0; i < LINE_SENSOR_N; i++)
        {
            if (sample == 0)
            {
                min[i] = values[i];
                max[i] = values[i];
            }
            else
            {
                if (values[i] < min[i]) min[i] = values[i];
                if (values[i] > max[i]) max[i] = values[i];
            }
        }
    }

    for (int i = 0; i < LINE_SENSOR_N; i++)
    {
        if (min[i] > g_calibration_max[i]) g_calibration_max[i] = min[i];
        if (max[i] < g_calibration_min[i]) g_calibration_min[i] = max[i];
    }

    g_calibrated = true;

    // ESP_LOGI(TAG, "Calib min: %u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
    //     g_calibration_min[0],
    //     g_calibration_min[1],
    //     g_calibration_min[2],
    //     g_calibration_min[3],
    //     g_calibration_min[4],
    //     g_calibration_min[5],
    //     g_calibration_min[6],
    //     g_calibration_min[7]
    // );

    // ESP_LOGI(TAG, "Calib max: %u\t%u\t%u\t%u\t%u\t%u\t%u\t%u",
    //     g_calibration_max[0],
    //     g_calibration_max[1],
    //     g_calibration_max[2],
    //     g_calibration_max[3],
    //     g_calibration_max[4],
    //     g_calibration_max[5],
    //     g_calibration_max[6],
    //     g_calibration_max[7]
    // );
}
