#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>

#include <lfw_esp/line_sensor.h>
#include <lfw_esp/motors.h>
#include <lfw_esp/control_loop.h>
#include <lfw_esp/wifi_man.h>
#include <lfw_esp/tcp_server.h>
#include <lfw_esp/turbine.h>


static const char TAG[] = "main";

static SemaphoreHandle_t g_core1_init_done;


static void core1_main(void *pv)
{
    line_sensor_init();
    motors_init();
    turbine_init();
    control_loop_init();

    xSemaphoreGive(g_core1_init_done);

    ESP_LOGW(TAG, "Will calibrate");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    control_loop_calibrate();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    control_loop_start();

    ESP_LOGW(TAG, "LFW running");

    vTaskDelete(0);
}

void app_main(void)
{
    ESP_LOGW(TAG, "LFW starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    tcp_server_init();

    g_core1_init_done = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(core1_main, "core1_main", 4096, 0, 7, 0, APP_CPU_NUM);

    while (xSemaphoreTake(g_core1_init_done, portMAX_DELAY) != pdTRUE);
    
    turbine_start();
    wifi_man_start();
    tcp_server_start();
}
