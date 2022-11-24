#include <lfw_esp/tcp_server.h>

#include <string.h>
#include <sys/param.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>

#include <lfw_esp/cmd_parser.h>


#define PORT                        8888
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3


static const char TAG[] = "tcp_server";

typedef enum message_type_t {
    MESSAGE_MOTORS,
    MESSAGE_ANGLE,
} message_type_t;

typedef struct message_t {
    message_type_t type;
    TickType_t timestamp;
    union {
        struct {
            int left;
            int right;
        } motors;
        int angle;
    };
} message_t;

static volatile int g_sock = -1;
static SemaphoreHandle_t g_sock_sem;
static QueueHandle_t g_send_queue;


static void tcp_recv_loop()
{
    int len;
    char rx_buffer[256];

    do
    {
        while (xSemaphoreTake(g_sock_sem, portMAX_DELAY) != pdTRUE);

        len = recv(g_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        xSemaphoreGive(g_sock_sem);

        if (len < 0)
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        }
        else if (len == 0)
        {
            ESP_LOGW(TAG, "Connection closed");
        }
        else
        {
            for (int i = 0; i < len; i++)
            {
                cmd_parser_feed(rx_buffer[i]);
            }
        }
    } while (len > 0);
}

static void tcp_send(const char buf[])
{
    int to_write = strlen(buf);
    int written = 0;
    while (to_write > 0)
    {
        int ret = send(g_sock, buf + written, to_write, 0);
        if (ret < 0)
        {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }
        to_write -= ret;
        written += ret;
    }
}

static void tcp_send_task(void *pv)
{
    message_t msg;
    char buf[256];

    for (;;)
    {   
        if (xQueueReceive(g_send_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            if (xSemaphoreTake(g_sock_sem, portMAX_DELAY) != pdTRUE)
                continue;

            if (g_sock < 0)
            {
                xSemaphoreGive(g_sock_sem);
                continue;
            }

            switch (msg.type)
            {
                case MESSAGE_MOTORS:
                    sprintf(buf, "%u;motors_left;%d\n", msg.timestamp, msg.motors.left);
                    tcp_send(buf);
                    sprintf(buf, "%u;motors_right;%d\n", msg.timestamp, msg.motors.right);
                    tcp_send(buf);
                    break;

                case MESSAGE_ANGLE:
                    sprintf(buf, "%u;angle;%d\n", msg.timestamp, msg.angle);
                    tcp_send(buf);
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown message type: %d", msg.type);
                    break;
            }

            xSemaphoreGive(g_sock_sem);
        }
    }
}

static void tcp_server_task(void *pv)
{
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET)
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *) &dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    for (;;)
    {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr;
        socklen_t addr_len = sizeof(source_addr);

        while (xSemaphoreTake(g_sock_sem, portMAX_DELAY) != pdTRUE);

        g_sock = accept(listen_sock, (struct sockaddr *) &source_addr, &addr_len);
        if (g_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            xSemaphoreGive(g_sock_sem);
            continue;
        }

        setsockopt(g_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(g_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(g_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(g_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

        ESP_LOGI(TAG, "Socket accepted");

        xSemaphoreGive(g_sock_sem);

        tcp_recv_loop();

        while (xSemaphoreTake(g_sock_sem, portMAX_DELAY) != pdTRUE);

        shutdown(g_sock, 0);
        close(g_sock);
        g_sock = -1;

        xSemaphoreGive(g_sock_sem);
    }
}

void tcp_server_init(void)
{
    g_send_queue = xQueueCreate(32, sizeof(message_t));
    g_sock_sem = xSemaphoreCreateMutex();
}

void tcp_server_start(void)
{
    xTaskCreate(tcp_server_task, "tcp_recv", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_send_task, "tcp_send", 4096, NULL, 5, NULL);
}

void tcp_server_send_motors(int left, int right)
{
    static TickType_t last_send = 0;

    TickType_t ticks = xTaskGetTickCount();
    if (ticks - last_send < 10) return;
    last_send = ticks;

    message_t msg = {
        .type = MESSAGE_MOTORS,
        .timestamp = ticks,
        .motors = {
            .left = left,
            .right = right,
        },
    };

    xQueueSend(g_send_queue, &msg, 0);
}

void tcp_server_send_angle(int angle)
{
    static TickType_t last_send = 0;

    TickType_t ticks = xTaskGetTickCount();
    if (ticks - last_send < 10) return;
    last_send = ticks;

    message_t msg = {
        .type = MESSAGE_ANGLE,
        .timestamp = ticks,
        .angle = angle,
    };

    xQueueSend(g_send_queue, &msg, 0);
}
