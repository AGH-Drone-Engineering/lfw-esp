#include <lfw_esp/cmd_parser.h>

#include <esp_log.h>
#include <string.h>

#include <lfw_esp/turn_pid.h>
#include <lfw_esp/control_loop.h>
#include <lfw_esp/turbine.h>


static const char TAG[] = "cmd_parser";

typedef enum state_t {
    STATE_TYPE,
    STATE_DATA,
} state_t;

static state_t g_state = STATE_TYPE;
static char g_type[256] = {0};
static char g_data[256] = {0};
static int g_index = 0;


static void cmd_parser_accept()
{
    char *endptr;
    int data = strtol(g_data, &endptr, 10);
    if (*endptr) return;

    if (!strcmp(g_type, "Set P"))
    {
        turn_pid_set_p(data / 1000.f);
        ESP_LOGI(TAG, "Set P to %d", data);
    }
    else if (!strcmp(g_type, "Set D"))
    {
        turn_pid_set_d(data / 1000.f);
        ESP_LOGI(TAG, "Set D to %d", data);
    }
    else if (!strcmp(g_type, "Set forward"))
    {
        control_loop_set_forward(data / 1000.f);
        ESP_LOGI(TAG, "Set forward to %d", data);
    }
    else if (!strcmp(g_type, "Enable"))
    {
        control_loop_set_enable(data);
        ESP_LOGI(TAG, "Set enable to %d", data);
    }
    else if (!strcmp(g_type, "Turbine"))
    {
        turbine_set_speed(data);
        ESP_LOGI(TAG, "Set turbine to %d", data);
    }
    else
    {
        ESP_LOGE(TAG, "Unknown command: %s", g_type);
    }
}

void cmd_parser_feed(char c)
{
    switch (g_state)
    {
        case STATE_TYPE:
            if (c == ';')
            {
                g_type[g_index] = '\0';
                g_index = 0;
                g_state = STATE_DATA;
            }
            else
            {
                g_type[g_index] = c;
                g_index++;
            }
            break;

        case STATE_DATA:
            if (c == '\n')
            {
                g_data[g_index] = '\0';
                cmd_parser_accept();
                g_index = 0;
                g_state = STATE_TYPE;
            }
            else
            {
                g_data[g_index] = c;
                g_index++;
            }
            break;
    }
}
