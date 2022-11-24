#include <lfw_esp/cmd_parser.h>

#include <esp_log.h>
#include <string.h>

#include <lfw_esp/turn_pid.h>


static const char TAG[] = "cmd_parser";

typedef enum state_t {
    STATE_TYPE,
    STATE_DATA,
} state_t;

static state_t g_state = STATE_TYPE;
static char g_type[64] = {0};
static int g_index = 0;
static int g_data = 0;
static bool g_negative = false;


static void cmd_parser_accept()
{
    if (!strcmp(g_type, "Set P"))
    {
        turn_pid_set_p(g_data / 1000.f);
        ESP_LOGI(TAG, "Set P to %d", g_data);
    }
    else if (!strcmp(g_type, "Set D"))
    {
        turn_pid_set_d(g_data / 1000.f);
        ESP_LOGI(TAG, "Set D to %d", g_data);
    }
    else
    {
        ESP_LOGE(TAG, "Unknown command: %s", g_type);
    }
}

void cmd_parser_feed(char c)
{
    // message format: type;data\n
    switch (g_state)
    {
        case STATE_TYPE:
            if (c == ';')
            {
                g_type[g_index] = '\0';
                g_data = 0;
                g_negative = false;
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
                if (g_negative)
                    g_data = -g_data;
                cmd_parser_accept();
                g_index = 0;
                g_state = STATE_TYPE;
            }
            else if (c == '-')
            {
                g_negative = true;
            }
            else
            {
                g_data = g_data * 10 + (c - '0');
            }
            break;
    }
}
