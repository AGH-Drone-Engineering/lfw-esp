#include <lfw_esp/turn_pid.h>

#include <freertos/FreeRTOS.h>


static float g_kp = 0.0f;
static float g_kd = 0.0f;

static float g_last_error1 = 0.0f;
static float g_last_error2 = 0.0f;
static uint32_t g_last_time = 0;


float turn_pid_update(float error)
{
    uint32_t time = portGET_RUN_TIME_COUNTER_VALUE();
    float dt = (time - g_last_time) / 655.f;
    g_last_time = time;

    float oP = -g_kp * error;

    float oD = 0;
    if (dt > 0)
    {
        float dE = g_last_error2 - 4 * g_last_error1 + 3 * error;
        g_last_error2 = g_last_error1;
        g_last_error1 = error;
        oD = -g_kd * dE / dt;
    }

    return oP + oD;
}

void turn_pid_set_p(float p)
{
    g_kp = p;
}

void turn_pid_set_d(float d)
{
    g_kd = d;
}
