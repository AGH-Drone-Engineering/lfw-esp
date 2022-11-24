#include <lfw_esp/turn_pid.h>


static float g_kp = 0.5f;
static float g_kd = 0.0f;

static float g_last_error = 0.0f;


float turn_pid_update(float error, float dt)
{
    float oP = -g_kp * error;

    float oD = 0;
    if (dt > 0)
    {
        float dE = error - g_last_error;
        g_last_error = error;
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
