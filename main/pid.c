#include <lfw_esp/pid.h>


float pid_update(pid_state_t *pid, float error, float dt)
{
    float oP = -pid->kp * error;

    float oD = 0;
    if (dt > 0) {
        float dE = error - pid->last_error;
        pid->last_error = error;
        oD = -pid->kd * dE / dt;
    }

    return oP + oD;
}
