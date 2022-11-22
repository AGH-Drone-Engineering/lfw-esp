#include <lfw_esp/pid.h>


int pid_update(pid_t *pid, int error, int dt)
{
    int oP = -(pid->kp * error) >> 8;

    int oD = 0;
    if (dt > 0) {
        int dE = error - pid->last_error;
        pid->last_error = error;
        oD = -(pid->kd * dE) / dt;
    }

    return oP + oD;
}
