#ifndef LFW_ESP_PID_H_
#define LFW_ESP_PID_H_


typedef struct {
    int kp;
    int kd;

    int last_error;
} pid_t;


int pid_update(pid_t *pid, int error, int dt);


#endif
