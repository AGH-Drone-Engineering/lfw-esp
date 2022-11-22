#ifndef LFW_ESP_PID_H_
#define LFW_ESP_PID_H_


typedef struct {
    float kp;
    float kd;

    float last_error;
} pid_state_t;


float pid_update(pid_state_t *pid, float error, float dt);


#endif
