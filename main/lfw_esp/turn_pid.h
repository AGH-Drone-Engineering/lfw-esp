#ifndef LFW_ESP_TURN_PID_H_
#define LFW_ESP_TURN_PID_H_


float turn_pid_update(float error);

void turn_pid_set_p(float p);

void turn_pid_set_d(float d);


#endif
