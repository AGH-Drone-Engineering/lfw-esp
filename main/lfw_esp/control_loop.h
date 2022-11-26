#ifndef LFW_ESP_CONTROL_LOOP_H_
#define LFW_ESP_CONTROL_LOOP_H_


void control_loop_init(void);

void control_loop_calibrate(void);

void control_loop_start(void);

void control_loop_set_forward(float speed);

void control_loop_set_enable(int enable);


#endif
