#ifndef LFW_ESP_TCP_SERVER_H_
#define LFW_ESP_TCP_SERVER_H_


void tcp_server_init(void);

void tcp_server_start(void);

void tcp_server_send_motors(int left, int right);

void tcp_server_send_angle(int angle);


#endif
