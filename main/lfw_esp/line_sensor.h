#ifndef LFW_ESP_LINE_SENSOR_H_
#define LFW_ESP_LINE_SENSOR_H_


#include <stdint.h>


#define LINE_SENSOR_N (8)


void line_sensor_init(void);

void line_sensor_measurement(uint32_t *line_position);

void line_sensor_calibrate(void);


#endif
