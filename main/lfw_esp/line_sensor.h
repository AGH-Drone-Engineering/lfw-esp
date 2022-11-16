#ifndef LFW_ESP_LINE_SENSOR_H_
#define LFW_ESP_LINE_SENSOR_H_


#include <stdint.h>


#define LINE_SENSOR_N (8)


typedef struct {
    uint32_t values[LINE_SENSOR_N];
} line_sensor_measurement_t;


int line_sensor_init(void);

void line_sensor_measurement(line_sensor_measurement_t *measurement);


#endif
