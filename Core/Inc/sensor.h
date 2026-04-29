#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct {
    uint8_t  left;
    uint8_t  right;
    uint16_t left_raw;
    uint16_t right_raw;
} EdgeState;

void Sensor_Init(void);
EdgeState Sensor_ReadEdge(void);
void Sensor_EdgeAvoid(void);

#endif