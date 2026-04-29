#ifndef __LIDAR_H
#define __LIDAR_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;

#define LIDAR_RING_SIZE    2048
#define MAX_LIDAR_POINTS   480

typedef struct {
    float    angle_deg; 
    uint16_t dist_mm;  
} LidarTarget;

typedef struct {
    uint8_t  enemy_detected;    
    float    angle_deg;    
    uint16_t dist_mm;      
    
    float    turn_norm;    
    float    speed_norm;   
} EnemyState;

void Lidar_Init(void);
void Lidar_Process(void);
LidarTarget Lidar_GetNearest(float fov_deg, uint16_t min_mm, uint16_t max_mm);
EnemyState  Lidar_GetEnemyState(float fov_deg, uint16_t min_mm, uint16_t max_mm);

#endif