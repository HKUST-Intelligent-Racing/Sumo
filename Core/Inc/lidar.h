#ifndef __LIDAR_H
#define __LIDAR_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

#define LIDAR_RING_SIZE    2048 
#define MAX_LIDAR_POINTS   480

typedef struct {
    float    angle_deg;  //0=front, negative=left, positive=right 
    uint16_t dist_mm;    
} LidarTarget;

typedef struct {
    uint8_t  enemy_detected;
    float    angle_deg;
    uint16_t dist_mm;
    float    turn_norm; 
    float    speed_norm; 
} EnemyState;

extern volatile uint16_t dbg_wr_idx;
extern volatile uint16_t dbg_rd_idx;
extern volatile uint32_t dbg_pkt_count;
extern volatile uint32_t dbg_crc_fail;
extern volatile uint32_t dbg_frame_count;
extern volatile uint16_t dbg_build_cnt;
extern volatile uint16_t dbg_frame_cnt;

void        Lidar_Init(void);
void        Lidar_Process(void);
LidarTarget Lidar_GetNearest(float fov_deg, uint16_t min_mm, uint16_t max_mm);
EnemyState  Lidar_GetEnemyState(float fov_deg, uint16_t min_mm, uint16_t max_mm);

#endif