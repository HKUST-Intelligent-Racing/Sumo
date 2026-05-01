#include "lidar.h"
#include <string.h>
#include <math.h>

#define PKT_SIZE       47
#define PKT_HEADER     0x54
#define PKT_VER        0x2C
#define POINTS_PER_PKT 12

volatile uint16_t dbg_wr_idx = 0;
volatile uint16_t dbg_rd_idx = 0;
volatile uint16_t dbg_pkt_count = 0;
volatile uint16_t dbg_point_count = 0;
volatile uint8_t  dbg_first_byte = 0;

static const uint8_t CrcTable[256] = {
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae, 0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07, 0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1, 0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18, 0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90, 0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39, 0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f, 0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26, 0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2, 0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b, 0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd, 0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64, 0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec, 0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45, 0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3, 0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a, 0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

static uint8_t  ring[LIDAR_RING_SIZE];
static uint16_t rd_idx = 0;

typedef struct {
    float    angle;
    uint16_t dist;
    uint8_t  intensity;
} Point;

static Point    points[MAX_LIDAR_POINTS];
static volatile uint16_t point_count = 0;

static uint8_t calc_crc8(const uint8_t *p, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) crc = CrcTable[(crc ^ p[i]) & 0xFF];
    return crc;
}

void Lidar_Init(void) {
    HAL_UART_Receive_DMA(&huart1, ring, LIDAR_RING_SIZE);
}

static void parse_packet(const uint8_t *p) {
    if (calc_crc8(p, 46) != p[46]) return;

    float start = (p[4] | (p[5] << 8)) / 100.0f;
    float end   = (p[42] | (p[43] << 8)) / 100.0f;
    float step  = end - start;
    
    if (step < 0) step += 360.0f;
    step /= (POINTS_PER_PKT - 1);

    if (end < start) { point_count = 0; } 

    for (int i = 0; i < POINTS_PER_PKT; i++) {
        const uint8_t *q = &p[6 + i * 3];
        uint16_t dist = q[0] | (q[1] << 8);
        float ang = start + step * i;
        if (ang >= 360.0f) ang -= 360.0f;

        if (point_count < MAX_LIDAR_POINTS) {
            points[point_count].angle     = ang;
            points[point_count].dist      = dist;
            points[point_count].intensity = q[2];
            point_count++;
        }
    }
}

void Lidar_Process(void) {
    uint16_t wr_idx = LIDAR_RING_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    
    dbg_wr_idx = wr_idx;           
    dbg_rd_idx = rd_idx;           
    dbg_first_byte = ring[wr_idx > 0 ? wr_idx - 1 : 0]; 

    while (rd_idx != wr_idx) {
        if (ring[rd_idx] == PKT_HEADER) {
            uint16_t next = (rd_idx + 1) & (LIDAR_RING_SIZE - 1);
            if (ring[next] == PKT_VER) {
                uint16_t avail = (wr_idx - rd_idx) & (LIDAR_RING_SIZE - 1);
                if (avail < PKT_SIZE) break;

                uint8_t pkt[PKT_SIZE];
                for (int i = 0; i < PKT_SIZE; i++)
                    pkt[i] = ring[(rd_idx + i) & (LIDAR_RING_SIZE - 1)];

                parse_packet(pkt);
                dbg_pkt_count++;    
                rd_idx = (rd_idx + PKT_SIZE) & (LIDAR_RING_SIZE - 1);
                continue;
            }
        }
        rd_idx = (rd_idx + 1) & (LIDAR_RING_SIZE - 1);
    }
    
    dbg_point_count = point_count;
}

LidarTarget Lidar_GetNearest(float fov_deg, uint16_t min_mm, uint16_t max_mm) {
    LidarTarget t = { 0.0f, 9999 };
    float half_fov = fov_deg / 2.0f;

    for (uint16_t i = 0; i < point_count; i++) {
        float a = points[i].angle;
        if (a > 180.0f) a -= 360.0f; 

        if (a < -half_fov || a > half_fov) continue; 
        uint16_t d = points[i].dist;
        if (d < min_mm || d > max_mm) continue;    
        if (points[i].intensity < 10) continue;      

        if (d < t.dist_mm) {
            t.dist_mm = d;
            t.angle_deg = a;
        }
    }
    return t;
}

EnemyState Lidar_GetEnemyState(float fov_deg, uint16_t min_mm, uint16_t max_mm) {
    EnemyState state;
    LidarTarget t = Lidar_GetNearest(fov_deg, min_mm, max_mm);

    if (t.dist_mm <= max_mm) { 
        state.enemy_detected = 1;
        state.dist_mm = t.dist_mm;
        state.angle_deg = t.angle_deg;

        float turn = t.angle_deg / 30.0f; 
        
        if (turn > 1.0f) turn = 1.0f;
        if (turn < -1.0f) turn = -1.0f;
        state.turn_norm = turn;

        state.speed_norm = 1.0f - (fabsf(turn) * 0.3f); 
        
    } else { 
        state.enemy_detected = 0;
        state.dist_mm = 0;
        state.angle_deg = 0.0f;
        state.turn_norm = 0.0f;
        state.speed_norm = 0.0f;
    }
    return state;
}