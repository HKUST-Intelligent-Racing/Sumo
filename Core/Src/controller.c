#include "controller.h"
#include "receiver.h"
#include "motor.h"
#include "sensor.h"
#include "stm32f4xx_hal.h"
#include "lidar.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;

static Receiver rx_turning;   // CH1 PA6 LR
static Receiver rx_throttle;   // CH2 PA7 UD
static Receiver rx_switch;   // CH3 PB0 auto/manual
//static uint32_t next_turn = 0; //use for demo

volatile uint8_t  dbg_auto_enemy_active = 0;
volatile float    dbg_auto_enemy_angle = 0;
volatile uint16_t dbg_auto_enemy_dist = 0;
volatile int16_t  dbg_auto_left = 0;
volatile int16_t  dbg_auto_right = 0;

void Controller_Init(void) {
    Receiver_Init(&rx_turning, &htim3, TIM_CHANNEL_2);  // PA6
    Receiver_Init(&rx_throttle, &htim3, TIM_CHANNEL_1);  // PA7
    Receiver_Init(&rx_switch, &htim3, TIM_CHANNEL_3);   // PB0
}

static uint8_t is_auto_mode(void) {
    //C: >1500 = auto, <1500 = maual
    if (!Receiver_IsAlive(&rx_switch)) return 0; 
    return (rx_switch.pulse_width > 1700) ? 1 : 0;
}

static void manual_mode(void) {
    if (!Receiver_IsAlive(&rx_throttle)
        || !Receiver_IsAlive(&rx_turning))
    {
        Motor_Stop();
        return;
    }

    int16_t throttle =  Receiver_GetScaled(&rx_throttle, 100);
    int16_t turning = -Receiver_GetScaled(&rx_turning, 100);

    if (throttle > -10 && throttle < 10) throttle = 0;
    if (turning > -10 && turning < 10) turning = 0;

    int16_t left  = throttle + turning;
    int16_t right = throttle - turning;
    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    Motor_Set((int8_t)left, (int8_t)right);
}

static void auto_mode(void) {
    
    /*EdgeState e = Sensor_ReadEdge();
    if (e.left || e.right) {
        Sensor_EdgeAvoid();
        return;
    }*/
    EnemyState enemy = Lidar_GetEnemyState(180.0f, 10, 300);
    dbg_auto_enemy_active = enemy.enemy_detected;
    dbg_auto_enemy_angle  = enemy.angle_deg;
    dbg_auto_enemy_dist   = enemy.dist_mm;
    
    if (enemy.enemy_detected) {
        float angle = enemy.angle_deg;

        float turn_norm = angle / 45.0f;
        if (turn_norm >  1.0f) turn_norm =  1.0f;
        if (turn_norm < -1.0f) turn_norm = -1.0f;

        int16_t left, right;

        if (fabsf(angle) > 20.0f) {
            int16_t spin = (int16_t)(turn_norm * 70);
            left  = -spin;
            right = spin;
        } else {
            /*int16_t throttle = 80;
            int16_t steering = (int16_t)(turn_norm 30);
            left  = throttle + steering;
            right = throttle - steering;*/
            left  = 0;
            right = 0;
        }

        if (left  >  100) left  =  100;
        if (left  < -100) left  = -100;
        if (right >  100) right =  100;
        if (right < -100) right = -100;

        dbg_auto_left  = left;
        dbg_auto_right = right;
        Motor_Set((int8_t)left, (int8_t)right);
    } else {
        Motor_Stop();
    }
}

/*static void auto_mode(void) {
    EdgeState e = Sensor_ReadEdge();
    if (e.left || e.right) {
        Sensor_EdgeAvoid();
        next_turn = HAL_GetTick() + 1000;
        return;
    }
    if (HAL_GetTick() > next_turn) {
        uint32_t r = HAL_GetTick() % 3;
        if (r == 0) {
            Motor_Set(50, -50);
            HAL_Delay(200);
        } else if (r == 1) {
            Motor_Set(-50, 50);
            HAL_Delay(200);
        }
        next_turn = HAL_GetTick() + 800 + (HAL_GetTick() % 500);
    }

    Motor_Set(80, 80);
}*/

void Controller_Update(void) {
    if (is_auto_mode()) {
        auto_mode();
    } else {
        manual_mode();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Receiver_HandleCapture(&rx_turning); break;
            case HAL_TIM_ACTIVE_CHANNEL_1:
                Receiver_HandleCapture(&rx_throttle); break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                Receiver_HandleCapture(&rx_switch); break;
            default: break;
        }
    }
}