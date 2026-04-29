#include "motor.h"
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim8;

#define ESC_STOP  1500
#define ESC_MIN    500
#define ESC_MAX   2500

static int8_t dir_m1 = -1; 
static int8_t dir_m2 = -1;

static int8_t last_left  = 0;
static int8_t last_right = 0;

static uint16_t pct_to_us(int16_t pct) {
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;
    return (uint16_t)(ESC_STOP + pct * 5);
}

void Motor_Init(void) {
    // M1 = PC7 = TIM8_CH2,  M2 = PC6 = TIM8_CH1
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ESC_STOP);  // M2
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ESC_STOP);  // M1

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    HAL_Delay(500);
}

void Motor_Set(int8_t left, int8_t right) {
    if ((last_left > 5 && left < -5) || (last_left < -5 && left > 5)) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ESC_STOP);
        HAL_Delay(30);
    }
    if ((last_right > 5 && right < -5) || (last_right < -5 && right > 5)) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ESC_STOP);
        HAL_Delay(30);
    }

    last_left  = left;
    last_right = right;

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
                          pct_to_us(left  * dir_m1));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
                          pct_to_us(right * dir_m2));
}

void Motor_Brake(void) {
    if (last_left > 0 || last_right > 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pct_to_us(-15 * dir_m1));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pct_to_us(-15 * dir_m2));
    } else if (last_left < 0 || last_right < 0) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pct_to_us(15 * dir_m1));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pct_to_us(15 * dir_m2));
    }
    HAL_Delay(50);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ESC_STOP);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ESC_STOP);
    last_left = 0;
    last_right = 0;
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ESC_STOP);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, ESC_STOP);
}

void esc_calibrate(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2500);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 2500);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    HAL_Delay(5000);   

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 500);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 500);
    HAL_Delay(3000);  

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1500);
    HAL_Delay(2000);

}