#include "controller.h"
#include "receiver.h"
#include "motor.h"

extern TIM_HandleTypeDef htim3;

static Receiver rx_turning;   // CH1 PA6 LR
static Receiver rx_throttle;   // CH2 PA7 UD

void Controller_Init(void) {
    Receiver_Init(&rx_turning, &htim3, TIM_CHANNEL_2);  // PA6
    Receiver_Init(&rx_throttle, &htim3, TIM_CHANNEL_1);  // PA7
}

void Controller_Update(void) {
    if (!Receiver_IsAlive(&rx_turning)
        || !Receiver_IsAlive(&rx_throttle))
    {
        Motor_Stop();
        return;
    }

    int16_t throttle = Receiver_GetScaled(&rx_throttle, 100);
    int16_t steering = Receiver_GetScaled(&rx_turning, 100);

    int16_t left  = throttle + steering;
    int16_t right = throttle - steering;
    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    Motor_Set((int8_t)left, (int8_t)right);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_2:
                Receiver_HandleCapture(&rx_turning); break;
            case HAL_TIM_ACTIVE_CHANNEL_1:
                Receiver_HandleCapture(&rx_throttle); break;
            default: break;
        }
    }
}