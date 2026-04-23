#include "receiver.h"

#define PULSE_MIN    1000
#define PULSE_MAX    2000
#define PULSE_MID    1500
#define DEADBAND     40        
#define TIMEOUT_MS   200       

volatile uint32_t debug_raw_width = 0;

void Receiver_Init(Receiver *r, TIM_HandleTypeDef *htim, uint32_t channel)
{
    r->htim = htim;
    r->channel = channel;
    r->last_capture = 0;
    r->last_state = 0;
    r->pulse_width = PULSE_MID;    
    r->last_update = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(htim, channel);
}

void Receiver_HandleCapture(Receiver *r)
{
    uint32_t cap = HAL_TIM_ReadCapturedValue(r->htim, r->channel);
    
    if (r->last_state == 0) {
        r->last_capture = cap;
        r->last_state = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(r->htim, r->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        uint32_t width;
        if (cap >= r->last_capture) width = cap - r->last_capture;
        else width = (0xFFFF - r->last_capture) + cap;
        
        debug_raw_width = width;
        
        if (width >= 800 && width <= 2200) {
            r->pulse_width = (uint16_t)width;
            r->last_update = HAL_GetTick();
        }
        r->last_state = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(r->htim, r->channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

uint8_t Receiver_IsAlive(Receiver *r)
{
    return (HAL_GetTick() - r->last_update) < TIMEOUT_MS;
}

int16_t Receiver_GetScaled(Receiver *r, int16_t range)
{
    if (!Receiver_IsAlive(r)) return 0;
    int32_t diff = (int32_t)r->pulse_width - PULSE_MID;
    if (diff > -DEADBAND && diff < DEADBAND) return 0;
  
    if (diff > 0) diff -= DEADBAND;
    else diff += DEADBAND;
  
    int32_t scaled = diff * range / (500 - DEADBAND);
    if (scaled >  range) scaled =  range;
    if (scaled < -range) scaled = -range;
    return (int16_t)scaled;
}