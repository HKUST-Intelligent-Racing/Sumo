#ifndef RECEIVER_H
#define RECEIVER_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t last_capture;
    uint8_t  last_state;        
    uint16_t pulse_width;      
    uint32_t last_update;     
} Receiver;

void Receiver_Init(Receiver *r, TIM_HandleTypeDef *htim, uint32_t channel);
void Receiver_HandleCapture(Receiver *r);   
uint8_t Receiver_IsAlive(Receiver *r);       
int16_t Receiver_GetScaled(Receiver *r, int16_t range); 

#endif