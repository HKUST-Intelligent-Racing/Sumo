#include "sensor.h"
#include "main.h"
#include "motor.h"

static uint16_t threshold = 500;  

volatile uint16_t dbg_ir_left = 0;
volatile uint16_t dbg_ir_right = 0;

static uint16_t read_qre(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &gpio);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 100; i++);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &gpio);

    uint16_t count = 0;
    while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
        count++;
        if (count > 5000) break; 
    }
    return count;
}

void Sensor_Init(void) {
}

EdgeState Sensor_ReadEdge(void) {
    EdgeState e;
    e.left_raw  = read_qre(GPIOA, GPIO_PIN_0);   // PA0
    e.right_raw = read_qre(GPIOA, GPIO_PIN_1);   // PA1

    dbg_ir_left  = e.left_raw;
    dbg_ir_right = e.right_raw;

    e.left  = (e.left_raw  > threshold) ? 1 : 0;
    e.right = (e.right_raw > threshold) ? 1 : 0;
    return e;
}

void Sensor_EdgeAvoid(void) {
    EdgeState e = Sensor_ReadEdge();

    if (e.left && e.right) {
        Motor_Set(-50, -50);
        HAL_Delay(350);
        Motor_Set(60, -60);
        HAL_Delay(650);
    } else if (e.left) {
        Motor_Set(30, -70);
        HAL_Delay(750);
    } else if (e.right) {
        Motor_Set(-70, 30);
        HAL_Delay(750);
    }
    Motor_Stop();
}