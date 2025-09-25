#ifndef __DS18B20_H
#define __DS18B20_H

#include "stm32f1xx_hal.h"

#define DS18B20_PORT GPIOA
#define DS18B20_PIN  GPIO_PIN_1

void DS18B20_Init(void);
float DS18B20_ReadTemp(void);
void DS18B20_StartConversion(void);
uint8_t DS18B20_IsBusy(void);            // 1 = đang bận; 0 = xong
float DS18B20_ReadTemp_NoDelay(void);    // đọc scratchpad, KHÔNG delay
#endif
