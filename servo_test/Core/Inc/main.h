#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart4;

uint8_t* read_UART_buffer(void);
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
