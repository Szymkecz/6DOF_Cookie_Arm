#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#include "tim.h"

extern uint8_t UART2_rxBuffer[21];
extern uint8_t UART2_txBuffer[54];

extern float f_val;
extern uint8_t uart_data[7];

extern volatile bool is_htim3_time_done;

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif // CALLBACKS_HPP
