#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#include "dma.h"
#include "tim.h"
#include "usart.h"

extern uint8_t UART2_rxBuffer[21];
extern uint8_t UART2_txBuffer[54];

extern float f_val;
extern uint8_t uart_data[7];
extern char jog_type;

extern volatile bool is_htim3_time_done;

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif // CALLBACKS_HPP
