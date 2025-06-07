#include "callbacks.hpp"

volatile bool is_htim3_time_done = false;
uint8_t UART2_rxBuffer[21] = {0};
uint8_t UART2_txBuffer[54] = {0};
float f_val = 0.0f;
uint8_t uart_data[7] = {0};

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        is_htim3_time_done = true;
    }
}
