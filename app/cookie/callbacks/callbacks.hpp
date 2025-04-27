#ifndef CALLBACKS_HPP
#define CALLBACKS_HPP

#ifdef __cplusplus
extern "C" {
#endif
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // CALLBACKS_HPP