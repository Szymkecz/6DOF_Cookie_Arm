#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "robot_controller.hpp"
#include "servo_manager.hpp"
#include "stdio.h"
#include "string.h"
#include "Eigen/Dense"

uint8_t UART2_rxBuffer[21] = {0};
volatile bool is_htim3_time_done = false;

Eigen::Matrix4d T6_0 = Eigen::Matrix4d::Identity();
float f_val;
uint8_t uart_data[7];

bool parse_uart_message(const char* msg, float* f_val, uint8_t uart_data[7])
{
    if (strncmp(msg, "JN[", 3) != 0)
        return false; // check prefix

    // Temporary variables
    float temp_float;
    int temp_array[7];

    // Parse using sscanf
    int parsed = sscanf(msg,
                        "JN[%f,%d,%d,%d,%d,%d,%d,%d];",
                        &temp_float,
                        &temp_array[0],
                        &temp_array[1],
                        &temp_array[2],
                        &temp_array[3],
                        &temp_array[4],
                        &temp_array[5],
                        &temp_array[6]);

    if (parsed != 8)
        return false; // Make sure we got all 8 values

    *f_val = temp_float;

    // Copy into uint8_t array
    for (int i = 0; i < 7; ++i) {
        uart_data[i] = (uint8_t)temp_array[i];
    }

    return true;
}

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    // GLOBAL REACH
    // HAL_ADC_Start_DMA(&hadc1, &adc_buffor, 1);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

    RobotController Controller;
    Controller.init();

    ServoManager::print_servo_config();
    HAL_Delay(200);

    HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 21);

    while (true) {
        // ServoManager::SrvArray[2].print_config();
        // Controller.play_demo();
        if (is_htim3_time_done) {
            // ServoManager::print_angles();
            // if (uart_data[5] != 0) {
            //     ServoManager::SrvArray[5].set_angle(
            //         ServoManager::SrvArray[5].get_curr_angle() +
            //         uart_data[5]);
            // }
            printf("%.1f\r\n",T6_0(0,0));
            Controller.update(f_val, uart_data);
            is_htim3_time_done = false;
        }
        // HAL_Delay(20);
    }
    return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        is_htim3_time_done = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2) // Make sure it's USART2
    {
        // Echo or handle response
        // HAL_UART_Transmit(&huart2, UART2_rxBuffer, 21, 100);

        // Optionally parse the message first

        if (parse_uart_message((char*)UART2_rxBuffer, &f_val, uart_data)) {
            // // Format and transmit f_val + data
            // char msg[32];
            // int len = snprintf(msg, sizeof(msg),
            //                    "%.1f %u %u %u %u %u %u %u\r\n",
            //                    f_val,
            //                    data[0], data[1], data[2], data[3],
            //                    data[4], data[5], data[6]);

            // HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
        }

        // Re-arm the DMA to receive the next message
        HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 21);
    }
}