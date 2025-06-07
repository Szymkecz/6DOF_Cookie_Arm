#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "callbacks.hpp"
#include "kinematics.hpp"
#include "robot_controller.hpp"
#include "servo_manager.hpp"
#include "stdio.h"
#include "string.h"

// temp--------------
#include <array>
// ------------------

/* uint8_t UART2_rxBuffer[21] = {0};
uint8_t UART2_txBuffer[54] = {0};
volatile bool is_htim3_time_done = false;
 */
// float f_val;
// uint8_t uart_data[7];

/* bool parse_uart_message(const char* msg, float* f_val, uint8_t uart_data[7])
{
    // Remove trailing spaces (optional)
    char trimmed[22]; // 21 chars + null terminator
    strncpy(trimmed, msg, 21);
    trimmed[21] = '\0'; // Ensure null-terminated

    // Optionally trim trailing whitespace
    for (int i = 20; i >= 0; --i) {
        if (trimmed[i] == ' ' || trimmed[i] == '\0') {
            trimmed[i] = '\0';
        } else {
            break;
        }
    }

    // Type 1: Joint movement message
    if (strncmp(trimmed, "JN[", 3) == 0) {
        float temp_float;
        int temp_array[7];

        int parsed = sscanf(trimmed,
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
            return false;

        *f_val = temp_float;
        for (int i = 0; i < 7; ++i) {
            uart_data[i] = (uint8_t)temp_array[i];
        }
        return true;
    }

    // Type 2: POWER_ON command
    if (strncmp(trimmed, "POWER_ON;", 9) == 0) {
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
        return true;
    }
    if (strncmp(trimmed, "POWER_OFF;", 10) == 0) {
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
        return true;
    }
    if (strncmp(trimmed, "RESET;", 6) == 0) {
        std::array<double, SERVO_COUNT> reset_angles = {0.0};
        ServoManager::set_angles(reset_angles);
        Kinematics::calc_T6_0(reset_angles);
        return true;
    }

    return false;
}
 */

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
    HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET); // ON

    RobotController Controller;
    Controller.init();

    HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 21);
    // HAL_UART_Transmit_DMA(&huart2, UART2_txBuffer, sizeof(UART2_txBuffer));

    while (true) {
        if (is_htim3_time_done) {
            ServoManager::print_angles();
            Kinematics::print_cords();
            Controller.update(f_val, uart_data,jog_type);
            // Controller.play_demo();
            // ServoManager::print_angles();

            is_htim3_time_done = false;
        }
    }
    return 0;
}

 
/* void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
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
} */

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
// {
//     if (huart->Instance == USART2) // Make sure it's USART2
//     {
//         // Get the current angles (assuming ServoManager::get_curr_angles()
//         // returns a std::array or similar)
//         std::array<double, 7> cr_ang = ServoManager::get_curr_angles();

//         // Define the buffer for the message (54 bytes as per your setting)
//         char buff[54] = {0}; // Initialize the buffer with zeroes (empty
//         space)

//         // Format the message using snprintf
//         int message_length =
//             snprintf(buff,
//                      sizeof(buff) - 3, // Ensure space for \r\n at the end
//                      "JC[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f];",
//                      cr_ang[0],
//                      cr_ang[1],
//                      cr_ang[2],
//                      cr_ang[3],
//                      cr_ang[4],
//                      cr_ang[5],
//                      cr_ang[6]);

//         // Ensure the message fits within the buffer (leaving space for \r\n)
//         if (message_length < sizeof(buff) - 2) {
//             // Pad the remaining space with spaces, leaving space for \r\n
//             memset(buff + message_length,
//                    ' ',
//                    sizeof(buff) - message_length - 2);
//         }

//         // Add \r\n at the end of the message
//         buff[sizeof(buff) - 2] = '\r'; // Carriage return
//         buff[sizeof(buff) - 1] = '\n'; // Newline

//         // Now copy the padded buffer to UART2_txBuffer
//         memcpy(UART2_txBuffer, buff, sizeof(buff));

//         // Transmit via DMA
//         HAL_UART_Transmit_DMA(&huart2, UART2_txBuffer,
//         sizeof(UART2_txBuffer));
//     }
// }
