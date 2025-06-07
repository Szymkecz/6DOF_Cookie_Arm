#include "callbacks.hpp"
#include "kinematics.hpp"
#include "servo_manager.hpp"
#include <array>

volatile bool is_htim3_time_done = false;
uint8_t UART2_rxBuffer[21] = {0};
uint8_t UART2_txBuffer[54] = {0};
float f_val = 0.0f;
uint8_t uart_data[7] = {0};
char jog_type = 'j';

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        is_htim3_time_done = true;
    }
}

bool parse_uart_message(const char* msg, float* f_val, uint8_t uart_data[7])
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
        jog_type = 'j'; //move in joint space 
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
        std::array<double, ServoManager::SERVO_COUNT>
            reset_angles{}; // zero init
        ServoManager::set_angles(reset_angles);
        Kinematics::calc_T6_0(reset_angles);
        return true;
    }

    return false;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
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
