
#include "robot_controller.hpp"

void RobotController::init()
{
    ServoManager::init();
    ServoManager::set_angles(this->angles);
    ServoManager::print_servo_config();
    HAL_Delay(200);
}

void RobotController::play_demo()
{
    static bool is_done = false;
    static int8_t srv_n = 5; // servo number
    static float speed = 0.5;

    while (!is_done && srv_n >= 2) {
        if (srv_n >= 2) { // for j4-j6 speeds are higher
            speed = 1.0;
        } else {
            speed = 0.5;
        }

        // go to max positive angle
        for (double i = ServoManager::SrvArray[srv_n].get_curr_angle();
             i <= ServoManager::SrvArray[srv_n].get_max_angle_map();
             i += speed) {
            angles[srv_n] = i;
            ServoManager::set_angles(this->angles);
            ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);

        // go to max negative angle
        for (double i = ServoManager::SrvArray[srv_n].get_curr_angle();
             i >= ServoManager::SrvArray[srv_n].get_min_angle_map();
             i -= speed) {
            angles[srv_n] = i;
            ServoManager::set_angles(this->angles);
            ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);

        // go to zero position
        for (double i = ServoManager::SrvArray[srv_n].get_curr_angle();
             i <= 0.0;
             i += speed) {
            angles[srv_n] = i;
            ServoManager::set_angles(this->angles);
            ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);
        srv_n--;
    }
    is_done = true;
    ServoManager::set_angles(this->angles);
    ServoManager::print_angles();
}

void RobotController::update(float f_val, uint8_t uart_data[7])
{
    float displacement[7];
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (uart_data[i] == 1) {
            displacement[i] = f_val;
        } else if (uart_data[i] == 2) {
            displacement[i] = -f_val;
        } else {
            displacement[i] = 0;
        }
        angles[i] += static_cast<double>(displacement[i]);
    }

    ServoManager::set_angles(this->angles);
}