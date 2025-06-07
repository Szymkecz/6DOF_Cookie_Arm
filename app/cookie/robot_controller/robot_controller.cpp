
#include "robot_controller.hpp"
#include "kinematics.hpp"
#include "servo_manager.hpp"
#include <array>

using namespace Eigen;

void RobotController::init()
{
    ServoManager::init();
    std::array<double, SERVO_COUNT> init_angles = {};
    ServoManager::set_angles(init_angles);
    ServoManager::print_servo_config();

    Kinematics::calc_T6_0(init_angles);
    Kinematics::print_cords();
    HAL_Delay(200);

    // WATCH OUT!!!!!!!!!!!!!!
    // angles[0] = 90;
    // angles[4] = 2;
    //---------------------
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
            // ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);

        // go to max negative angle
        for (double i = ServoManager::SrvArray[srv_n].get_curr_angle();
             i >= ServoManager::SrvArray[srv_n].get_min_angle_map();
             i -= speed) {
            angles[srv_n] = i;
            ServoManager::set_angles(this->angles);
            // ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);

        // go to zero position
        for (double i = ServoManager::SrvArray[srv_n].get_curr_angle();
             i <= 0.0;
             i += speed) {
            angles[srv_n] = i;
            ServoManager::set_angles(this->angles);
            // ServoManager::print_angles();
            HAL_Delay(20);
        }

        HAL_Delay(20);
        srv_n--;
    }
    is_done = true;
    ServoManager::set_angles(this->angles);
    // ServoManager::print_angles();
}

void RobotController::update(float f_val, uint8_t uart_data[7], char jog_type)
{
    // // ptp - variables
    // Matrix<double, 6, 1> destination;
    // Matrix<double, 6, 1> delta_x;
    // Matrix<double, 6, 1> dq;
    // static bool is_done = false;

    switch (current_mode) {
        case RobotMode::Jog:
            if (jog_type == 'j') {
                std::array<double, SERVO_COUNT> new_angles;
                new_angles = ServoManager::get_curr_angles();
                // move arm depending on sent data
                float displacement[7];
                for (uint8_t i = 0; i < SERVO_COUNT; i++) {
                    if (uart_data[i] == 1) {
                        displacement[i] = f_val;
                    } else if (uart_data[i] == 2) {
                        displacement[i] = -f_val;
                    } else {
                        displacement[i] = 0;
                    }
                    new_angles[i] += static_cast<double>(displacement[i]);
                }

                Kinematics::calc_T6_0(new_angles);
                // Kinematics::print_cords();
                ServoManager::set_angles(new_angles);
            }

            break;

            // case RobotMode::Ptp_demo:
            //     destination << 0.000, 257.338, 307.509, -1.571, -1.536,
            //     0.000;

            //     if ((destination -
            //     Kinematics::cords).array().abs().maxCoeff() >
            //             1.0 &&
            //         is_done == false) {
            //         // step 1-2
            //         delta_x = 0.1 * (destination - Kinematics::cords);
            //         // step 3
            //         Kinematics::calc_jacobian(this->angles);
            //         // step 4-5
            //         dq = Kinematics::calc_inv_jacobian() * delta_x ;
            //         // step 6-7
            //         std::array<double, 6> new_q;
            //         for (size_t i = 0; i < 6; ++i) {
            //             this->angles[i] += dq(i);
            //         }
            //         Kinematics::calc_T6_0(this->angles);
            //     } else {
            //         is_done = true;
            //     }
            //     ServoManager::set_angles(this->angles);
            //     break;

        case RobotMode::Ptp_demo: {
            std::array<std::array<double, SERVO_COUNT>, 5> trajectory = {
                {{90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                 {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
                 {3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0},
                 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}};
            uint16_t max_elem;
            std::array<double, SERVO_COUNT> dq = {0.0};

            if (!is_calc_done) {
                max_elem = *std::max_element(trajectory[current_step].begin(),
                                             trajectory[current_step].end());
                // printf("%u\n", (unsigned int)max_elem);
                // kp = max_element;
                is_calc_done = true;
            } else {
                for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
                    if (i == 0) {
                        this->angles[i] +=
                            std::max(kp * (trajectory[current_step][i] -
                                           this->angles[i]),
                                     1.0);
                        continue;
                    }
                    this->angles[i] +=
                        kp * (trajectory[current_step][i] - this->angles[i]);
                }
            }

            ServoManager::set_angles(this->angles);
            ServoManager::print_angles();
        } break;
        default:
            break;
    }
}