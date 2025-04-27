#ifndef SERVO_MANAGER_HPP
#define SERVO_MANAGER_HPP

#include "servo.hpp"
#include "stdio.h"
#include <array>

namespace ServoManager {

    constexpr uint8_t SERVO_COUNT = 7;
    extern Servo SrvArray[SERVO_COUNT];

    void init();
    void set_pwm(uint16_t pwm_val[SERVO_COUNT - 1]);
    void set_angles(std::array<double, SERVO_COUNT - 1>& angles);

    std::array<uint16_t, SERVO_COUNT - 1> get_curr_pwm();
    std::array<double, SERVO_COUNT - 1> get_curr_angles();

    // debugging
    void print_curr_pwm();
    void print_angles();
    void print_servo_config();

} // namespace ServoManager

#endif // SERVO_MANAGER_HPP