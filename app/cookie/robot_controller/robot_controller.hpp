#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "array"
#include "servo_manager.hpp"
using namespace std;
using namespace ServoManager;

struct RobotController {
public:
    void init();
    void play_demo();
    void update(float f_val, uint8_t uart_data[7]);

private:
    array<double, SERVO_COUNT - 1> angles = {0.0};
};

#endif // ROBOT_CONTROLLER_HPP