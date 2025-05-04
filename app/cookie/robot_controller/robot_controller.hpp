#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "array"
#include "servo_manager.hpp"
using namespace std;
using namespace ServoManager;

struct RobotController {
public:
    enum class RobotMode { Jog, Ptp_demo, Manual };

    void init();
    void play_demo();
    void update(float f_val, uint8_t uart_data[SERVO_COUNT]);

private:
    array<double, SERVO_COUNT> angles = {0.0};
    RobotMode current_mode = RobotMode::Jog;
};

#endif // ROBOT_CONTROLLER_HPP