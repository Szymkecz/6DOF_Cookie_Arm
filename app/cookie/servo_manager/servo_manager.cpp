
#include "servo_manager.hpp"
#include "tim.h"

namespace ServoManager {
    // WAIST------------------
    constexpr uint16_t J1_MIN_PWM = 550;
    constexpr uint16_t J1_MAX_PWM = 2600;
    // SHOULDER---------------
    constexpr uint16_t J2_MIN_PWM = 550;
    constexpr uint16_t J2_MAX_PWM = 2450;
    // ELBOW------------------
    constexpr uint16_t J3_MIN_PWM = 600;
    constexpr uint16_t J3_MAX_PWM = 2450;
    // ROLL-------------------
    constexpr uint16_t J4_MIN_PWM = 350;
    constexpr uint16_t J4_MAX_PWM = 2600;
    // PITCH------------------
    // middle pos at 1600us
    constexpr uint16_t J5_MIN_PWM = 625;
    constexpr uint16_t J5_MAX_PWM = 2575;
    // YAW--------------------
    // middle pos at 1475
    constexpr uint16_t J6_MIN_PWM = 500;
    constexpr uint16_t J6_MAX_PWM = 2450;
    // GRIPPER-----------------
    constexpr uint16_t J7_MIN_PWM = 350;
    constexpr uint16_t J7_MAX_PWM = 2600;
    //-------------------------------------
    Servo SrvArray[SERVO_COUNT];

    void init()
    {
        // printf("initialized !\r\n");
        SrvArray[0].init(&htim3, TIM_CHANNEL_1, J1_MIN_PWM, J1_MAX_PWM);
        SrvArray[1].init(&htim3, TIM_CHANNEL_2, J2_MIN_PWM, J2_MAX_PWM);
        SrvArray[2].init(&htim1, TIM_CHANNEL_2, J3_MIN_PWM, J3_MAX_PWM);
        SrvArray[3].init(&htim1, TIM_CHANNEL_1, J4_MIN_PWM, J4_MAX_PWM);
        SrvArray[4].init(&htim1, TIM_CHANNEL_3, J5_MIN_PWM, J5_MAX_PWM);
        SrvArray[5].init(&htim1, TIM_CHANNEL_4, J6_MIN_PWM, J6_MAX_PWM);
        SrvArray[6].init(&htim3, TIM_CHANNEL_4, J7_MIN_PWM, J7_MAX_PWM);

        SrvArray[2].set_new_map(180, 0);
        // added offsets for
        SrvArray[0].set_angle_offset(90);
        SrvArray[1].set_angle_offset(87);
        SrvArray[2].set_angle_offset(79);
        SrvArray[3].set_angle_offset(100);
        SrvArray[4].set_angle_offset(90);
        SrvArray[5].set_angle_offset(90);
        SrvArray[6].set_angle_offset(90);
    }

    void set_pwm(uint16_t pwm_val[SERVO_COUNT])
    {
        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            SrvArray[i].set_pwm(pwm_val[i]);
        }
    }

    void set_angles(std::array<double, SERVO_COUNT>& angles)
    {
        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            SrvArray[i].set_angle(angles[i]);
        }
    }

    std::array<uint16_t, SERVO_COUNT> get_curr_pwm()
    {
        std::array<uint16_t, SERVO_COUNT> pwm;

        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            pwm[i] = SrvArray[i].get_curr_pwm();
        }
        return pwm;
    }

    std::array<double, SERVO_COUNT> get_curr_angles()
    {
        std::array<double, SERVO_COUNT> angles;

        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            angles[i] = SrvArray[i].get_curr_angle();
        }
        return angles;
    }

    void print_curr_pwm()
    {
        std::array<uint16_t, SERVO_COUNT> pwm;
        pwm = get_curr_pwm();

        char buffor[128];

        printf("current pwm: %u, %u, %u, %u, %u, %u, %u \r\n",
               pwm[0],
               pwm[1],
               pwm[2],
               pwm[3],
               pwm[4],
               pwm[5],
               pwm[6]);
    }

    void print_angles()
    {
        std::array<double, SERVO_COUNT> angles;
        angles = get_curr_angles();

        char buffor[128];

        printf("JC[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f];\r\n",
               angles[0],
               angles[1],
               angles[2],
               angles[3],
               angles[4],
               angles[5],
               angles[6]);
    }

    void print_servo_config()
    {
        static bool is_done = false;

        if (is_done) {
            return;
        }
        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            printf("----| %u |-----\r\n", i);
            SrvArray[i].print_config();
        }

        is_done = true;
    }

} // namespace ServoManager
