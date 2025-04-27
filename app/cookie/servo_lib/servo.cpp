#include <servo.hpp>

float remapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Servo::Servo(TIM_HandleTypeDef* timer,
             uint32_t channel,
             uint16_t min_pwm,
             uint16_t max_pwm)
{
    init(timer, channel, min_pwm, max_pwm);
}

void Servo::init(TIM_HandleTypeDef* timer,
                 uint32_t channel,
                 uint16_t min_pwm,
                 uint16_t max_pwm)
{
    if (is_initialized_)
        return;

    timer_ = timer;
    channel_ = channel;

    HAL_TIM_PWM_Start(timer, channel);
    min_pwm_ = std::max(min_pwm, MIN_PULSE_WIDTH);
    max_pwm_ = std::min(max_pwm, MAX_PULSE_WIDTH);

    is_initialized_ = true;
}

void Servo::set_new_map(int16_t min_map, int16_t max_map)
{
    min_angle_map_ = min_map;
    max_angle_map_ = max_map;
}

void Servo::set_angle_offset(int16_t offset)
{
    offset_angle_map_ = offset;
}

void Servo::turn_off()
{
    HAL_TIM_PWM_Stop(timer_, channel_);
}

void Servo::turn_on()
{
    HAL_TIM_PWM_Start(timer_, channel_);
}

void Servo::set_pwm(uint16_t pwm)
{
    if (pwm + PWM_DEADBAND > prev_pwm_ || pwm - PWM_DEADBAND < prev_pwm_) {
        curr_pwm_ = pwm;
        curr_pwm_ = std::clamp(curr_pwm_, min_pwm_, max_pwm_);
        prev_pwm_ = curr_pwm_;
    } else {
        curr_pwm_ = prev_pwm_;
    }

    __HAL_TIM_SET_COMPARE(timer_, channel_, curr_pwm_);
}

void Servo::set_angle(double angle)
{
    // // update angle in structure
    // curr_angle_ = std::clamp(angle,
    //                          static_cast<double>(get_min_angle_map()),
    //                          static_cast<double>(get_max_angle_map()));
    // // remap
    // double offset_pos = angle;
    // int16_t result = round(remapf(offset_pos,
    //                               get_min_angle_map(),
    //                               get_max_angle_map(),
    //                               min_pwm_,
    //                               max_pwm_));

    // straighten the angle and then use basic map//default 0-180
    double offseted_angle = angle + static_cast<double>(offset_angle_map_);

    double min_angle = std::min(min_angle_map_, max_angle_map_);
    double max_angle = std::max(min_angle_map_, max_angle_map_);

    curr_angle_ = std::clamp(offseted_angle, min_angle, max_angle) -
                  static_cast<double>(offset_angle_map_);

    int16_t result = round(remapf(offseted_angle,
                                  static_cast<double>(min_angle_map_),
                                  static_cast<double>(max_angle_map_),
                                  static_cast<double>(min_pwm_),
                                  static_cast<double>(max_pwm_)));
    // add constraint
    // result = constrain()

    set_pwm(result);
}
void Servo::print_config() const noexcept
{
    printf("Servo config:\r\n");
    printf("  Channel: %lu\r\n", channel_);
    printf("  PWM Range: [%u | %u]\r\n", min_pwm_, max_pwm_);
    printf("  Current PWM: %u\r\n", curr_pwm_);
    printf("  Current Angle: %.2f°\r\n", curr_angle_);
    printf("  Angle Map: [%d | %d]\r\n",
           get_min_angle_map(),
           get_max_angle_map());
    printf("  Offset: %d\r\n", offset_angle_map_);
    printf("--------------------------\r\n");
}
