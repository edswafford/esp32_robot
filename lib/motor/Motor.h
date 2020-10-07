#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "neo_base_config.h"
class Controller
{
public:
    Controller(int pwm_channel, int pwm_pin, int motor_pinA, int motor_pinB);
    void spin(int pwm);

    // Setting PWM properties
    static const int freq = 20000;
    static const int PWM_CHANNEL_1 = 0;
    static const int PWM_CHANNEL_2 = 1;
    static const int resolution = PWM_BITS;
    static const int MAX_DUTY_CYCLE = 254; // 2 to 11th
    static const int HALF_DUTY_CYCLE = MAX_DUTY_CYCLE / 2;
    static constexpr float START_DUTY_CYCLE = MAX_DUTY_CYCLE * 0.60;

private:
    int previous_pwm_{0};
    int pwm_pin_;
    int pwm_channel_;
    int motor_pinA_;
    int motor_pinB_;
};

#endif
