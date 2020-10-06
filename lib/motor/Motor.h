#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Controller
{
public:
    Controller(int pwm_channel, int pwm_pin, int motor_pinA, int motor_pinB);
    void spin(int pwm);

    // Setting PWM properties
    static const int freq = 30000;
    static const int PWM_CHANNEL_1 = 0;
    static const int PWM_CHANNEL_2 = 1;
    static const int resolution = 11;
    static const int MAX_DUTY_CYCLE = 2048; // 2 to 11th
    static const int HALF_DUTY_CYCLE = MAX_DUTY_CYCLE / 2;

private:
    int pwm_pin_;
    int pwm_channel_;
    int motor_pinA_;
    int motor_pinB_;
};

#endif
