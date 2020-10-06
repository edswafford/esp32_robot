#include "Motor.h"

Controller::Controller(int pwm_channel, int pwm_pin, int motor_pinA, int motor_pinB) : pwm_pin_(pwm_pin),
                                                                                       pwm_channel_(pwm_channel),
                                                                                       motor_pinA_(motor_pinA),
                                                                                       motor_pinB_(motor_pinB)
{
    pinMode(pwm_pin_, OUTPUT);
    pinMode(motor_pinA_, OUTPUT);
    pinMode(motor_pinB_, OUTPUT);

    // configure LED PWM functionalitites
    ledcSetup(pwm_channel_, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(pwm_pin_, pwm_channel_);

    //ensure that the motor is in neutral state during bootup
    ledcWrite(pwm_channel_, 0);
}

void Controller::spin(int pwm)
{
    if (pwm > 0)
    {
        digitalWrite(motor_pinA_, LOW);
        digitalWrite(motor_pinB_, HIGH);
    }
    else if (pwm < 0)
    {

        digitalWrite(motor_pinA_, HIGH);
        digitalWrite(motor_pinB_, LOW);
    }
    
    ledcWrite(pwm_channel_,  abs(pwm));
}
