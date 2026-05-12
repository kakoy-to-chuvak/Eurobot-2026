// Disable acceleration for better control
#define GS_NO_ACCEL
#include <GyverStepper2.h>
#undef GS_NO_ACCEL

#include "pinout.h"
#include "parameters.h"


GStepper2<STEPPER2WIRE> stepper_l(WHEEL_STEPS_PER_REV, WHEEL_L_STP, WHEEL_L_DIR);    // left motor
GStepper2<STEPPER2WIRE> stepper_r(WHEEL_STEPS_PER_REV, WHEEL_R_STP, WHEEL_R_DIR);    // right motor

// Глобальные переменные скорости
int32_t wheel_l_speed, wheel_r_speed;
extern float wheel_speed_linear, wheel_speed_linear;
extern bool global_collide;




void SetupWheels() {
    LogDebug("Setup wheels");

    pinMode(WHEEL_L_ENA, OUTPUT);
    digitalWrite(WHEEL_L_ENA, 1);

    pinMode(WHEEL_R_ENA, OUTPUT);
    digitalWrite(WHEEL_R_ENA, 1);

    // НЕ УБИРАТЬ (int32_t) ИЗ-ЗА ОСОБЕНОСТИ БИБЛИОТЕКИ!!!!!
    stepper_l.setSpeed((int32_t)0);
    stepper_r.setSpeed((int32_t)0);

    stepper_l.reverse(1);
}


void WheelsSetSpeed(float _Linear, float _Angular) {

    wheel_speed_linear = _Linear;
    wheel_speed_angular = _Angular;

    int linear_k = wheel_speed_linear * WHEEL_STEPS_PER_M;
    int angular_k = wheel_speed_angular * WHEEL_BASE * 0.5 * WHEEL_STEPS_PER_M;

    wheel_l_speed = linear_k - angular_k;
    wheel_r_speed = linear_k + angular_k;

    LogTrace("Wheels set speed | linear: %f | angular: %f | wheel_left: %i | wheel_right: %i", _Linear, _Angular, wheel_l_speed, wheel_r_speed);

    stepper_l.setSpeed(wheel_l_speed);
    stepper_r.setSpeed(wheel_r_speed);

    if ( wheel_l_speed ) {
        // Enable motor
        digitalWrite(WHEEL_L_ENA, 0);
    } else {
        // Disable motor
        digitalWrite(WHEEL_L_ENA, 1);
    }

    if ( wheel_r_speed ) {
        // Enable motor
        digitalWrite(WHEEL_R_ENA, 0);
    } else {
        // Disable motor
        digitalWrite(WHEEL_R_ENA, 1);
    }
}


void WheelsTick() {
    if ( !global_collide ) {
        stepper_l.tick();
        stepper_r.tick();
    }
}






