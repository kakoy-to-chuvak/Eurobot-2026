#include "pinout.h"
#include "parameters.h"



float theta = 0.0;
float xPos = 0.0;
float yPos = 0.0;

int32_t prev_current_l = 0;
int32_t prev_current_r = 0;

void SetOdometry(float _Xpos, float _Ypos, float _Theta) {
    xPos = _Xpos;
    yPos = _Ypos;
    theta = _Theta;
}   



void ComputeOdometry() {
    LogTrace2("Compute odometry");

    int32_t current_l = stepper_l.getCurrent();
    int32_t current_r = stepper_r.getCurrent();

    float delta_l = ( current_l - prev_current_l ) / WHEEL_STEPS_PER_M;
    float delta_r = ( current_r - prev_current_r ) / WHEEL_STEPS_PER_M;
    
    float delta_s = ( delta_l + delta_r ) / 2.0;
    float delta_theta = ( delta_r - delta_l ) / WHEEL_BASE;

    xPos += delta_s * cos(theta + delta_theta / 2.0);
    yPos += delta_s * sin(theta + delta_theta / 2.0);
    theta += delta_theta;

    // LogInfo("%f %f %f %f %f %f %f", delta_l, delta_r, delta_s, delta_theta, theta, xPos, yPos);

    prev_current_l = current_l;
    prev_current_r = current_r;
}