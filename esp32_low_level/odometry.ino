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

    theta += delta_theta;
    xPos += delta_s * cos(theta);
    yPos += delta_s * sin(theta);

    prev_current_l = current_l;
    prev_current_r = current_r;
}