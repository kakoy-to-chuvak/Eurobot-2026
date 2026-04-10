// Disable acceleration for better control
#define GS_NO_ACCEL
#include <GyverStepper2.h>
#undef GS_NO_ACCEL

#include "pinout.h"
#include "parameters.h"


GStepper2<STEPPER2WIRE> lift_l(LIFT_STEPS_PER_REV, LIFT_L_STP, LIFT_L_DIR);    // left lift motor
GStepper2<STEPPER2WIRE> lift_r(LIFT_STEPS_PER_REV, LIFT_R_STP, LIFT_R_DIR);    // right lift motor


uint16_t lift_target_height = 0;


inline void SetupLift() {
    LogDebug("Setup lift");

    pinMode(LIFT_L_ENA, OUTPUT);
    pinMode(LIFT_R_ENA, OUTPUT);

    lift_l.setMaxSpeed(8000);
    lift_r.setMaxSpeed(8000);

    lift_l.reverse(1);
    lift_r.reverse(1);

    // Enable motors
    digitalWrite(LIFT_L_ENA, 0);
    digitalWrite(LIFT_R_ENA, 0);
}

inline void LiftSetTarget(uint16_t _Height_mm) {
    LogTrace("Set lift target");
    // Ограничиваем позицию пределами рабочей зоны
    lift_target_height = constrain(_Height_mm, LIFT_MIN_POSITION, LIFT_MAX_POSITION);
    
    lift_l.setTarget(lift_target_height * LIFT_L_STEPS_PER_MM, ABSOLUTE);
    lift_r.setTarget(lift_target_height * LIFT_R_STEPS_PER_MM, ABSOLUTE);
}

uint16_t LiftGetHeight() {
    LogTrace("Get lift height");
    return ( lift_l.getCurrent() / LIFT_L_STEPS_PER_MM + lift_r.getCurrent() / LIFT_R_STEPS_PER_MM ) / 2;
}

inline void LiftTick() {
    lift_l.tick();
    lift_r.tick();

    if ( lift_l.getCurrent() < int(2.0 * LIFT_L_STEPS_PER_MM) ) {
        // Disable motors
        digitalWrite(LIFT_L_ENA, 1);
    } else {
        // Enable motors
        digitalWrite(LIFT_L_ENA, 0);
    }

    if ( lift_r.getCurrent() < int(2.0 * LIFT_R_STEPS_PER_MM) ) {
        // Disable motors
        digitalWrite(LIFT_R_ENA, 1);
    } else {
        // Enable motors
        digitalWrite(LIFT_R_ENA, 0);
    }
}
