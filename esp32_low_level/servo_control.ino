#include <ESP32Servo.h>
#include "pinout.h"
#include "parameters.h"





Servo servos[4];
const uint8_t servosLimits[4][2] = {
    { 95,  5   },
    { 104, 14  },
    { 79,  169 },
    { 78,  168 },
};

uint8_t servosTargetPos[4]  = { servosLimits[0][1], servosLimits[1][1], servosLimits[2][1], servosLimits[3][1] };
uint8_t servosCurrentPos[4] = { servosLimits[0][1], servosLimits[1][1], servosLimits[2][1], servosLimits[3][1] };


inline void SetupServo() {
    LogDebug("Setup servo");

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(3);
    ESP32PWM::allocateTimer(4);

    for (int i = 0; i < 4; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PINS[i], 500, 2500);
        servos[i].write(servosCurrentPos[i]);
    }
}


inline void ServoPosControl() {
    for ( uint8_t i = 0 ; i < 4; i++ ) {
        if ( servosCurrentPos[i] < servosTargetPos[i] ) {
            servosCurrentPos[i]++;    
            servos[i].write(servosCurrentPos[i]);
        } else if ( servosCurrentPos[i] > servosTargetPos[i] ) {
            servosCurrentPos[i]--;
            servos[i].write(servosCurrentPos[i]);
        }
    }
}


inline void ServoSetTarget(uint8_t *_Buffer) {
    LogTrace("Set sevo target");
    
    for ( int i = 0 ; i < 4 ; i++ ) {
        // constrain value
        if ( _Buffer[i] > 90 ) {
            _Buffer[i] = 90;
        }

        servosTargetPos[i] = map(_Buffer[i], 0, 90, servosLimits[i][0], servosLimits[i][1]);
    }
};


inline uint8_t *ServoGetCurrentPos() {
    LogTrace("Get servo pos");

    static uint8_t buffer[4];

    for ( uint8_t i = 0 ; i < 4 ; i++ ) {
        buffer[i] = map(servosCurrentPos[i], servosLimits[i][0], servosLimits[i][1], 0, 90);
    }

    return buffer;
}
