#include <ESP32Servo.h>
#include "pinout.h"
#include "parameters.h"




Servo servos[4];

const uint8_t servosLimits[4][2] = {
    { 95,  5   },
    { 104, 14  },
    { 75,  165 },
    { 75,  165 },
};



uint8_t servosCurrentPos[4] = { servosLimits[0][1], servosLimits[1][1], servosLimits[2][1], servosLimits[3][1] };


inline void SetupServo() {
    LogDebug("Setup servo");    

    for (int i = 0; i < 4; i++) {
        servos[i].attach(SERVO_PINS[i], 500, 2500);
        servos[i].write(servosLimits[i][1]);
    }
}


inline void ServoSetTarget(uint8_t *_Buffer) {
    LogTrace("Set servo target: %i %i %i %i", _Buffer[0], _Buffer[1], _Buffer[2], _Buffer[3]);
    
    for ( int i = 0 ; i < 4 ; i++ ) {
        // constrain value
        uint8_t value = constrain(_Buffer[i], 0, 90);
        servosCurrentPos[i] = map(value, 0, 90, servosLimits[i][0], servosLimits[i][1]);
        servos[i].write(servosCurrentPos[i]);
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
