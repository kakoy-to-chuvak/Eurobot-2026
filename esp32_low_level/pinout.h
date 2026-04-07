#ifndef __PINOUT_H
#define __PINOUT_H



// Main motors
#define WHEEL_L_ENA 4
#define WHEEL_L_STP 19
#define WHEEL_L_DIR 18

#define WHEEL_R_ENA 5
#define WHEEL_R_STP 23
#define WHEEL_R_DIR 22

// Lift
#define LIFT_L_ENA 26
#define LIFT_L_DIR 13
#define LIFT_L_STP 12

#define LIFT_R_ENA 25
#define LIFT_R_DIR 14
#define LIFT_R_STP 27

// Servos
#define SERVO_0 32
#define SERVO_1 33
#define SERVO_2 15
#define SERVO_3 2
const int SERVO_PINS[4] = { SERVO_0, SERVO_1, SERVO_2, SERVO_3 };

// Lidar
#define LIDAR_RX_PIN 16    // lidar TX -> ESP RX
#define LIDAR_TX_PIN 17    // lidar RX (не обязательно использовать)   

// Other
#define STARTER 0
#define SIDE_SWITCH 2









#endif // __PINOUT_H