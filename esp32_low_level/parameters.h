#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#define _USE_MATH_DEFINES
#include <math.h>



// Server
#define MAX_CLIENTS 8
#define SERVER_PORT 8080
#define LIDAR_SEND_DELAY 300
#define MAX_REQUEST_TIMEOUT 300



// Robot
#define WHEEL_R 0.0455     // m
#define WHEEL_BASE 0.2075  // m
#define WHEEL_STEPS_PER_REV 800
constexpr float WHEEL_STEPS_PER_M = (float)WHEEL_STEPS_PER_REV / ( 2.0f * M_PI * WHEEL_R );

// Lift
#define LIFT_MIN_POSITION 0.0    // mm
#define LIFT_MAX_POSITION 200.0  // mm

#define LIFT_STEPS_PER_REV 800
#define LIFT_R_MM_PER_REV 16.05 
#define LIFT_L_MM_PER_REV 16.07

constexpr float LIFT_L_STEPS_PER_MM = LIFT_STEPS_PER_REV / LIFT_L_MM_PER_REV;
constexpr float LIFT_R_STEPS_PER_MM = LIFT_STEPS_PER_REV / LIFT_R_MM_PER_REV;


// Speed and timers
#define SERVO_SPEED 100  // degree per second
constexpr uint16_t SERVO_DELAY = 1000 / SERVO_SPEED; // delay between calls

#define SERVER_DELAY   50 // delay between calls
#define ODOMETRY_DELAY 10 // delay between calls




// ==== Debug ====
#define LOGGING -1

#if LOGGING <= 0
#   define LogInfo(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#   define LogInfo(fmt, ...) 
#endif

#if LOGGING <= -1
#   define LogDebug(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#   define LogDebug(fmt, ...) 
#endif

#if LOGGING <= -2
#   define LogTrace(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#   define LogTrace(fmt, ...) 
#endif


#endif