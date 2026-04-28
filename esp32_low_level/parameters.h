#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#define _USE_MATH_DEFINES
#include <math.h>



// Server
#define SERVER_PORT 8080
#define LIDAR_PORT 8090



// Robot
#define WHEEL_R 0.0455     // m
#define WHEEL_BASE 0.2066407  // m
#define WHEEL_STEPS_PER_REV 800
constexpr float WHEEL_STEPS_PER_M = (float)WHEEL_STEPS_PER_REV / ( 2.0f * M_PI * WHEEL_R );

// Lift
#define LIFT_MIN_POSITION 0    // mm
#define LIFT_MAX_POSITION 190  // mm

#define LIFT_STEPS_PER_REV 800
#define LIFT_R_MM_PER_REV 16.05 
#define LIFT_L_MM_PER_REV 16.07

constexpr uint16_t LIFT_L_STEPS_PER_MM = LIFT_STEPS_PER_REV / LIFT_L_MM_PER_REV + 0.5f;
constexpr uint16_t LIFT_R_STEPS_PER_MM = LIFT_STEPS_PER_REV / LIFT_R_MM_PER_REV + 0.5f;


// Speed and delays
const unsigned long DELAY_PER_SERVO[4] = { 20, 20, 20, 20 };

#define SERVER_DELAY   5 // delay between calls
#define ODOMETRY_DELAY 50 // delay between 
#define SENSORS_DELAY  50








// ==== Debug ====
#define LOG_LEVEL 0

#if LOG_LEVEL <= 2
#   define LodError(fmt, ...) Serial.printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#else
#   define LodError(fmt, ...) 
#endif

#if LOG_LEVEL <= 1
#   define LogWarn(fmt, ...) Serial.printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#else
#   define LogWarn(fmt, ...) 
#endif

#if LOG_LEVEL <= 0
#   define LogInfo(fmt, ...) Serial.printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#else
#   define LogInfo(fmt, ...) 
#endif

#if LOG_LEVEL <= -1
#   define LogDebug(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
#   define LogDebug(fmt, ...) 
#endif

#if LOG_LEVEL <= -2
#   define LogTrace(fmt, ...) Serial.printf("[TRACE] " fmt "\n", ##__VA_ARGS__)
#else
#   define LogTrace(fmt, ...) 
#endif

#if LOG_LEVEL <= -3
#   define LogTrace2(fmt, ...) Serial.printf("[TRACE] " fmt "\n", ##__VA_ARGS__)
#else
#   define LogTrace2(fmt, ...) 
#endif

#endif