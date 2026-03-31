#include <WiFi.h>
#include "esp_task_wdt.h"
#include "pinout.h"
#include "parameters.h"



// Wifi
#define SSID "robotx"
#define PASSWORD "78910442"



uint32_t server_timer = 0;
uint32_t odometry_timer = 0;    
uint32_t servo_timer = 0;   // timer for smooth moving

uint32_t tps = 0; // ticks per second
uint32_t tps_counter = 0; // ticks per second
uint32_t tps_timer = 0;


void setup() {
    Serial.begin(115200);

    LogDebug("Setup main motors\n");
    SetupWheels();

    LogDebug("Setup lift\n");
    SetupLift();

    LogDebug("Setup servo\n");
    SetupServo();

    // Creating access point
    LogInfo("Creating acces point \"%s\"\n", SSID);
    WiFi.softAP(SSID, PASSWORD);
    delay(200);
    
    // Get ip of access point
    IPAddress Wifi_IP = WiFi.softAPIP(); 
    Serial.print("Created! IP: ");
    Serial.println(Wifi_IP);

    LogDebug("Starting server\n");
    StartServer();


    // Запуск задачи лидара на ядре 0
    LogDebug("Starting lidar task\n");
    xTaskCreatePinnedToCore(LidarTask, "LidarTask", 8192, nullptr, 2, nullptr, 0);
    
    tps_timer = millis();
}


void loop() {
    uint32_t current_time = millis();

    if ( current_time - server_timer > SERVER_DELAY ) {
        HandleServer();
        server_timer = current_time;
    }

    WheelsTick();
    LiftTick();
    ServoPosControl();

    if ( current_time - servo_timer > SERVO_DELAY ) {
        servo_timer = current_time;
    }

    if ( current_time - odometry_timer > ODOMETRY_DELAY ) {
        ComputeOdometry();
        odometry_timer = current_time;
    }

    tps_counter++;

    if ( current_time - tps_timer >= 1000 ) {
        tps = tps_counter;
        tps_counter = 0;
        tps_timer = millis();
    }
}







