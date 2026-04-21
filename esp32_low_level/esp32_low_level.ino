#include <WiFi.h>
#include "esp_task_wdt.h"
#include "pinout.h"
#include "parameters.h"




// Wifi
#define SSID "robotx"
#define PASSWORD "78914040"

#define CREATE_ACCESS 0
#define MY_SSID "my_robotx"
#define MY_PASSWORD "78914040"


uint32_t server_timer = 0;
uint32_t odometry_timer = 0;    
uint32_t servo_timer = 0;   // timer for smooth moving

uint32_t tps = 0; // ticks per second
uint32_t tps_counter = 0; // ticks per second
uint32_t tps_timer = 0;


void setup() {
    Serial.begin(115200);

    SetupWheels();
    SetupServo();
    SetupLift();
    // CalibrateLift();

#if CREATE_ACCESS
    // Creating access point
    LogInfo("Creating acces point \"%s\"", MY_SSID);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(MY_SSID, MY_PASSWORD);
    delay(200);
    
    // Get ip of access point
    LogInfo("Access point created!");
    LogInfo("IP: %s", WiFi.softAPIP().toString().c_str());
#else
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    LogInfo("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print('.');
    }
    Serial.print("\n");
    
    LogInfo("Wifi connected!");
    LogInfo("IP: %s", WiFi.localIP().toString().c_str());
#endif

    SetupSocketServer();


    // Запуск задачи лидара на ядре 0
    LogDebug("Starting lidar task\n");
    xTaskCreatePinnedToCore(LidarTask, "LidarTask", 8192, nullptr, 2, nullptr, 0);
    
    tps_timer = millis();
}


void loop() {
    uint32_t current_time = millis();

    if ( current_time - server_timer > SERVER_DELAY ) {
        HandleAPIServer();
        server_timer = current_time;
    }

    WheelsTick();
    LiftTick();
    // ServoPosControl();

    if ( current_time - odometry_timer > ODOMETRY_DELAY ) {
        ComputeOdometry();
        odometry_timer = current_time;
    }

    tps_counter++;

    if ( current_time - tps_timer >= 1000 ) {
        tps = tps_counter;
        tps_counter = 0;
        tps_timer = millis();
        LogDebug("TPS: %lu", tps);
    }
}





