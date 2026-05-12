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


// Таймеры / задержки
uint32_t server_timer = 0;
uint32_t odometry_timer = 0;    
uint32_t servo_timer = 0;   // timer for smooth moving
uint32_t sensors_timer = 0;

// Статистиа
uint32_t tps = 0; // ticks per second
uint32_t tps_counter = 0; // ticks per second
uint32_t tps_timer = 0;
uint32_t min_loop_time = 0;
uint32_t max_loop_time = 0;

// Одометрия
float theta = 0.0;
float xPos = 0.0;
float yPos = 0.0;
bool global_collide = false;

// Скорости
float wheel_speed_linear = 0;
float wheel_speed_angular = 0;


#define STARTER_IN  0
#define STARTER_OUT 1
bool starter_state = STARTER_IN;

#define SIDE_YELLOW 0
#define SIDE_BLUE   1
bool side = SIDE_YELLOW;

void setup() {
    uint32_t setup_start_time = millis();

    Serial.begin(115200);

    SetupWheels();
    SetupServo();
    SetupLift();
    // CalibrateLift();

    // Инициализируем кнопки и переключатели
    pinMode(STARTER, INPUT_PULLUP);
    pinMode(SIDE_SWITCH, INPUT_PULLUP);

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
    LogDebug("Setup ended is %llu ms", millis() - setup_start_time);
}


void loop() {
    uint32_t current_time = millis();

    if ( current_time - server_timer > SERVER_DELAY ) {
        HandleAPIServer();
        server_timer = current_time;
    }

    WheelsTick();
    LiftTick();

    if ( current_time - odometry_timer > ODOMETRY_DELAY ) {
        ComputeOdometry();
        odometry_timer = current_time;
    }

    if ( millis() - sensors_timer > SENSORS_DELAY ) {
        // check starter
        byte tmp_state = !digitalRead(STARTER);
        if ( tmp_state != starter_state ) {
            starter_state = tmp_state;
            ServerSendStart(starter_state);

            if ( starter_state == STARTER_IN ) {
                WheelsSetSpeed(0.0, 0.0);
            }
        }

        // check side
        tmp_state = !digitalRead(SIDE_SWITCH);
        if ( side != tmp_state ) {
            side = tmp_state;
            ServerSendSide(side);
        }

        sensors_timer = millis();
    }
    
    // statistics
    tps_counter++;
    uint32_t loop_time = millis() - current_time;
    if ( loop_time < min_loop_time ) {
        min_loop_time = loop_time;

    }
    if ( loop_time > max_loop_time ) {
        max_loop_time = loop_time;
    }

    if ( current_time - tps_timer >= 1000 ) {
        tps = tps_counter;
        tps_counter = 0;

        LogDebug("Statistics: | TPS: %lu | min loop time: %llu | max loop time: %llu |", tps, min_loop_time, max_loop_time);

        tps_timer = millis();
    }
}





