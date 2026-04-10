#include <WiFiClient.h>
#include <WiFiServer.h>

#include "pinout.h"
#include "parameters.h"



uint8_t __send_buffer[64];
int __buffer_counter;

// macros for sending messages
#define NEW_MSG(event) do{  __send_buffer[2] = (event); \
                            __buffer_counter=3;   }while(0)

#define SET_SEND_DATA(type, var) do{    *(type*)(__send_buffer + __buffer_counter) = (var); \
                                        __buffer_counter += sizeof(type);    }while(0)

#define SEND_MSG() do{  *(uint16_t*)__send_buffer = __buffer_counter - 2;    \
                        socket_client.write(__send_buffer, __buffer_counter);  \
                        __buffer_counter = 0;   \
                        LogTrace("Send message");   }while(0)


// Server events
#define SERVER_GET_ALL 10
#define SERVER_GET_MOTORS_SPEED 11
#define SERVER_GET_LIFT_HEIGHT  12
#define SERVER_GET_SERVO_STATE  13
#define SERVER_GET_ODOMETRY     14

#define SERVER_ANSWER_GET_ALL 40
#define SERVER_ANSWER_GET_MOTORS_SPEED 41
#define SERVER_ANSWER_GET_LIFT_HEIGHT  42
#define SERVER_ANSWER_GET_SERVO_STATE  43
#define SERVER_ANSWER_GET_ODOMETRY     44


#define SERVER_SET_MOTORS_SPEED 71
#define SERVER_SET_LIFT_HEIGHT  72
#define SERVER_SET_SERVO_STATE  73
#define SERVER_SET_ODOMETRY     74

#define SERVER_SEND_LIDAR 110




WiFiServer socket_server(SERVER_PORT, 1);
WiFiClient socket_client;
bool socket_connected = 0;
int data_size;





inline void SetupSocketServer() {
    LogInfo("Starting socket server on port %i", SERVER_PORT);
    socket_server.begin();
    LogInfo("Socket server started");
}



void HandleAPIServer() {
    LogTrace("Handle API server");

    if ( !socket_client.connected() || !socket_connected ) {
        if ( socket_connected ) {
            socket_connected = 0;
            WheelsSetSpeed(0.0, 0.0);
            LogWarn("Socket client disconnected.");
        }
        
        if ( socket_server.hasClient() ) {
            socket_client = socket_server.accept();
            socket_connected = 1;
            LogInfo("New socket client connected! IP: %s", socket_client.localIP().toString().c_str());
        }
        
        return;
    }
    
    if ( data_size == 0 ) {
        if ( socket_client.available() < 2 ) {
            return;
        }

        socket_client.read((uint8_t*)&data_size, 2);

        LogTrace("New data (size: %i)", data_size);
    }

    if ( socket_client.available() < data_size ) {
        return;
    }

    uint8_t buffer[64];

    socket_client.read(buffer, data_size);
    LogTrace("Read data");
    
    data_size--;
    HandleData(buffer[0], buffer+1);
    data_size = 0;
}   



void HandleData(uint8_t event, uint8_t *data) {
    LogDebug("Handle data. Event %i", event);
    
    switch (event) {
        case SERVER_SET_MOTORS_SPEED:
            if ( data_size < 2 * sizeof(float)) {
                break;
            }
            WheelsSetSpeed(((float*)data)[0], ((float*)data)[1]);
            break;

        case SERVER_SET_LIFT_HEIGHT:
            if ( data_size < sizeof(uint16_t)) {
                break;
            }
            LiftSetTarget(*(uint16_t*)data);
            break;

        case SERVER_SET_SERVO_STATE:
            if ( data_size < 4 ) {
                break;
            }
            ServoSetTarget(data);
            break;

        case SERVER_SET_ODOMETRY:
            if ( data_size < 3 * sizeof(float)) {
                break;
            }
            SetOdometry(((float*)data)[0], ((float*)data)[1], ((float*)data)[2]);
            break;

        case SERVER_GET_MOTORS_SPEED:
            NEW_MSG(SERVER_ANSWER_GET_MOTORS_SPEED);
            SET_SEND_DATA(float, wheel_speed_linear);
            SET_SEND_DATA(float, wheel_speed_angular);
            SEND_MSG();
            break;

        case SERVER_GET_LIFT_HEIGHT:
            NEW_MSG(SERVER_ANSWER_GET_LIFT_HEIGHT);
            SET_SEND_DATA(uint16_t, LiftGetHeight());
            SEND_MSG();
            break;

        case SERVER_GET_SERVO_STATE:
            NEW_MSG(SERVER_ANSWER_GET_SERVO_STATE);
            SET_SEND_DATA(uint32_t, *(uint32_t*)ServoGetCurrentPos());
            SEND_MSG();
            break;

        case SERVER_GET_ODOMETRY:
            NEW_MSG(SERVER_ANSWER_GET_ODOMETRY);
            SET_SEND_DATA(float, theta);
            SET_SEND_DATA(float, xPos);
            SET_SEND_DATA(float, yPos);
            SEND_MSG();
            break;

        case SERVER_GET_ALL:
            NEW_MSG(SERVER_ANSWER_GET_ALL);
            // speed
            SET_SEND_DATA(float, wheel_speed_linear);
            SET_SEND_DATA(float, wheel_speed_angular);
            // lift height
            SET_SEND_DATA(float, LiftGetHeight());
            // servo state
            SET_SEND_DATA(uint32_t, *(uint32_t*)ServoGetCurrentPos());
            // odometry
            SET_SEND_DATA(float, theta);
            SET_SEND_DATA(float, xPos);
            SET_SEND_DATA(float, yPos);

            SEND_MSG();
            break;
    }
}











