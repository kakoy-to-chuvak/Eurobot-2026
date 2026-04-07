#include <WiFiClient.h>
#include <WiFiServer.h>

#include "pinout.h"
#include "parameters.h"


uint8_t send_buffer[64];
int buffer_counter;

#define NEW_MSG(event) do{  send_buffer[2] = (event); \
                            buffer_counter=3;   }while(0)

#define SET_SEND_DATA(type, var) do{    *(type*)(send_buffer + buffer_counter) = (var); \
                                        buffer_counter += sizeof(type);    }while(0)

#define SEND_MSG(client) do{    *(uint16_t*)send_buffer = buffer_counter - 2;    \
                                (client).write(send_buffer, buffer_counter);  \
                                buffer_counter = 0;    }while(0)


// Server events
#define SERVER_GET_ALL 10
#define SERVER_GET_MOTORS_SPEED 11
#define SERVER_GET_LIFT_HEIGH   12
#define SERVER_GET_SERVO_STATE  13
#define SERVER_GET_ODOMETRY     14

#define SERVER_ANSWER_GET_ALL 40
#define SERVER_ANSWER_GET_MOTORS_SPEED 41
#define SERVER_ANSWER_GET_LIFT_HEIGH   42
#define SERVER_ANSWER_GET_SERVO_STATE  43
#define SERVER_ANSWER_GET_ODOMETRY     44

#define SERVER_SET_MOTORS_SPEED 71
#define SERVER_SET_LIFT_HEIGH   72
#define SERVER_SET_SERVO_STATE  73
#define SERVER_SET_ODOMETRY     74

#define SERVER_SEND_LIDAR 110


struct MyClient {
    WiFiClient client;

    int data_size;

    uint32_t connect_time;
    uint32_t request_time;

    struct MyClient *next;
    struct MyClient *prev;
};



WiFiServer server(SERVER_PORT);
MyClient *clients = NULL;
int now_id = 0;



void DeleteClient(struct MyClient *_Client) {
    if ( _Client->client.connected() ) {
        _Client->client.stop();
    }

    if ( _Client->next ) {
        _Client->next->prev = _Client->prev;
    }

    if ( _Client->prev ) {
        _Client->prev->next = _Client->next;
    } else {
        clients = _Client->next;
    }

    free(_Client);
}
 


inline void SetupServer() {
    LogInfo("Starting server on port %i\n", SERVER_PORT);
    server.begin();
    LogInfo("Server started\n");
}



void HandleServer() {
    while ( server.hasClient() ) {
        WiFiClient new_sock = server.accept();
        LogInfo("New client! IP: %s\n", new_sock.localIP().toString().c_str());

        struct MyClient *new_client = (struct MyClient*)calloc(1, sizeof(struct MyClient));

        new_client->client = new_sock;
        new_client->connect_time = millis();

        new_client->next = clients;
        clients = new_client;
    }

    MyClient *now = clients;

    while ( now ) {
        HandleClient(now);

        now = now->next;
    }
}





void HandleClient(struct MyClient *_Client) {
    WiFiClient client = _Client->client;

    if ( !client.connected() ) {
        LogInfo("Client disconnected. IP: %s\n", client.localIP().toString().c_str());
        DeleteClient(_Client);
        return;
    }

    if ( _Client->data_size == 0 ) {
        if ( client.available() < 2 ) {
            return;
        }

        client.read((uint8_t*)&_Client->data_size, 2);
        _Client->request_time = millis();

        LogTrace("New data (size: %i)\n", _Client->data_size);
    }

    if ( client.available() < _Client->data_size ) {
        if ( millis() - _Client->request_time > MAX_REQUEST_TIMEOUT ) {
            LogInfo("Too big request time, deleting client. IP: %s\n", client.localIP().toString().c_str());
            DeleteClient(_Client);
        }

        return;
    }

    uint8_t buffer[64];

    client.read(buffer, _Client->data_size);
    
    _Client->data_size--;
    HandleData(buffer[0], buffer+1, _Client);
    _Client->data_size = 0;
}   



void HandleData(uint8_t event, uint8_t *data, MyClient *_Client) {
    LogTrace("Handle data. EventL %i", event);
    
    switch (event) {
        case SERVER_SET_MOTORS_SPEED:
            if ( _Client->data_size < 2 * sizeof(float)) {
                break;
            }
            WheelsSetSpeed(((float*)data)[0], ((float*)data)[1]);
            break;

        case SERVER_SET_LIFT_HEIGH:
            if ( _Client->data_size < sizeof(float)) {
                break;
            }
            LiftSetTarget(*(float*)data);
            break;

        case SERVER_SET_SERVO_STATE:
            if ( _Client->data_size < 4 ) {
                break;
            }
            ServoSetTarget(data);
            break;

        case SERVER_SET_ODOMETRY:
            if ( _Client->data_size < 3 * sizeof(float)) {
                break;
            }
            SetOdometry(((float*)data)[0], ((float*)data)[1], ((float*)data)[2]);
            break;

        case SERVER_GET_MOTORS_SPEED:
            NEW_MSG(SERVER_ANSWER_GET_MOTORS_SPEED);
            SET_SEND_DATA(float, wheel_speed_linear);
            SET_SEND_DATA(float, wheel_speed_angular);
            SEND_MSG(_Client->client);
            break;

        case SERVER_GET_LIFT_HEIGH:
            NEW_MSG(SERVER_ANSWER_GET_LIFT_HEIGH);
            SET_SEND_DATA(float, LiftGetHeight());
            SEND_MSG(_Client->client);
            break;

        case SERVER_GET_SERVO_STATE:
            NEW_MSG(SERVER_ANSWER_GET_SERVO_STATE);
            SET_SEND_DATA(uint32_t, *(uint32_t*)ServoGetCurrentPos());
            SEND_MSG(_Client->client);
            break;

        case SERVER_GET_ODOMETRY:
            NEW_MSG(SERVER_ANSWER_GET_ODOMETRY);
            SET_SEND_DATA(float, theta);
            SET_SEND_DATA(float, xPos);
            SET_SEND_DATA(float, yPos);
            SEND_MSG(_Client->client);
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

            SEND_MSG(_Client->client);
            break;
    }
}

void ServerSendLidar(uint8_t *buffer, int length) {
    struct MyClient *now = clients;

    uint8_t send_buf[3];
    *(uint16_t*)send_buf = length + 1;
    send_buf[2] = SERVER_SEND_LIDAR;

    while ( now ) {
        LogTrace("Sending lidar to %s", now->client.localIP().toString().c_str());
        now->client.write(send_buf, 3);
        now->client.write(buffer, length);
    }
}











