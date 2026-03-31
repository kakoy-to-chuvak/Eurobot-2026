#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#include "pinout.h"
#include "parameters.h"




// Server events
#define SERVER_GET_ALL 10
#define SERVER_GET_MOTORS_SPEED 11
#define SERVER_GET_LIFT_HEIGHt  12
#define SERVER_GET_SERVO_STATE  13
#define SERVER_GET_ODOMETRY     14

#define SERVER_SET_MOTORS_SPEED 61
#define SERVER_SET_LIFT_HEIGHt  62
#define SERVER_SET_SERVO_STATE  63
#define SERVER_SET_ODOMETRY     64

#define SERVER_SEND_LIDAR 110


typedef struct MyClient {
    WiFiClient client;

    bool connected;
    int id;
    int data_size;

    uint32_t connect_time;
    uint32_t request_time;

    struct MyClient *next;
    struct MyClient *prev;
} MyClient;



WiFiServer server(SERVER_PORT);
MyClient *clients = NULL;
int now_id = 0;



void DeleteClient(MyClient *_Client) {
    if ( _Client->client.connected ) {
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

    LogDebug("Setup server clients\n");
}



void HandleServer() {
    if ( server.hasClient() ) {
        LogInfo("New client! ID: %i\n", now_id);

        WiFiClient new = server.accept();

        MyClient *new_client = calloc(1, sizeof(MyClient));

        new_client->client = new;
        new_client->connect_time = millis();
        new_client->id = now_id;
        now_id++;

        new_client->next = clients;
        clients = new_client;
    }

    MyClient *now = clients;

    while ( now ) {
        HandleClient(now);

        now = now->next;
    }
}





void HandleClient(MyClient *_Client) {
    if ( !_Client->connected ) {
        return;
    }

    WiFiClient client = _Client->client;

    if ( !client.connected() ) {
        LogInfo("Deleting client. ID: %i\n", _Client->id);
        DeleteClient(_Client);
        return;
    }

    if ( _Client->data_size == 0 ) {
        if ( client.available() < 2 ) {
            return;
        }

        client.read(&_Client->data_size, 2);
        _Client->request_time = millis();

        LogTrace("New data (size: %i) from client %i\n", _Client->data_size, _Client->id);
    }

    if ( client.available() < _Client->data_size ) {
        if ( millis() - _Client.request_time > MAX_REQUEST_TIMEOUT ) {
            LogInfo("Deleting client. ID: %i\n", _Client->id);
            DeleteClient(_Client)
        }

        return;
    }

    uint8_t buffer[64];

    client.read(buffer, _Client->data_size);
    HandleData(buffer[0], buffer+1, _Client);
}   



void HandleData(uint8_t event, uint8_t *data, MyClient *_Client) {

}













