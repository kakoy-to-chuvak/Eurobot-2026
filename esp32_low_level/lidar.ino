#include <WiFiUdp.h>

#include "pinout.h"
#include "parameters.h"


#define LIDAR_BAUD 115200

static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
#define BODY_LEN  32            // байт в теле пакета лидара (8 точек по 4 байта)
#define INTENSITY_MIN 2    // минимальное значение интенсивности для учёта точки
#define MAX_SPREAD_DEG 20.0f
#define FRAME_LEN 20     // длина упакованных данных на каждую порцию (2 байта нач.угол, 2 байта кон.угол, 8*2 байта дистанции)
#define MAX_FRAMES 64    // макс. количество порций на один полный оборот (64*20 ≈ 1280 байт)

static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1;


// UDP server
#define UDP_PASSWORD "udp_password"
WiFiUDP udp_server;
IPAddress udp_client_ip;

void StartUdpServer() {

    // Starting udp server
    LogInfo("Starting UDP server on port %i", UDP_SERVER_PORT);
    udp_server.begin(UDP_SERVER_PORT);
    LogInfo("UDP server started");

    // Waiting for udp client
    LogInfo("Waiting for udp client");
    while ( !udp_server.parsePacket() ) {
        vTaskDelay(10);
    }

    LogInfo("UDP client connected");

    uint8_t data[32];
    // read data
    udp_server.read(data, 32);

    udp_client_ip = udp_server.remoteIP();
    LogInfo("UDP client registred! IP: %s", udp_client_ip.toString().c_str());
}

// ==== Helper lidar functions ====
inline float decodeAngle(uint16_t _Raw) {
    // Декодирует угол (двухбайтное значение) из формата LDS
    float a = (_Raw - 0xA000) / 64.0f;
    if (a < 0) 
        a += 360.0f;
    else if (a >= 360) 
        a -= 360.0f;
    return a;
}
bool readBytes(HardwareSerial &_Serial, uint8_t *_Dst, size_t _Len, uint32_t _Timeout = 300) {
    uint32_t t0 = millis();
    for ( size_t i = 0 ; i < _Len ; i++ ) {
        while ( !_Serial.available() ) {
            if ( millis() - t0 > _Timeout ) {
                return false;
            }
            vTaskDelay(1);
        }
        _Dst[i] = _Serial.read();
    }
    return true;
}
bool waitLidarHeader(HardwareSerial &_Serial) {
    uint8_t pos = 0;
    uint32_t t0 = millis();
    while (true) {
        if (_Serial.available()) {
            if (uint8_t(_Serial.read()) == HDR[pos]) {
                if (++pos == 4) return true;
            } else {
                pos = 0;
            }
        }
        if (millis() - t0 > 200) return false;
    }
}
inline uint16_t crc16(uint16_t _Crc, uint8_t _V) {
    _Crc ^= _V;
    for (uint8_t i = 0; i < 8; ++i) {
        _Crc = (_Crc & 1) ? (_Crc >> 1) ^ 0xA001 : (_Crc >> 1);
    }
    return _Crc;
}


// ==== Main lidar task ====
void LidarTask(void *_Param) {
    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    uint8_t body[BODY_LEN];

    StartUdpServer();

    
    while (true) {
        vTaskDelay(1);

        if ( !waitLidarHeader(Serial2) || !readBytes(Serial2, body, BODY_LEN) ) {
            continue;
        }
        
        // Распарсить порцию точек лидара
        float startDeg = decodeAngle(   body[2] | (body[3] << 8));
        uint8_t offset = 4;
        uint16_t dist[8];
        uint8_t quality[8];
        for (int i = 0; i < 8; ++i) {
            dist[i] = body[offset] | (body[offset + 1] << 8);
            quality[i] = body[offset + 2];
            offset += 3;
        }

        float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
        if (endDeg < startDeg) endDeg += 360.0f;
        if (endDeg - startDeg > MAX_SPREAD_DEG) {
            // Пропускаем пакет, если слишком большой разрыв (ошибка)
            continue;
        }

        // Упаковываем 20 байт в общий буфер скана
        uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
        uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
        *wr++ = s & 0xFF;
        *wr++ = s >> 8;
        *wr++ = e & 0xFF;
        *wr++ = e >> 8;
        for (int i = 0; i < 8; ++i) {
            uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
            *wr++ = d & 0xFF;
            *wr++ = d >> 8;
        }
        frameCount++;

        if (frameCount >= MAX_FRAMES) {
            frameCount = 0;
            wr = scanBuf;
            prevStartAngle = -1;    // сброс для корректного «перескока» угла
            continue;                         // переходим к следующему пакету
        }

        // Проверяем перескок через 0° (начало нового круга)
        if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
            size_t scanSize = frameCount * FRAME_LEN;
            // Вычисляем CRC для всего скана
            uint16_t crc = 0xFFFF;
            for (size_t i = 0; i < scanSize; ++i) {
                crc = crc16(crc, scanBuf[i]);
            }
            scanBuf[scanSize] = crc & 0xFF;
            scanBuf[scanSize + 1] = crc >> 8;
            
            // Отправляем по Socket
            udp_server.write(scanBuf, scanSize + 2);

            // Сбрасываем буфер для следующего оборота
            wr = scanBuf;
            frameCount = 0;
        }
        prevStartAngle = startDeg;
    }
}