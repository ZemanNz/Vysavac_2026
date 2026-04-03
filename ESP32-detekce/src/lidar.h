#pragma once
#include <Arduino.h>

#define LIDAR_RX 13
#define LIDAR_TX 10 // TX pin pro Serial2, není přímo LiDARem využíván, ale je potřeba pro .begin()

const int PACKET_SIZE = 47;
uint8_t packet[PACKET_SIZE];
int packetIndex = 0;

void processPacket() {
    // Start a koncové úhly ve formátu 0.01 stupně
    uint16_t startAngleX100 = packet[4] | (packet[5] << 8);
    uint16_t endAngleX100 = packet[42] | (packet[43] << 8);
    
    float startAngle = startAngleX100 / 100.0f;
    float endAngle = endAngleX100 / 100.0f;
    
    // Výpočet kroku (úhlového rozestupu) mezi 12 body v paketu
    float step;
    if (endAngle < startAngle) {
        // Kladný přechod přes 360 stupňů
        step = (endAngle + 360.0f - startAngle) / 11.0f;
    } else {
        step = (endAngle - startAngle) / 11.0f;
    }
    
    for (int i = 0; i < 12; i++) {
        int baseIndex = 6 + i * 3;
        uint16_t distance = packet[baseIndex] | (packet[baseIndex + 1] << 8);
        
        // Výpočet úhlu pro daný bod
        float angle = startAngle + step * i;
        
        // Posunutí zorného pole o 90 stupňů dopředu/dozadu
        angle -= 90.0f;
        
        // Ošetření přetečení a podtečení přes 360
        while (angle >= 360.0f) {
            angle -= 360.0f;
        }
        while (angle < 0.0f) {
            angle += 360.0f;
        }
        
        // Filtrace předních 180 stupňů okousaná o dalších 3 stupně tj. celkově 13 stupňů z obou stran (tj. 13° - 167°)
        // Tím se fyzicky odstřihnou překážky moc vlevo a moc vpravo z obrazu.
        if (angle >= 13.0f && angle <= 167.0f) {
            // Rychlé binární odesílání: 6 bajtů = [0xAA, 0x55, angleL, angleH, distL, distH]
            uint8_t buf[6];
            buf[0] = 0xAA;
            buf[1] = 0x55;
            uint16_t angleInt = (uint16_t)(angle * 100.0f);
            buf[2] = angleInt & 0xFF;
            buf[3] = (angleInt >> 8) & 0xFF;
            buf[4] = distance & 0xFF;
            buf[5] = (distance >> 8) & 0xFF;
            Serial.write(buf, 6);
        }
    }
}

void init_lidar() {
    // Inicializace sériové linky pro komunikaci s PC
    // POZOR: Změnil jsi baud rate na 921600. Tuto rychlost musel PC monitor podporovat!
    Serial.begin(921600);
    
    // Inicializace UART2 pro komunikaci s LiDARem (230400 baud, 8N1)
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void loop_lidar() {
    while (Serial2.available()) {
        uint8_t c = Serial2.read();

        if (packetIndex == 0) {
            // Hledání hlavičky 0x54
            if (c == 0x54) {
                packet[packetIndex++] = c;
            }
        } else if (packetIndex == 1) {
            // Validace délky (12 bodů = 0x0C) a verze
            if (c == 0x2C) {
                packet[packetIndex++] = c;
            } else if (c == 0x54) {
                // Může to být další hlavička, zkusíme zachovat
                packetIndex = 1;
            } else {
                // Špatný formát, reset
                packetIndex = 0;
            }
        } else {
            // Načítání zbylých dat
            packet[packetIndex++] = c;
            if (packetIndex == PACKET_SIZE) {
                processPacket();
                packetIndex = 0;
            }
        }
    }
}
