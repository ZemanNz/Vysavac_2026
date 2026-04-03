#pragma once
#include <Arduino.h>
#include <math.h>

#define LIDAR_RX 13
#define LIDAR_TX 10

const int PACKET_SIZE = 47;
uint8_t packet[PACKET_SIZE];
int packetIndex = 0;

// Absolutní pozice robota na 1x1m hřišti
int16_t rob_x = 500;
int16_t rob_y = 0;

// Počet automaticky zahazovaných největších naměřených chyb (odseknutí top čtverců)
const int NUM_OUTLIERS = 8;

// Polní buffery pro uložení surových mapových bodů v dané čtvrtině zdí
struct PBuf { int16_t x; int16_t y; };
PBuf pts_R[150]; int n_R = 0;
PBuf pts_F[150]; int n_F = 0;
PBuf pts_L[150]; int n_L = 0;

uint32_t last_scan_time = 0;


// --- ODESÍLACÍ FUNKCE PROTOKOLU (0xBB 0x55) ---
void send_packet_point(int16_t x, int16_t y) {
    uint8_t buf[9];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 0; buf[3] = 4;
    buf[4] = x & 0xFF;  buf[5] = (x >> 8) & 0xFF;
    buf[6] = y & 0xFF;  buf[7] = (y >> 8) & 0xFF;
    buf[8] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]; 
    Serial.write(buf, 9);
}

void send_packet_line(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    uint8_t buf[13];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 1; buf[3] = 8;
    buf[4] = x1 & 0xFF; buf[5] = (x1 >> 8) & 0xFF;
    buf[6] = y1 & 0xFF; buf[7] = (y1 >> 8) & 0xFF;
    buf[8] = x2 & 0xFF; buf[9] = (x2 >> 8) & 0xFF;
    buf[10] = y2 & 0xFF; buf[11] = (y2 >> 8) & 0xFF;
    buf[12] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8] + buf[9] + buf[10] + buf[11];
    Serial.write(buf, 13);
}

void send_packet_robot(int16_t x, int16_t y) {
    uint8_t buf[9];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 2; buf[3] = 4;
    buf[4] = x & 0xFF;  buf[5] = (x >> 8) & 0xFF;
    buf[6] = y & 0xFF;  buf[7] = (y >> 8) & 0xFF;
    buf[8] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7];
    Serial.write(buf, 9);
}

// --- POKROČILÁ DVOU-FÁZOVÁ REGRESE S ODSEKNUTÍM CHYB ---
bool robustFitLine(PBuf* pts, int n, float& m, float& b, bool is_X_on_Y) {
    if (n < NUM_OUTLIERS + 3) return false; // Není dostatek bodů pro výpočet a odstranění
    
    // Fáze 1: První hrubý nástřel Least Squares regrese
    float sX=0, sY=0, sXY=0, sS=0;
    for(int i=0; i<n; i++) {
        sX += pts[i].x; sY += pts[i].y;
        if(is_X_on_Y) { sXY += pts[i].x * pts[i].y; sS += pts[i].y * pts[i].y; }
        else { sXY += pts[i].x * pts[i].y; sS += pts[i].x * pts[i].x; }
    }
    float m_init, b_init;
    if (is_X_on_Y) {
        float denom = (n * sS) - (sY * sY) + 0.0001f;
        m_init = ((n * sXY) - (sY * sX)) / denom;
        b_init = (sX - m_init * sY) / n;
    } else {
        float denom = (n * sS) - (sX * sX) + 0.0001f;
        m_init = ((n * sXY) - (sX * sY)) / denom;
        b_init = (sY - m_init * sX) / n;
    }

    // Fáze 2: Odstranění specifikovaného počtu "největších čtverců" (odštěpků/chyb)
    bool excl[150] = {false};
    for(int k=0; k<NUM_OUTLIERS; k++) {
        float max_err = -1.0f; int max_i = -1;
        for(int i=0; i<n; i++) {
            if(excl[i]) continue;
            float err;
            if(is_X_on_Y) {
                float pX = m_init * pts[i].y + b_init;
                err = (pts[i].x - pX) * (pts[i].x - pX);
            } else {
                float pY = m_init * pts[i].x + b_init;
                err = (pts[i].y - pY) * (pts[i].y - pY);
            }
            if(err > max_err) { max_err = err; max_i = i; }
        }
        if(max_i >= 0) excl[max_i] = true;
    }

    // Fáze 3: Nový, zcela přesný výpočet bez oněch chyb
    int final_n = 0; sX=0; sY=0; sXY=0; sS=0;
    for(int i=0; i<n; i++) {
        if(!excl[i]) {
            final_n++;
            sX += pts[i].x; sY += pts[i].y;
            if(is_X_on_Y) { sXY+=pts[i].x*pts[i].y; sS+=pts[i].y*pts[i].y; }
            else { sXY+=pts[i].x*pts[i].y; sS+=pts[i].x*pts[i].x; }
        }
    }
    
    if (is_X_on_Y) {
        float denom = (final_n * sS) - (sY * sY) + 0.0001f;
        m = ((final_n * sXY) - (sY * sX)) / denom;
        b = (sX - m * sY) / final_n;
    } else {
        float denom = (final_n * sS) - (sX * sX) + 0.0001f;
        m = ((final_n * sXY) - (sX * sY)) / denom;
        b = (sY - m * sX) / final_n;
    }
    return true;
}


void processPacket() {
    uint16_t startAngleX100 = packet[4] | (packet[5] << 8);
    uint16_t endAngleX100 = packet[42] | (packet[43] << 8);
    
    float startAngle = startAngleX100 / 100.0f;
    float endAngle = endAngleX100 / 100.0f;
    
    float step;
    if (endAngle < startAngle) step = (endAngle + 360.0f - startAngle) / 11.0f;
    else step = (endAngle - startAngle) / 11.0f;
    
    for (int i = 0; i < 12; i++) {
        int baseIndex = 6 + i * 3;
        uint16_t distance = packet[baseIndex] | (packet[baseIndex + 1] << 8);
        float angle = startAngle + step * i;
        
        angle -= 90.0f;
        while (angle >= 360.0f) angle -= 360.0f;
        while (angle < 0.0f) angle += 360.0f;
        
        // Zorné pole sníženo na 160° okno
        if (angle >= 10.0f && angle <= 170.0f) {
            float rad = angle * (PI / 180.0f);
            
            float fx = (float)distance * cos(rad);
            float fy = (float)distance * sin(rad);
            int16_t x = round(fx);
            int16_t y = round(fy);
            
            send_packet_point(rob_x + x, rob_y + y);

            // 1. Zvýšený limit "odstínění" mrtvé sféry z 20cm rovnou na 30 cm = 300 mm!
            if (distance >= 300 && distance < 1500) {
                if (angle >= 10.0f && angle <= 60.0f) {         
                    if (n_R < 150) { pts_R[n_R].x = x; pts_R[n_R].y = y; n_R++; }
                } else if (angle > 60.0f && angle <= 120.0f) {   
                    if (n_F < 150) { pts_F[n_F].x = x; pts_F[n_F].y = y; n_F++; }
                } else if (angle > 120.0f && angle <= 170.0f) {  
                    if (n_L < 150) { pts_L[n_L].x = x; pts_L[n_L].y = y; n_L++; }
                }
            }
        }
    }
}

void init_lidar() {
    Serial.begin(921600);
    Serial2.setRxBufferSize(1024);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void loop_lidar() {
    while (Serial2.available()) {
        uint8_t c = Serial2.read();

        if (packetIndex == 0) {
            if (c == 0x54) packet[packetIndex++] = c;
        } else if (packetIndex == 1) {
            if (c == 0x2C) packet[packetIndex++] = c;
            else if (c == 0x54) packetIndex = 1;
            else packetIndex = 0;
        } else {
            packet[packetIndex++] = c;
            if (packetIndex == PACKET_SIZE) {
                processPacket();
                packetIndex = 0;
            }
        }
    }
    
    // Na konci každé rotace provedeme regresi bez chyb a nakreslíme výsledek odvozené absolutní mapy
    if (millis() - last_scan_time > 120) {
        
        float b_L = 0, m_L = 0;
        bool L_valid = robustFitLine(pts_L, n_L, m_L, b_L, true);

        float b_R = 0, m_R = 0;
        bool R_valid = robustFitLine(pts_R, n_R, m_R, b_R, true);

        float b_F = 0, m_F = 0;
        bool F_valid = robustFitLine(pts_F, n_F, m_F, b_F, false);

        // --- ODVOZENÍ POZICE ROBOTA ---
        if (L_valid && R_valid) {
            rob_x = (-b_L + (1000 - b_R)) / 2;
        } else if (L_valid) {
            rob_x = -b_L;
        } else if (R_valid) {
            rob_x = 1000 - b_R;
        }
        if (F_valid) rob_y = 1000 - b_F;
        
        send_packet_robot(rob_x, rob_y);

        // --- ODVOZENÍ DLOUHÝCH STĚN PŘES CELÉ PLÁTNO ---
        if (L_valid) {
            int16_t y_start = -500; int16_t x_start = rob_x + (int16_t)(m_L * y_start + b_L);
            int16_t y_end = 1500;   int16_t x_end = rob_x + (int16_t)(m_L * y_end + b_L);
            send_packet_line(x_start, rob_y + y_start, x_end, rob_y + y_end);
        }
        if (R_valid) {
            int16_t y_start = -500; int16_t x_start = rob_x + (int16_t)(m_R * y_start + b_R);
            int16_t y_end = 1500;   int16_t x_end = rob_x + (int16_t)(m_R * y_end + b_R);
            send_packet_line(x_start, rob_y + y_start, x_end, rob_y + y_end);
        }
        if (F_valid) {
            int16_t x_start = -500; int16_t y_start = rob_y + (int16_t)(m_F * x_start + b_F);
            int16_t x_end = 1500;   int16_t y_end = rob_y + (int16_t)(m_F * x_end + b_F);
            send_packet_line(rob_x + x_start, y_start, rob_x + x_end, y_end);
        }

        // Reset bufferů
        n_L = 0; n_R = 0; n_F = 0;
        last_scan_time = millis();
    }
}
