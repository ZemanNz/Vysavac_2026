#pragma once
#include <Arduino.h>
#include <math.h>

#define LIDAR_RX 13
#define LIDAR_TX 10

const int PACKET_SIZE = 47;
uint8_t packet[PACKET_SIZE];
int packetIndex = 0;

// Robot staticky uprostřed hřiště pro snazší debug a nezavislost mapování zdi
int16_t rob_x = 500;
int16_t rob_y = 500; 

// Globální buffer pro VŠECHNY viditelné body (žádné omezující sektory L/R/F)
struct PBuf { int16_t x; int16_t y; bool used; };
PBuf pts[500];
int n_pts = 0;

uint32_t last_scan_time = 0;

// --- ODESÍLACÍ FUNKCE PROTOKOLU ---
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

void send_packet_opponent(int16_t x, int16_t y) {
    uint8_t buf[9];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 3; buf[3] = 4;
    buf[4] = x & 0xFF;  buf[5] = (x >> 8) & 0xFF;
    buf[6] = y & 0xFF;  buf[7] = (y >> 8) & 0xFF;
    buf[8] = buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7];
    Serial.write(buf, 9);
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
        
        // Snížení "šíleně za záda" zdi – povoleno to co robot orientačně mapuje kolem sebe
        if (angle >= 5.0f && angle <= 175.0f) {
            float rad = angle * (PI / 180.0f);
            
            float fx = (float)distance * cos(rad);
            float fy = (float)distance * sin(rad);
            int16_t x = round(fx);
            int16_t y = round(fy);
            
            // Limitace robotova těla a okrajů pokoje (30cm až 1.5m)
            if (distance >= 300 && distance < 1500) {
                // Přímé streamování zelených teček na PC
                send_packet_point(rob_x + x, rob_y + y);

                // Hodit nekompromisně všechno na jednu globální pánev
                if (n_pts < 500) {
                    pts[n_pts].x = x;
                    pts[n_pts].y = y;
                    pts[n_pts].used = false; // Ještě nezařazeno do žádné stěny
                    n_pts++;
                }
            }
        }
    }
}

void init_lidar() {
    Serial.begin(921600);
    randomSeed(analogRead(34)); // Pevné seedování Ransacu
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
    
    // Zpracování dat 8x za vteřinu
    if (millis() - last_scan_time > 120) {
        
        // Pevná, zabetonovaná pozice přesně uprostřed hřiště
        rob_x = 500; 
        rob_y = 500;
        send_packet_robot(rob_x, rob_y);
        
        int points_left = n_pts;
        
        // GLOBÁLNÍ ITERATIVNÍ RANSAC: Najde až 4 nejdůležitější dlouhé přímky nehledě na jejich kolmost
        for (int line_idx = 0; line_idx < 4; line_idx++) {
            if (points_left < 15) break; 
            
            int best_inliers = 0;
            float best_nx = 0, best_ny = 0, best_c = 0;
            
            // Nahodíme přes celou hromadu 40 náhodných hádankových přímek
            for (int iter = 0; iter < 40; iter++) {
                int retries = 0;
                int i1 = random(0, n_pts);
                while (pts[i1].used && retries < 15) { i1 = random(0, n_pts); retries++; }
                if (pts[i1].used) continue;
                
                int i2 = random(0, n_pts);
                retries = 0;
                while ((pts[i2].used || i1 == i2) && retries < 15) { i2 = random(0, n_pts); retries++; }
                if (pts[i2].used || i1 == i2) continue;
                
                float p1x = pts[i1].x, p1y = pts[i1].y;
                float p2x = pts[i2].x, p2y = pts[i2].y;
                float dx = p2x - p1x;
                float dy = p2y - p1y;
                float len = sqrt(dx*dx + dy*dy);
                if (len < 10.0f) continue; // Stejné vzdálené body nevyváří přímku
                
                // Určíme normálu úsečky (Rovnice Ax + By + C = 0 dokáže otáčet 360 stupni svobodně zeď po zdi)
                float nx = -dy / len;
                float ny = dx / len;
                float c = -(nx * p1x + ny * p1y); 
                
                int inliers = 0;
                for (int i = 0; i < n_pts; i++) {
                    if (pts[i].used) continue;
                    // Výpočet čisté kolmé dálky k této natržené niti (Vzdálenost bodu od přímky v rovině)
                    float dist = abs(nx * pts[i].x + ny * pts[i].y + c);
                    if (dist < 40.0f) inliers++;
                }
                
                // Uložíme jen tu "nejtlustší logickou" strunu protínající stěnu
                if (inliers > best_inliers) {
                    best_inliers = inliers; best_nx = nx; best_ny = ny; best_c = c;
                }
            }
            
            if (best_inliers < 25) break; 
            
            // Nyní spočítáme čistou statistiku nalezené stěny (Délka a orientace vektoru)
            float span_min = 999999, span_max = -999999;
            float avg_x = 0, avg_y = 0;
            int final_inliers = 0;
            
            // Druhá prochodní iterace přes Inliere – Teď stěnu trvale z pole "zkonzumujeme"
            for (int i = 0; i < n_pts; i++) {
                if (pts[i].used) continue;
                float dist = abs(best_nx * pts[i].x + best_ny * pts[i].y + best_c);
                if (dist < 40.0f) {
                    pts[i].used = true; // Vymazáno ze světa! Zítřejší stěna 2,3,4 už je neuvidí!
                    points_left--;
                    final_inliers++;
                    avg_x += pts[i].x;
                    avg_y += pts[i].y;
                    
                    // Po Strune odhadneme délku shluku (Aby křesla a lživá zrcadlení rohu tvořící tečny shořely)
                    float proj = (-best_ny) * pts[i].x + (best_nx) * pts[i].y;
                    if (proj < span_min) span_min = proj;
                    if (proj > span_max) span_max = proj;
                }
            }
            
            if (final_inliers < 25) continue; // Přísná ochrana (hustota minimálně 25 naměřených zásahů na stěnu)
            
            // Je struna dlouhá alespoň ze 35 cm nametení? Jinak je to prostě odpad smítka v rohu.
            if ((span_max - span_min) < 350.0f) {
                continue; // Odpad (botu/křeslo/vysavač) jsme díky used=true už teď smazali tak či tak, ale PC se o něm nedozví! Nestav se překážkám zdi!
            }
            
            // Těžištěm středu vytáhneme nekonečnou Fialovou přes Python obálku
            avg_x /= final_inliers;
            avg_y /= final_inliers;
            
            float vx = -best_ny; float vy = best_nx;
            int16_t x1 = rob_x + (int16_t)(avg_x - 1500 * vx);
            int16_t y1 = rob_y + (int16_t)(avg_y - 1500 * vy);
            int16_t x2 = rob_x + (int16_t)(avg_x + 1500 * vx);
            int16_t y2 = rob_y + (int16_t)(avg_y + 1500 * vy);
            
            send_packet_line(x1, y1, x2, y2);
        }

        // --- DETEKCE SOUPEŘE ZE ZBYTKU BODŮ ---
        float opp_x = 0, opp_y = 0;
        int opp_pts = 0;
        
        for (int i = 0; i < n_pts; i++) {
            if (!pts[i].used) {
                // Slitek veškerého zbylého materiálu
                opp_x += pts[i].x;
                opp_y += pts[i].y;
                opp_pts++;
            }
        }
        
        if (opp_pts >= 4) { // Je to shluk alespoň 4 bodů (Ne ojedinělý Lidar šum)
            opp_x /= opp_pts;
            opp_y /= opp_pts;
            send_packet_opponent(rob_x + (int16_t)opp_x, rob_y + (int16_t)opp_y);
        }

        // Vyčistit lokální pole (nová otáčka zčuchne čistý papír dat)
        n_pts = 0;
        last_scan_time = millis();
    }
}
