#pragma once
#include <Arduino.h>
#include <math.h>

#define LIDAR_RX 13
#define LIDAR_TX 10 // TX pin for Serial2, not used but needed for begin()

#define MEM_LIFESPAN_FRAMES 500 // Životnost a doba fixace paměti hlavní stěny v počtu cyklů (20 = cca 2.5 sekundy)

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

void printStats() {
    Serial.print("Points in buffer: ");
    Serial.println(n_pts);
}

// --- PAMĚŤ DOMINANTNÍ STĚNY ---
float mem_nx = 0, mem_ny = 0;
int mem_consistent = 0;
int mem_miss = 0;
bool mem_locked = false;

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
        
        struct LineParams { float nx; float ny; float c; };
        LineParams walls[4];
        int num_walls = 0;
        
        // GLOBÁLNÍ ITERATIVNÍ RANSAC: Najde až 3 nejdůležitější dlouhé přímky (do 180st robot z principu víc zdí neuvidí)
        for (int line_idx = 0; line_idx < 3; line_idx++) {
            if (points_left < 30) break; 
            
            int best_inliers = 0;
            float best_score = 0.0f;
            float best_nx = 0, best_ny = 0, best_c = 0;
            
            // Nahodíme drastických 200 pokusů pro garantované objevení i tence zastoupených stěn bez přeskočení!
            for (int iter = 0; iter < 200; iter++) {
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
                
                // Určíme normálu úsečky
                float nx = -dy / len;
                float ny = dx / len;
                
                // PAMĚŤOVÝ ZÁMEK: Pokud jsme zamčeni na dominantu, natvrdo sledujeme pouze tu!
                if (line_idx == 0 && mem_locked) {
                    float dot = abs(nx * mem_nx + ny * mem_ny);
                    if (dot < 0.85f) continue; // Zaručí neprůstřelné držení žluté stěny, ignorující náhlé fluktuace skóre jinde v poli
                }
                
                // --- ORTOGONÁLNÍ KOREKCE (Geometrie arény) ---
                // První se vždy najde ta nejvíce nasvícená dominantní stěna. Obě další zbylé stěny 
                // v témže snímku k ní musí být logicky perfektně kolmé nebo rovnoběžné! (+- 10 stupňů tolerance)
                if (num_walls > 0) {
                    float dot = abs(nx * walls[0].nx + ny * walls[0].ny);
                    // Tohle zamezí "šikmým zdem přes soupeře"! Přísná ortogonalizace uvnitř jednoho framu (10 stupňů = 0.984). 
                    if (dot > 0.174f && dot < 0.984f) {
                        continue; // Šikmá čára směřující bůhvíkde do soupeře se natvrdo odmítne!
                    }
                }
                
                float c = -(nx * p1x + ny * p1y); 
                
                int inliers = 0;
                float span_min = 999999, span_max = -999999;
                for (int i = 0; i < n_pts; i++) {
                    if (pts[i].used) continue;
                    float dist = abs(nx * pts[i].x + ny * pts[i].y + c);
                    if (dist < 15.0f) { // Zpřísněný offset na pouhých +- 1.5 cm! Kruhový soupeř už neprojde jako rovná záchytná hrana.
                        inliers++;
                        float proj = (-ny) * pts[i].x + (nx) * pts[i].y;
                        if (proj < span_min) span_min = proj;
                        if (proj > span_max) span_max = proj;
                    }
                }
                
                float span = span_max - span_min;
                if (inliers < 2) span = 0.0f; // Bezpečnost
                
                // VÝPOČET OSTRÉHO SKÓRE (Absolutní priorita hustoty. Delší zeď vítězí až při úplné shodě bodů!)
                float score = (float)inliers * 10000.0f + span;
                
                if (score > best_score) {
                    best_score = score; best_inliers = inliers; best_nx = nx; best_ny = ny; best_c = c;
                }
            }
            
            if (best_inliers < 30) {
                if (line_idx == 0 && mem_locked) {
                    mem_miss++;
                    if (mem_miss >= MEM_LIFESPAN_FRAMES) {
                        mem_locked = false; mem_miss = 0; mem_consistent = 0; // Paměť rozvázána
                    }
                }
                break; 
            }
            
            // --- LOGIKA PAMĚTI (Učení a Sledování stěny) ---
            if (line_idx == 0) {
                if (!mem_locked) {
                    if (mem_consistent == 0) {
                        mem_nx = best_nx; mem_ny = best_ny; mem_consistent = 1;
                    } else {
                        if (abs(best_nx * mem_nx + best_ny * mem_ny) > 0.85f) {
                            mem_consistent++;
                            mem_nx = best_nx; mem_ny = best_ny;
                            if (mem_consistent >= MEM_LIFESPAN_FRAMES) { mem_locked = true; mem_miss = 0; } // Naučeno!
                        } else {
                            mem_nx = best_nx; mem_ny = best_ny; mem_consistent = 1; // Šum, začínáme znovu
                        }
                    }
                } else {
                    mem_miss = 0; // Úspěšně jsme našli paměť, nulujeme čítač omylů
                    mem_nx = best_nx; mem_ny = best_ny; // Plynulé adaptování na točení robota!
                }
            }
            
            // Nyní spočítáme čistou statistiku nalezené stěny (Délka a orientace vektoru)
            float span_min = 999999, span_max = -999999;
            float avg_x = 0, avg_y = 0;
            int final_inliers = 0;
            
            // Druhá prochodní iterace přes Inliere – Teď stěnu trvale z pole "zkonzumujeme"
            for (int i = 0; i < n_pts; i++) {
                if (pts[i].used) continue;
                float dist = abs(best_nx * pts[i].x + best_ny * pts[i].y + best_c);
                if (dist < 15.0f) {
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
            
            if (final_inliers < 30) continue; // Přísná ochrana (hustota minimálně 30 naměřených zásahů na stěnu)
            
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
            
            
            walls[num_walls].nx = best_nx; walls[num_walls].ny = best_ny; walls[num_walls].c = best_c;
            num_walls++;
            
            send_packet_line(x1, y1, x2, y2);
        }

        // --- DETEKCE SOUPEŘE ZE ZBYTKU BODŮ ---
        
        // 1. Očistíme mapu od zbytků zdí (stíny, odštěpky, nedodělky), soupeř nesmí být nalepený < 12cm na zdi
        for (int i = 0; i < n_pts; i++) {
            if (!pts[i].used) {
                for (int w = 0; w < num_walls; w++) {
                    if (abs(walls[w].nx * pts[i].x + walls[w].ny * pts[i].y + walls[w].c) < 120.0f) {
                        pts[i].used = true; // Vymazáno!
                        break;
                    }
                }
            }
        }
        
        // 2. Najdeme nejhustší izolovaný cluster uprostřed
        int best_opp_pts = 0;
        float best_opp_x = 0, best_opp_y = 0;
        float c_pts_x[150]; // Lokální zásobník pro PCA matici chumlu (aby nedošlo k Stack Overflow uvnitř iterace)
        float c_pts_y[150];
        
        for (int i = 0; i < n_pts; i++) {
            if (pts[i].used) continue;
            
            int cluster_pts = 0;
            float sum_x = 0, sum_y = 0;
            for (int j = 0; j < n_pts; j++) {
                if (!pts[j].used) {
                    float dx = pts[j].x - pts[i].x;
                    float dy = pts[j].y - pts[i].y;
                    if (dx*dx + dy*dy < 40000.0f) { // Soupeř se vejde do okruhu 20 cm
                        if (cluster_pts < 150) {
                            c_pts_x[cluster_pts] = pts[j].x;
                            c_pts_y[cluster_pts] = pts[j].y;
                        }
                        cluster_pts++;
                        sum_x += pts[j].x;
                        sum_y += pts[j].y;
                    }
                }
            }
            
            int n_c = (cluster_pts > 150) ? 150 : cluster_pts; // Ochrana proti buffer overflow
            
            if (n_c >= 5) {
                // PCA Analýza dat (Je to Soupeř (křivý) nebo nedetekovaná Rovná zeď(šum)?)
                float mean_x = sum_x / cluster_pts;
                float mean_y = sum_y / cluster_pts;
                float cov_xx = 0, cov_yy = 0, cov_xy = 0;
                
                for(int k=0; k<n_c; k++) {
                    float dx = c_pts_x[k] - mean_x;
                    float dy = c_pts_y[k] - mean_y;
                    cov_xx += dx*dx;
                    cov_yy += dy*dy;
                    cov_xy += dx*dy;
                }
                
                cov_xx /= n_c; cov_yy /= n_c; cov_xy /= n_c;
                
                float trace = cov_xx + cov_yy;
                float det = cov_xx * cov_yy - cov_xy * cov_xy;
                float disc = trace*trace - 4*det;
                if (disc < 0) disc = 0;
                
                float lambda1 = (trace + sqrt(disc)) / 2.0f; // Rozptyl podél osy (Length span)
                float lambda2 = (trace - sqrt(disc)) / 2.0f; // Rozptyl kolmo na osu (Thickness)
                
                // Pokud zbytek bodů tvoří útvar Tenčí než 13mm a přitom Delší než 30mm, 
                // znamená to, že se jedná o neověřenou rovinkatou stěnu, ne o soupeře! Jde rovnou z kola ven!
                if (sqrt(lambda2) < 13.0f && sqrt(lambda1) > 30.0f) {
                    continue; 
                }
                
                if (cluster_pts > best_opp_pts) {
                    best_opp_pts = cluster_pts;
                    best_opp_x = sum_x;
                    best_opp_y = sum_y;
                }
            }
        }
        
        if (best_opp_pts >= 5) { // Bezpečně zformovaný ne-rovný blob soupeře
            send_packet_opponent(rob_x + (int16_t)(best_opp_x / best_opp_pts), rob_y + (int16_t)(best_opp_y / best_opp_pts));
        }

        // Vyčistit lokální pole (nová otáčka zčuchne čistý papír dat)
        n_pts = 0;
        last_scan_time = millis();
    }
}
