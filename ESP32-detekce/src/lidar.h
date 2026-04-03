#pragma once
#include <Arduino.h>
#include <math.h>

#define LIDAR_RX 13
#define LIDAR_TX 10

#define MEM_LIFESPAN_FRAMES 500

const int PACKET_SIZE = 47;
uint8_t packet[PACKET_SIZE];
int packetIndex = 0;

struct PBuf { int16_t x; int16_t y; bool used; };
PBuf pts[500];
int n_pts = 0;
bool points_ready = false;

// --- PROTOKOL ---
// 0xBB 0x55 | type | len | payload... | checksum
// Type 0: zelená tečka  (x, y) globální
// Type 1: primka        (x1,y1,x2,y2) globální
// Type 2: nový frame    (rob_x, rob_y, heading_deg) — 6 bytů
// Type 3: soupeř        (x, y) globální

void send_packet_new_frame(int16_t rx, int16_t ry, int16_t hdeg) {
    uint8_t buf[11];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 2; buf[3] = 6;
    buf[4] = rx   & 0xFF; buf[5] = (rx  >> 8) & 0xFF;
    buf[6] = ry   & 0xFF; buf[7] = (ry  >> 8) & 0xFF;
    buf[8] = hdeg & 0xFF; buf[9] = (hdeg >> 8) & 0xFF;
    buf[10] = (buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]) & 0xFF;
    Serial.write(buf, 11);
}

void send_packet_point(int16_t x, int16_t y) {
    uint8_t buf[9];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 0; buf[3] = 4;
    buf[4] = x & 0xFF; buf[5] = (x >> 8) & 0xFF;
    buf[6] = y & 0xFF; buf[7] = (y >> 8) & 0xFF;
    buf[8] = (buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]) & 0xFF;
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
    buf[12] = (buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]+buf[11]) & 0xFF;
    Serial.write(buf, 13);
}

void send_packet_opponent(int16_t x, int16_t y) {
    uint8_t buf[9];
    buf[0] = 0xBB; buf[1] = 0x55;
    buf[2] = 3; buf[3] = 4;
    buf[4] = x & 0xFF; buf[5] = (x >> 8) & 0xFF;
    buf[6] = y & 0xFF; buf[7] = (y >> 8) & 0xFF;
    buf[8] = (buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]) & 0xFF;
    Serial.write(buf, 9);
}

// --- PARSOVÁNÍ LIDAR PAKETU ---
// Lokální souřadnice: střed = robot, +X = pravá strana, +Y = dopředu
void processPacket() {
    uint16_t startAngleX100 = packet[4] | (packet[5] << 8);
    uint16_t endAngleX100   = packet[42] | (packet[43] << 8);
    float startAngle = startAngleX100 / 100.0f;
    float endAngle   = endAngleX100   / 100.0f;
    float step = (endAngle < startAngle)
        ? (endAngle + 360.0f - startAngle) / 11.0f
        : (endAngle - startAngle) / 11.0f;

    for (int i = 0; i < 12; i++) {
        int baseIndex = 6 + i * 3;
        uint16_t distance = packet[baseIndex] | (packet[baseIndex + 1] << 8);
        float angle = startAngle + step * i;
        angle -= 90.0f;
        while (angle >= 360.0f) angle -= 360.0f;
        while (angle < 0.0f)    angle += 360.0f;

        // Přední polokoule: 5° – 175° (lokální)
        if (angle >= 5.0f && angle <= 175.0f && distance >= 300 && distance < 1500) {
            float rad = angle * (PI / 180.0f);
            if (n_pts < 500) {
                pts[n_pts].x = (int16_t)roundf((float)distance * cosf(rad));
                pts[n_pts].y = (int16_t)roundf((float)distance * sinf(rad));
                pts[n_pts].used = false;
                n_pts++;
            }
        }
    }
}

void init_lidar() {
    Serial.begin(921600);
    randomSeed(analogRead(34));
    Serial2.setRxBufferSize(1024);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

// --- PAMĚŤ DOMINANTNÍ STĚNY ---
float mem_nx = 0, mem_ny = 0;
int   mem_consistent = 0;
bool  mem_locked = false;

// Stabilní pozice robota (průměrovaná z rámce do rámce)
static float global_rob_x = 500.0f;
static float global_rob_y = 500.0f;
static float global_heading = 0.0f; // radiány CW od severu (+Y globální)

// -----------------------------------------------------------------------
// Pomocná funkce: transformuje lokální bod na globální
// CW konvence: gx = rx + lx*cos(h) + ly*sin(h)
//              gy = ry - lx*sin(h) + ly*cos(h)
// -----------------------------------------------------------------------
static inline void local_to_global(float lx, float ly,
                                    float rx,  float ry, float h,
                                    int16_t &gx, int16_t &gy) {
    gx = (int16_t)roundf(rx + lx * cosf(h) + ly * sinf(h));
    gy = (int16_t)roundf(ry - lx * sinf(h) + ly * cosf(h));
}

// -----------------------------------------------------------------------
void loop_lidar() {
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        if (packetIndex == 0) {
            if (c == 0x54) packet[packetIndex++] = c;
        } else if (packetIndex == 1) {
            if      (c == 0x2C) packet[packetIndex++] = c;
            else if (c == 0x54) packetIndex = 1;
            else                packetIndex = 0;
        } else {
            packet[packetIndex++] = c;
            if (packetIndex == PACKET_SIZE) {
                processPacket();
                packetIndex = 0;
                if (n_pts > 400) points_ready = true;
            }
        }
    }

    if (!points_ready) return;
    points_ready = false;
    int n_local = n_pts;

    // =======================================================
    // RANSAC – hledání až 3 stěn v lokálních souřadnicích
    // =======================================================
    struct WallParams { float nx, ny, c; };
    struct WallSeg    { float x1, y1, x2, y2; };
    WallParams walls[3];
    WallSeg    segs[3];
    int num_walls = 0;

    for (int line_idx = 0; line_idx < 3; line_idx++) {
        int   best_n = 0;
        float best_score = 0, best_nx = 0, best_ny = 0, best_c = 0;

        for (int iter = 0; iter < 200; iter++) {
            int i1 = random(0, n_local), i2 = random(0, n_local);
            if (pts[i1].used || pts[i2].used || i1 == i2) continue;
            float dx = pts[i2].x - pts[i1].x;
            float dy = pts[i2].y - pts[i1].y;
            float len = sqrtf(dx*dx + dy*dy);
            if (len < 10.0f) continue;
            float nx = -dy/len, ny = dx/len;

            // Paměťový zámek na dominantu
            if (line_idx == 0 && mem_locked)
                if (fabsf(nx*mem_nx + ny*mem_ny) < 0.85f) continue;

            // Ortogonální omezení vůči první stěně
            if (num_walls > 0) {
                float dot = fabsf(nx*walls[0].nx + ny*walls[0].ny);
                if (dot > 0.174f && dot < 0.984f) continue;
            }

            float c = -(nx*pts[i1].x + ny*pts[i1].y);
            int inliers = 0;
            float smin = 1e9, smax = -1e9;
            for (int i = 0; i < n_local; i++) {
                if (pts[i].used) continue;
                if (fabsf(nx*pts[i].x + ny*pts[i].y + c) < 15.0f) {
                    inliers++;
                    float proj = -ny*pts[i].x + nx*pts[i].y;
                    if (proj < smin) smin = proj;
                    if (proj > smax) smax = proj;
                }
            }
            float score = inliers * 10000.0f + (smax - smin);
            if (score > best_score) {
                best_score = score; best_n = inliers;
                best_nx = nx; best_ny = ny; best_c = c;
            }
        }

        if (best_n < 30) break;

        // Paměť dominantní stěny
        if (line_idx == 0) {
            if (!mem_locked) {
                if (mem_consistent == 0 || fabsf(best_nx*mem_nx + best_ny*mem_ny) > 0.85f) {
                    mem_consistent++;
                    mem_nx = best_nx; mem_ny = best_ny;
                    if (mem_consistent >= MEM_LIFESPAN_FRAMES) mem_locked = true;
                } else { mem_nx = best_nx; mem_ny = best_ny; mem_consistent = 1; }
            } else { mem_nx = best_nx; mem_ny = best_ny; }
        }

        // Druhý průchod – "spotřebování" bodů + přesné krajní body
        float smin = 1e9, smax = -1e9;
        int final_n = 0;
        for (int i = 0; i < n_local; i++) {
            if (pts[i].used) continue;
            if (fabsf(best_nx*pts[i].x + best_ny*pts[i].y + best_c) < 15.0f) {
                pts[i].used = true; final_n++;
                float proj = -best_ny*pts[i].x + best_nx*pts[i].y;
                if (proj < smin) smin = proj;
                if (proj > smax) smax = proj;
            }
        }
        if (final_n < 30 || (smax - smin) < 350.0f) continue;

        // Krajní body primky v lokálním systému (mm od robota)
        walls[num_walls] = { best_nx, best_ny, best_c };
        segs[num_walls]  = {
            -best_ny*smin - best_nx*best_c,
             best_nx*smin - best_ny*best_c,
            -best_ny*smax - best_nx*best_c,
             best_nx*smax - best_ny*best_c
        };
        num_walls++;
    }

    // =======================================================
    // SLAM: z RANSAC stěn odvodit pozici + heading robota
    //
    // Princip "překryvu":
    //   Každá detekovaná primka musí odpovídat jedné ze 4 hranic
    //   arény (X=0, X=1000, Y=0, Y=1000).
    //   Vzdálenost od robota k primce = |c|.
    //   "Kolmice" (foot) z robota na primku = (-c*nx, -c*ny).
    //   Heading = úhel foot vektoru od lokálního +Y (dopředu).
    //   Robot_x nebo Robot_y = vzdálenost od té hranice arény.
    // =======================================================

    if (num_walls > 0) {
        // --- Heading z dominantní stěny ---
        // foot = vektor od robota k nejbližšímu bodu na primce
        float f0x = -walls[0].c * walls[0].nx;
        float f0y = -walls[0].c * walls[0].ny;
        // Úhel foot od lokálního +Y (dopředu robota)
        // heading = 0  → robot směřuje sever (+Y globálně), wall přímo vpřed
        // heading > 0  → robot otočen CW (doprava)
        float new_heading = atan2f(f0x, f0y);

        // Snapping na nejbližší 90° + zachování jemného úhlu (pro stěnu vpravo/vlevo/nahoře)
        // Protože LiDAR vidí jen 5°–175°, foot_y > 0 vždy (stěna vždy vpřed).
        // new_heading ≈ skutečný heading robota vůči normále dané stěny.
        // Abychom získali absolutní heading, musíme vědět KTERÉ stěně odpovídá:
        //   |f0y| >> |f0x|  → horizontální stěna = horní/dolní (Y=0 nebo Y=1000)
        //   |f0x| >> |f0y|  → vertikální stěna = levá/pravá   (X=0 nebo X=1000)
        // Heading snapsujeme tak, aby north (0°) odpovídal pohledu na horní stěnu přímo vpřed.

        float abs_heading;
        if (fabsf(f0y) >= fabsf(f0x)) {
            // Horizontální stěna → heading = new_heading přímo
            abs_heading = new_heading;
        } else {
            // Vertikální stěna → robot vidí boční stěnu jako "přední" → posunout o ±90°
            if (f0x > 0.0f) abs_heading = new_heading - (PI / 2.0f); // stěna vpravo
            else             abs_heading = new_heading + (PI / 2.0f); // stěna vlevo
        }

        // Plynulá aktualizace headingu (LERP 30%)
        global_heading = global_heading * 0.7f + abs_heading * 0.3f;

        // --- Pozice robota z každé detekované stěny ---
        for (int i = 0; i < num_walls; i++) {
            float fix = -walls[i].c * walls[i].nx;
            float fiy = -walls[i].c * walls[i].ny;
            float dist = fabsf(walls[i].c);

            // Rotujeme foot vektor do globálního rámce (CW konvence)
            // g_foot = R_cw(heading) * local_foot
            float gfx = fix * cosf(global_heading) + fiy * sinf(global_heading);
            float gfy = -fix * sinf(global_heading) + fiy * cosf(global_heading);

            // gfx/gfy říká, na které GLOBÁLNÍ straně leží stěna od robota:
            //   gfy >> 0  → stěna je globálně "nahoře" (Y=1000)
            //   gfy << 0  → stěna je globálně "dole"   (Y=0)     [LiDARem těžko vidět]
            //   gfx >> 0  → stěna je globálně "vpravo"  (X=1000)
            //   gfx << 0  → stěna je globálně "vlevo"   (X=0)
            const float thresh = 0.6f;
            if (gfy > thresh * dist) {
                // Stěna je globálně nahoře → Y=1000
                global_rob_y = global_rob_y * 0.7f + (1000.0f - dist) * 0.3f;
            } else if (gfy < -thresh * dist) {
                // Stěna je globálně dole → Y=0
                global_rob_y = global_rob_y * 0.7f + dist * 0.3f;
            } else if (gfx > thresh * dist) {
                // Stěna je globálně vpravo → X=1000
                global_rob_x = global_rob_x * 0.7f + (1000.0f - dist) * 0.3f;
            } else if (gfx < -thresh * dist) {
                // Stěna je globálně vlevo → X=0
                global_rob_x = global_rob_x * 0.7f + dist * 0.3f;
            }
        }
    }

    // Zaokrouhlení pro protokol
    int16_t rx = (int16_t)roundf(global_rob_x);
    int16_t ry = (int16_t)roundf(global_rob_y);
    int16_t hd = (int16_t)roundf(global_heading * 180.0f / PI);

    // =======================================================
    // ODESLÁNÍ DAT DO PC
    // Pořadí: 1) nový frame (smaže staré čáry v Pythonu)
    //         2) transformované stěny (překryjí hranice arény)
    //         3) transformované zelené body (point cloud)
    //         4) soupeř
    // =======================================================
    send_packet_new_frame(rx, ry, hd);

    // Stěny v globálních souřadnicích (přímky překrývající hranice arény)
    for (int i = 0; i < num_walls; i++) {
        int16_t gx1, gy1, gx2, gy2;
        local_to_global(segs[i].x1, segs[i].y1, global_rob_x, global_rob_y, global_heading, gx1, gy1);
        local_to_global(segs[i].x2, segs[i].y2, global_rob_x, global_rob_y, global_heading, gx2, gy2);
        send_packet_line(gx1, gy1, gx2, gy2);
    }

    // Zelené body v globálních souřadnicích
    for (int i = 0; i < n_local; i++) {
        int16_t gx, gy;
        local_to_global((float)pts[i].x, (float)pts[i].y,
                        global_rob_x, global_rob_y, global_heading, gx, gy);
        send_packet_point(gx, gy);
    }

    // --- Detekce soupeře (PCA filtr) ---
    for (int i = 0; i < n_local; i++) {
        if (!pts[i].used) {
            for (int w = 0; w < num_walls; w++) {
                if (fabsf(walls[w].nx*pts[i].x + walls[w].ny*pts[i].y + walls[w].c) < 120.0f) {
                    pts[i].used = true; break;
                }
            }
        }
    }

    int   best_opp = 0; float box = 0, boy = 0;
    float cpx[150], cpy[150];
    for (int i = 0; i < n_local; i++) {
        if (pts[i].used) continue;
        int cnt = 0; float sx = 0, sy = 0;
        for (int j = 0; j < n_local; j++) {
            if (!pts[j].used) {
                float dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y;
                if (dx*dx + dy*dy < 40000.0f) {
                    if (cnt < 150) { cpx[cnt] = pts[j].x; cpy[cnt] = pts[j].y; }
                    cnt++; sx += pts[j].x; sy += pts[j].y;
                }
            }
        }
        int nc = (cnt > 150) ? 150 : cnt;
        if (nc < 5) continue;
        float mx = sx/cnt, my = sy/cnt, cxx=0, cyy=0, cxy=0;
        for (int k = 0; k < nc; k++) {
            float dx=cpx[k]-mx, dy=cpy[k]-my;
            cxx+=dx*dx; cyy+=dy*dy; cxy+=dx*dy;
        }
        cxx/=nc; cyy/=nc; cxy/=nc;
        float tr=cxx+cyy, det=cxx*cyy-cxy*cxy, disc=tr*tr-4*det;
        if (disc<0) disc=0;
        float l1=(tr+sqrtf(disc))/2, l2=(tr-sqrtf(disc))/2;
        if (sqrtf(l2) < 13.0f && sqrtf(l1) > 30.0f) continue;
        if (cnt > best_opp) { best_opp = cnt; box = sx; boy = sy; }
    }

    if (best_opp >= 5) {
        int16_t gox, goy;
        local_to_global(box/best_opp, boy/best_opp,
                        global_rob_x, global_rob_y, global_heading, gox, goy);
        send_packet_opponent(gox, goy);
    }

    n_pts = 0;
}
