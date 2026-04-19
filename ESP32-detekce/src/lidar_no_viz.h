#pragma once
#include <Arduino.h>
#include <math.h>

// ============================================================
//  lidar_no_viz.h  –  SLAM výstup do Serial Monitoru (textově)
//  Neposílá binární data, nepoužívá Python vizualizátor.
//  Výstup:  POS  X Y  HEADING  DIST_HOME  [OPP  X Y  DIST  REL_ANGLE]
// ============================================================

// === Rozměry robota a poloha LiDARu ===
#define NV_ROBOT_LENGTH_MM   350.0f
#define NV_ROBOT_WIDTH_MM    300.0f
#define NV_LIDAR_FROM_FRONT   40.0f
#define NV_LIDAR_FRONT_EDGE  ( NV_LIDAR_FROM_FRONT)
#define NV_LIDAR_BACK_EDGE   (-(NV_ROBOT_LENGTH_MM - NV_LIDAR_FROM_FRONT))
#define NV_LIDAR_HALF_WIDTH  ( NV_ROBOT_WIDTH_MM / 2.0f)
#define NV_LIDAR_TO_CENTER   (NV_ROBOT_LENGTH_MM / 2.0f - NV_LIDAR_FROM_FRONT)

#define NV_LIDAR_RX 13
#define NV_LIDAR_TX 10
#define NV_MEM_LIFESPAN_FRAMES 500
#define NV_ARENA_SIZE 1500.0f

// === Zorné pole LiDARu (stupně) — které úhly bereme v potaz ===
#define NV_FOV_MIN   5.0f    // spodní mez (°)
#define NV_FOV_MAX 175.0f    // horní mez (°)

// Korekční odchylka pro fyzické usazení lidaru (jemné ladění zrcadlové osy)
static float angleOffset = 7.0f;

// --- Pozice "domů" (pravý dolní roh, 30 cm od krajů) ---
#define HOME_X  (NV_ARENA_SIZE - 300.0f)
#define HOME_Y  300.0f

// ---- Interní stav (vlastní namespace pomocí nv_ prefixu) ----
static const int NV_PACKET_SIZE = 47;
static uint8_t nv_packet[NV_PACKET_SIZE];
static int nv_packetIndex = 0;

struct NV_PBuf { int16_t x; int16_t y; bool used; };
static NV_PBuf nv_pts[500];
static int nv_n_pts = 0;
static bool nv_points_ready = false;

// SLAM stav (vyhlazené)
static float nv_g_rx = 500.0f, nv_g_ry = 500.0f, nv_g_h = 0.0f;
// Surové z RANSACu
static float nv_raw_rx = 500.0f, nv_raw_ry = 500.0f, nv_raw_h = 0.0f;
// EMA heading (sin/cos)
static float nv_h_sin_avg = 0.0f, nv_h_cos_avg = 1.0f;

// EMA alfy – stejné jako v lidar.h
#define NV_SLAM_ALPHA_H    0.60f
#define NV_SLAM_ALPHA_POS  0.50f

// Paměť dominantní stěny
static float nv_mem_nx = 0, nv_mem_ny = 0;
static int nv_mem_consistent = 0;
static bool nv_mem_locked = false;

// Soupeř – poslední známá pozice (globální), platný flag
static bool nv_opp_valid = false;
static float nv_opp_gx = 0, nv_opp_gy = 0;

// ---------- Pomocné funkce ----------

// Lokální → globální transformace
static void nv_l2g(float lx, float ly, float &gx, float &gy) {
    float ch = cosf(nv_g_h), sh = sinf(nv_g_h);
    gx = nv_g_rx + lx*ch + ly*sh;
    gy = nv_g_ry - lx*sh + ly*ch;
}

// ---------- Zpracování paketu z LiDARu ----------
static void nv_processPacket() {
    uint16_t sa = nv_packet[4]|(nv_packet[5]<<8);
    uint16_t ea = nv_packet[42]|(nv_packet[43]<<8);
    float s = sa/100.0f, e = ea/100.0f;
    float step = (e<s) ? (e+360.0f-s)/11.0f : (e-s)/11.0f;
    for (int i=0; i<12; i++) {
        uint16_t d = nv_packet[6+i*3]|(nv_packet[7+i*3]<<8);
        float a = s + step*i - 90.0f;

        // Kalibrační oprava - fyzické zarovnání natočení
        a -= angleOffset;
        // Pojistka na ošetření přetečení (pokud tam zatím žádnou nemáš)
        while (a >= 360.0f) a -= 360.0f;
        while (a < 0.0f) a += 360.0f;

        if (a>=NV_FOV_MIN && a<=NV_FOV_MAX && d>0 && nv_n_pts<500) {
            float r = a*(PI/180.0f);
            nv_pts[nv_n_pts].x = (int16_t)roundf(d*cosf(r));
            nv_pts[nv_n_pts].y = (int16_t)roundf(d*sinf(r));
            nv_pts[nv_n_pts].used = false;
            nv_n_pts++;
        }
    }
}

// ---------- Init ----------
void init_lidar_nv() {
    Serial.begin(115200);      // Pomalejší baud pro čitelný Serial Monitor
    randomSeed(analogRead(34));
    Serial2.setRxBufferSize(1024);
    Serial2.begin(230400, SERIAL_8N1, NV_LIDAR_RX, NV_LIDAR_TX);
    Serial.println(F("=== LIDAR Serial Monitor (no-viz) ==="));
    Serial.println(F("Formát: POS x y | HEAD ° | HOME dist angle° | OPP x y dist rel°"));
}

// ---------- Hlavní smyčka ----------
void loop_lidar_nv() {
    // ---- Čtení paketů ----
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        if (nv_packetIndex==0) { if(c==0x54) nv_packet[nv_packetIndex++]=c; }
        else if (nv_packetIndex==1) {
            if(c==0x2C) nv_packet[nv_packetIndex++]=c;
            else if(c==0x54) nv_packetIndex=1; else nv_packetIndex=0;
        } else {
            nv_packet[nv_packetIndex++]=c;
            if (nv_packetIndex==NV_PACKET_SIZE) {
                nv_processPacket(); nv_packetIndex=0;
                if (nv_n_pts>400) nv_points_ready=true;
            }
        }
    }
    if (!nv_points_ready) return;
    nv_points_ready = false;
    int nl = nv_n_pts;

    // ===================== RANSAC: až 3 stěny =====================
    struct W { float nx,ny,c; };
    struct S { float x1,y1,x2,y2; };
    W walls[3]; S segs[3]; int nw = 0;

    for (int li=0; li<3; li++) {
        int bn=0; float bs=0, bnx=0, bny=0, bc=0;
        for (int it=0; it<200; it++) {
            int i1=random(0,nl), i2=random(0,nl);
            if (nv_pts[i1].used||nv_pts[i2].used||i1==i2) continue;
            float dx=nv_pts[i2].x-nv_pts[i1].x, dy=nv_pts[i2].y-nv_pts[i1].y;
            float len=sqrtf(dx*dx+dy*dy);
            if (len<10) continue;
            float nx=-dy/len, ny=dx/len;
            if (li==0 && nv_mem_locked && fabsf(nx*nv_mem_nx+ny*nv_mem_ny)<0.85f) continue;
            if (nw>0) { float d=fabsf(nx*walls[0].nx+ny*walls[0].ny); if(d>0.174f&&d<0.984f) continue; }
            float cc=-(nx*nv_pts[i1].x+ny*nv_pts[i1].y);
            int inl=0; float sn=1e9, sx=-1e9;
            for (int i=0;i<nl;i++) {
                if(nv_pts[i].used) continue;
                if(fabsf(nx*nv_pts[i].x+ny*nv_pts[i].y+cc)<15) {
                    inl++; float p=-ny*nv_pts[i].x+nx*nv_pts[i].y;
                    if(p<sn) sn=p; if(p>sx) sx=p;
                }
            }
            float sc=inl*10000.0f+(sx-sn);
            if(sc>bs) { bs=sc; bn=inl; bnx=nx; bny=ny; bc=cc; }
        }
        if (bn<30) break;
        // Paměť
        if (li==0) {
            if (!nv_mem_locked) {
                if (nv_mem_consistent==0||fabsf(bnx*nv_mem_nx+bny*nv_mem_ny)>0.85f) {
                    nv_mem_consistent++; nv_mem_nx=bnx; nv_mem_ny=bny;
                    if (nv_mem_consistent>=NV_MEM_LIFESPAN_FRAMES) nv_mem_locked=true;
                } else { nv_mem_nx=bnx; nv_mem_ny=bny; nv_mem_consistent=1; }
            } else { nv_mem_nx=bnx; nv_mem_ny=bny; }
        }
        // Spotřebování bodů + krajní body
        float sn=1e9, sx=-1e9; int fn=0;
        for (int i=0;i<nl;i++) {
            if(nv_pts[i].used) continue;
            if(fabsf(bnx*nv_pts[i].x+bny*nv_pts[i].y+bc)<15) {
                nv_pts[i].used=true; fn++;
                float p=-bny*nv_pts[i].x+bnx*nv_pts[i].y;
                if(p<sn) sn=p; if(p>sx) sx=p;
            }
        }
        if (fn<30||(sx-sn)<350) continue;
        walls[nw]={bnx,bny,bc};
        segs[nw]={-bny*sn-bnx*bc, bnx*sn-bny*bc, -bny*sx-bnx*bc, bnx*sx-bny*bc};
        nw++;
    }

    // ===================== SLAM: heading + pozice =====================
    if (nw > 0) {
        float fx = -walls[0].c * walls[0].nx;
        float fy = -walls[0].c * walls[0].ny;
        float ch = cosf(nv_g_h), sh = sinf(nv_g_h);

        float fgx = fx*ch + fy*sh;
        float fgy = -fx*sh + fy*ch;

        float toff = 0;
        if (fabsf(fgy) >= fabsf(fgx))
            toff = (fgy > 0) ? 0.0f : PI;
        else
            toff = (fgx > 0) ? (PI/2.0f) : (-PI/2.0f);

        nv_raw_h = toff + atan2f(-fx, fy);

        nv_h_sin_avg = (1.0f - NV_SLAM_ALPHA_H) * nv_h_sin_avg + NV_SLAM_ALPHA_H * sinf(nv_raw_h);
        nv_h_cos_avg = (1.0f - NV_SLAM_ALPHA_H) * nv_h_cos_avg + NV_SLAM_ALPHA_H * cosf(nv_raw_h);
        nv_g_h = atan2f(nv_h_sin_avg, nv_h_cos_avg);

        ch = cosf(nv_g_h); sh = sinf(nv_g_h);
        for (int i=0; i<nw; i++) {
            float fix = -walls[i].c * walls[i].nx;
            float fiy = -walls[i].c * walls[i].ny;
            float dist = fabsf(walls[i].c);
            float gfx = fix*ch + fiy*sh;
            float gfy = -fix*sh + fiy*ch;
            if (fabsf(gfy) >= fabsf(gfx)) {
                if (gfy>0) nv_raw_ry = NV_ARENA_SIZE - dist;
                else       nv_raw_ry = dist;
            } else {
                if (gfx>0) nv_raw_rx = NV_ARENA_SIZE - dist;
                else       nv_raw_rx = dist;
            }
        }
        nv_g_rx = (1.0f - NV_SLAM_ALPHA_POS) * nv_g_rx + NV_SLAM_ALPHA_POS * nv_raw_rx;
        nv_g_ry = (1.0f - NV_SLAM_ALPHA_POS) * nv_g_ry + NV_SLAM_ALPHA_POS * nv_raw_ry;
    }

    // ===================== Detekce soupeře (PCA) =====================
    nv_opp_valid = false;
    for (int i=0;i<nl;i++) {
        if(!nv_pts[i].used) {
            for(int w=0;w<nw;w++) {
                if(fabsf(walls[w].nx*nv_pts[i].x+walls[w].ny*nv_pts[i].y+walls[w].c)<120) {
                    nv_pts[i].used=true; break;
                }
            }
        }
    }
    int bo=0; float bsx=0,bsy=0;
    float cpx[150],cpy[150];
    for (int i=0;i<nl;i++) {
        if(nv_pts[i].used) continue;
        int cnt=0; float sx=0,sy=0;
        for(int j=0;j<nl;j++) {
            if(!nv_pts[j].used) {
                float dx=nv_pts[j].x-nv_pts[i].x, dy=nv_pts[j].y-nv_pts[i].y;
                if(dx*dx+dy*dy<40000) {
                    if(cnt<150){cpx[cnt]=nv_pts[j].x;cpy[cnt]=nv_pts[j].y;}
                    cnt++; sx+=nv_pts[j].x; sy+=nv_pts[j].y;
                }
            }
        }
        int nc=(cnt>150)?150:cnt;
        if(nc>=5) {
            float mx=sx/cnt,my=sy/cnt,cxx=0,cyy=0,cxy=0;
            for(int k=0;k<nc;k++){float dx=cpx[k]-mx,dy=cpy[k]-my;cxx+=dx*dx;cyy+=dy*dy;cxy+=dx*dy;}
            cxx/=nc;cyy/=nc;cxy/=nc;
            float tr=cxx+cyy,det=cxx*cyy-cxy*cxy,disc=tr*tr-4*det;
            if(disc<0)disc=0;
            if(sqrtf((tr-sqrtf(disc))/2)<13&&sqrtf((tr+sqrtf(disc))/2)>30) continue;
            if(cnt>bo){bo=cnt;bsx=sx;bsy=sy;}
        }
    }
    if(bo>=5) {
        float olx = bsx/bo, oly = bsy/bo;   // lokální pozice soupeře
        float ogx, ogy;
        nv_l2g(olx, oly, ogx, ogy);
        nv_opp_gx = ogx;
        nv_opp_gy = ogy;
        nv_opp_valid = true;
    }

    // ===================== VÝPIS NA SERIAL MONITOR =====================
    float heading_deg = nv_g_h * 180.0f / PI;
    float pos_x = constrain(nv_g_rx, 0, NV_ARENA_SIZE);
    float pos_y = constrain(nv_g_ry, 0, NV_ARENA_SIZE);

    // Vzdálenost a úhel domů
    float dx_home = HOME_X - pos_x;
    float dy_home = HOME_Y - pos_y;
    float dist_home = sqrtf(dx_home*dx_home + dy_home*dy_home);
    // Úhel k domovu v globálu (0 = sever/+Y, CW)
    float angle_to_home = atan2f(dx_home, dy_home);
    // Relativní úhel = kam je domov oproti mému směru pohledu
    float rel_home = (angle_to_home - nv_g_h) * 180.0f / PI;
    while (rel_home > 180.0f)  rel_home -= 360.0f;
    while (rel_home < -180.0f) rel_home += 360.0f;

    // Formát:  POS 452 318 | HEAD -12° | HOME 215mm  35°R
    Serial.printf("POS %4d %4d | HEAD %4d° | HOME %4dmm %3d°%c",
        (int)roundf(pos_x), (int)roundf(pos_y),
        (int)roundf(heading_deg),
        (int)roundf(dist_home),
        (int)roundf(fabsf(rel_home)),
        rel_home >= 0 ? 'R' : 'L');

    if (nv_opp_valid) {
        // Vzdálenost k soupeři
        float dx_opp = nv_opp_gx - pos_x;
        float dy_opp = nv_opp_gy - pos_y;
        float dist_opp = sqrtf(dx_opp*dx_opp + dy_opp*dy_opp);

        // Úhel soupeře v globálních souřadnicích (od severu CW)
        float angle_to_opp = atan2f(dx_opp, dy_opp);  // radiány, 0 = sever(+Y)
        // Relativní úhel = úhel soupeře − můj heading
        float rel_angle = (angle_to_opp - nv_g_h) * 180.0f / PI;
        // Normalizace do -180 .. +180
        while (rel_angle > 180.0f)  rel_angle -= 360.0f;
        while (rel_angle < -180.0f) rel_angle += 360.0f;

        Serial.printf(" | OPP %4d %4d  dist %4dmm  %3d°%c",
            (int)roundf(nv_opp_gx), (int)roundf(nv_opp_gy),
            (int)roundf(dist_opp),
            (int)roundf(fabsf(rel_angle)),
            rel_angle >= 0 ? 'R' : 'L');
    }

    Serial.println();

    nv_n_pts = 0;
}
