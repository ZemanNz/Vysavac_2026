#pragma once
#include <Arduino.h>
#include <math.h>

// === Rozměry robota a poloha LiDARu ===
// LiDAR je fyzický střed SLAM souřadnic – vše se měří od něj
#define ROBOT_LENGTH_MM   350.0f   // Délka robota (směr vpřed = lokální +Y)
#define ROBOT_WIDTH_MM    300.0f   // Šířka robota (lokální ±X)
#define LIDAR_FROM_FRONT   40.0f   // LiDAR je 40 mm od přední hrany
// Vzdálenosti od LiDARu k okrajům těla robota (lokální souřadnice)
#define LIDAR_FRONT_EDGE  ( LIDAR_FROM_FRONT)                           // +40 mm
#define LIDAR_BACK_EDGE   (-(ROBOT_LENGTH_MM - LIDAR_FROM_FRONT))       // -310 mm
#define LIDAR_HALF_WIDTH  ( ROBOT_WIDTH_MM / 2.0f)                      // ±150 mm
// Vzdálenost středu robota od LiDARu (pro případné přepočty)
#define LIDAR_TO_CENTER   (ROBOT_LENGTH_MM / 2.0f - LIDAR_FROM_FRONT)   // 135 mm dozadu

#define LIDAR_RX 13
#define LIDAR_TX 10
#define MEM_LIFESPAN_FRAMES 500
#define ARENA_SIZE 2500.0f

// === Zorné pole LiDARu (stupně) — které úhly bereme v potaz ===
#define FOV_MIN   5.0f    // spodní mez (°)
#define FOV_MAX 175.0f    // horní mez (°)

// Korekční odchylka pro fyzické usazení lidaru (jemné ladění zrcadlové osy)
static float angleOffset = 7.0f;

const int PACKET_SIZE = 47;
uint8_t packet[PACKET_SIZE];
int packetIndex = 0;

struct PBuf { int16_t x; int16_t y; bool used; };
PBuf pts[500];
int n_pts = 0;
bool points_ready = false;

// SLAM stav (vyhlazene - odesila se do Pythonu)
static float g_rx = 500.0f, g_ry = 500.0f, g_h = 0.0f;

// Rohove hodnoty pred vyhlazenem (surove z RANSACu)
static float raw_rx = 500.0f, raw_ry = 500.0f, raw_h = 0.0f;
// Pro heading: EMA pres sin/cos - bezpecne pri prechodu +/-180
static float h_sin_avg = 0.0f, h_cos_avg = 1.0f;
// Alfa pro EMA vyhlazovani:
//   H (heading): 1.0 = okamzita odezva (zadny lag), stabilitu zajistuje Memory Lock
//   POS (pozice): 0.30 = nova hodnota ma vahu 30%, zbytek je minula
//   Zvysit ALPHA_POS pro rychlejsi odezvu, snizit pro plynulejsi pohyb
#define SLAM_ALPHA_H    0.60f
#define SLAM_ALPHA_POS  0.50f

// Pamet dominantni steny
float mem_nx = 0, mem_ny = 0;
int mem_consistent = 0;
bool mem_locked = false;

// --- Protokol: 0xBB 0x55 | type | len | payload | checksum ---
void send_frame(int16_t rx, int16_t ry, int16_t hd) {
    uint8_t b[11];
    b[0]=0xBB; b[1]=0x55; b[2]=2; b[3]=6;
    b[4]=rx&0xFF; b[5]=(rx>>8)&0xFF;
    b[6]=ry&0xFF; b[7]=(ry>>8)&0xFF;
    b[8]=hd&0xFF; b[9]=(hd>>8)&0xFF;
    b[10]=(b[2]+b[3]+b[4]+b[5]+b[6]+b[7]+b[8]+b[9])&0xFF;
    Serial.write(b,11);
}
void send_point(int16_t x, int16_t y) {
    uint8_t b[9];
    b[0]=0xBB; b[1]=0x55; b[2]=0; b[3]=4;
    b[4]=x&0xFF; b[5]=(x>>8)&0xFF;
    b[6]=y&0xFF; b[7]=(y>>8)&0xFF;
    b[8]=(b[2]+b[3]+b[4]+b[5]+b[6]+b[7])&0xFF;
    Serial.write(b,9);
}
void send_line(int16_t x1,int16_t y1,int16_t x2,int16_t y2) {
    uint8_t b[13];
    b[0]=0xBB; b[1]=0x55; b[2]=1; b[3]=8;
    b[4]=x1&0xFF; b[5]=(x1>>8)&0xFF; b[6]=y1&0xFF; b[7]=(y1>>8)&0xFF;
    b[8]=x2&0xFF; b[9]=(x2>>8)&0xFF; b[10]=y2&0xFF; b[11]=(y2>>8)&0xFF;
    b[12]=(b[2]+b[3]+b[4]+b[5]+b[6]+b[7]+b[8]+b[9]+b[10]+b[11])&0xFF;
    Serial.write(b,13);
}
void send_opp(int16_t x, int16_t y) {
    uint8_t b[9];
    b[0]=0xBB; b[1]=0x55; b[2]=3; b[3]=4;
    b[4]=x&0xFF; b[5]=(x>>8)&0xFF;
    b[6]=y&0xFF; b[7]=(y>>8)&0xFF;
    b[8]=(b[2]+b[3]+b[4]+b[5]+b[6]+b[7])&0xFF;
    Serial.write(b,9);
}

// Lokalni → globalni transformace (CW konvence)
// gx = rx + lx*cos(h) + ly*sin(h)
// gy = ry - lx*sin(h) + ly*cos(h)
void l2g(float lx, float ly, int16_t &gx, int16_t &gy) {
    float ch = cosf(g_h), sh = sinf(g_h);
    gx = (int16_t)roundf(g_rx + lx*ch + ly*sh);
    gy = (int16_t)roundf(g_ry - lx*sh + ly*ch);
}

void processPacket() {
    uint16_t sa = packet[4]|(packet[5]<<8);
    uint16_t ea = packet[42]|(packet[43]<<8);
    float s = sa/100.0f, e = ea/100.0f;
    float step = (e<s) ? (e+360.0f-s)/11.0f : (e-s)/11.0f;
    for (int i=0; i<12; i++) {
        uint16_t d = packet[6+i*3]|(packet[7+i*3]<<8);
        float a = s + step*i - 90.0f;

        // Kalibrační oprava - fyzické zarovnání natočení
        a -= angleOffset;
        // Pojistka na ošetření přetečení (pokud tam zatím žádnou nemáš)
        while (a >= 360.0f) a -= 360.0f;
        while (a < 0.0f) a += 360.0f;

        if (a>=FOV_MIN && a<=FOV_MAX && d>0 && n_pts<500) {
            float r = a*(PI/180.0f);
            pts[n_pts].x = (int16_t)roundf(d*cosf(r));
            pts[n_pts].y = (int16_t)roundf(d*sinf(r));
            pts[n_pts].used = false;
            n_pts++;
        }
    }
}

void init_lidar() {
    Serial.begin(921600);
    randomSeed(analogRead(34));
    Serial2.setRxBufferSize(1024);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
}

void loop_lidar() {
    while (Serial2.available()) {
        uint8_t c = Serial2.read();
        if (packetIndex==0) { if(c==0x54) packet[packetIndex++]=c; }
        else if (packetIndex==1) {
            if(c==0x2C) packet[packetIndex++]=c;
            else if(c==0x54) packetIndex=1; else packetIndex=0;
        } else {
            packet[packetIndex++]=c;
            if (packetIndex==PACKET_SIZE) {
                processPacket(); packetIndex=0;
                if (n_pts>400) points_ready=true;
            }
        }
    }
    if (!points_ready) return;
    points_ready = false;
    int nl = n_pts;

    // === RANSAC: az 3 steny v lokalnich souradnicich ===
    struct W { float nx,ny,c; };
    struct S { float x1,y1,x2,y2; };
    W walls[3]; S segs[3]; int nw = 0;

    for (int li=0; li<3; li++) {
        int bn=0; float bs=0, bnx=0, bny=0, bc=0;
        for (int it=0; it<200; it++) {
            int i1=random(0,nl), i2=random(0,nl);
            if (pts[i1].used||pts[i2].used||i1==i2) continue;
            float dx=pts[i2].x-pts[i1].x, dy=pts[i2].y-pts[i1].y;
            float len=sqrtf(dx*dx+dy*dy);
            if (len<10) continue;
            float nx=-dy/len, ny=dx/len;
            if (li==0 && mem_locked && fabsf(nx*mem_nx+ny*mem_ny)<0.85f) continue;
            if (nw>0) { float d=fabsf(nx*walls[0].nx+ny*walls[0].ny); if(d>0.174f&&d<0.984f) continue; }
            float cc=-(nx*pts[i1].x+ny*pts[i1].y);
            int inl=0; float sn=1e9, sx=-1e9;
            for (int i=0;i<nl;i++) {
                if(pts[i].used) continue;
                if(fabsf(nx*pts[i].x+ny*pts[i].y+cc)<15) {
                    inl++; float p=-ny*pts[i].x+nx*pts[i].y;
                    if(p<sn) sn=p; if(p>sx) sx=p;
                }
            }
            float sc=inl*10000.0f+(sx-sn);
            if(sc>bs) { bs=sc; bn=inl; bnx=nx; bny=ny; bc=cc; }
        }
        if (bn<30) break;
        // Pamet
        if (li==0) {
            if (!mem_locked) {
                if (mem_consistent==0||fabsf(bnx*mem_nx+bny*mem_ny)>0.85f) {
                    mem_consistent++; mem_nx=bnx; mem_ny=bny;
                    if (mem_consistent>=MEM_LIFESPAN_FRAMES) mem_locked=true;
                } else { mem_nx=bnx; mem_ny=bny; mem_consistent=1; }
            } else { mem_nx=bnx; mem_ny=bny; }
        }
        // Spotrebovani bodu + krajni body
        float sn=1e9, sx=-1e9; int fn=0;
        for (int i=0;i<nl;i++) {
            if(pts[i].used) continue;
            if(fabsf(bnx*pts[i].x+bny*pts[i].y+bc)<15) {
                pts[i].used=true; fn++;
                float p=-bny*pts[i].x+bnx*pts[i].y;
                if(p<sn) sn=p; if(p>sx) sx=p;
            }
        }
        if (fn<30||(sx-sn)<350) continue;
        walls[nw]={bnx,bny,bc};
        segs[nw]={-bny*sn-bnx*bc, bnx*sn-bny*bc, -bny*sx-bnx*bc, bnx*sx-bny*bc};
        nw++;
    }

    // === SLAM: heading + pozice z prekryvu ===
    if (nw > 0) {
        // Foot dominantni steny (vektor od robota ke zdi)
        float fx = -walls[0].c * walls[0].nx;
        float fy = -walls[0].c * walls[0].ny;
        float ch = cosf(g_h), sh = sinf(g_h);

        // Rotace foot do globalniho ramce (pomoci PREDCHOZIHO headingu)
        float fgx = fx*ch + fy*sh;
        float fgy = -fx*sh + fy*ch;

        // Urceni ktere hranici areny odpovida dominantni stena
        float toff = 0;
        if (fabsf(fgy) >= fabsf(fgx))
            toff = (fgy > 0) ? 0.0f : PI;
        else
            toff = (fgx > 0) ? (PI/2.0f) : (-PI/2.0f);

        // Heading = offset + odchylka foot od "primo vpred" (surov, pred EMA)
        raw_h = toff + atan2f(-fx, fy);

        // --- EMA vyhlazeni headingu pres sin/cos ---
        h_sin_avg = (1.0f - SLAM_ALPHA_H) * h_sin_avg + SLAM_ALPHA_H * sinf(raw_h);
        h_cos_avg = (1.0f - SLAM_ALPHA_H) * h_cos_avg + SLAM_ALPHA_H * cosf(raw_h);
        g_h = atan2f(h_sin_avg, h_cos_avg);

        // Pozice z kazde steny (s vyhlazenym headingem)
        ch = cosf(g_h); sh = sinf(g_h);
        for (int i=0; i<nw; i++) {
            float fix = -walls[i].c * walls[i].nx;
            float fiy = -walls[i].c * walls[i].ny;
            float dist = fabsf(walls[i].c);
            float gfx = fix*ch + fiy*sh;
            float gfy = -fix*sh + fiy*ch;
            if (fabsf(gfy) >= fabsf(gfx)) {
                if (gfy>0) raw_ry = ARENA_SIZE - dist; // horni stena Y=1000
                else       raw_ry = dist;               // spodni Y=0
            } else {
                if (gfx>0) raw_rx = ARENA_SIZE - dist; // prava X=1000
                else       raw_rx = dist;               // leva X=0
            }
        }
        // EMA vyhlazeni pozice
        g_rx = (1.0f - SLAM_ALPHA_POS) * g_rx + SLAM_ALPHA_POS * raw_rx;
        g_ry = (1.0f - SLAM_ALPHA_POS) * g_ry + SLAM_ALPHA_POS * raw_ry;
    }

    // === Odeslani vsech dat v globalnich souradnicich ===
    int16_t rx=(int16_t)roundf(constrain(g_rx,0,ARENA_SIZE));
    int16_t ry=(int16_t)roundf(constrain(g_ry,0,ARENA_SIZE));
    int16_t hd=(int16_t)roundf(g_h*180.0f/PI);
    send_frame(rx, ry, hd);

    // Steny (primky prekryvajici hranice areny)
    for (int i=0; i<nw; i++) {
        int16_t gx1,gy1,gx2,gy2;
        l2g(segs[i].x1, segs[i].y1, gx1, gy1);
        l2g(segs[i].x2, segs[i].y2, gx2, gy2);
        send_line(gx1,gy1,gx2,gy2);
    }

    // Zelene body
    for (int i=0; i<nl; i++) {
        int16_t gx,gy;
        l2g((float)pts[i].x, (float)pts[i].y, gx, gy);
        send_point(gx, gy);
    }

    // === Detekce soupere (PCA) ===
    for (int i=0;i<nl;i++) {
        if(!pts[i].used) {
            for(int w=0;w<nw;w++) {
                if(fabsf(walls[w].nx*pts[i].x+walls[w].ny*pts[i].y+walls[w].c)<120) {
                    pts[i].used=true; break;
                }
            }
        }
    }
    int bo=0; float bsx=0,bsy=0;
    float cpx[150],cpy[150];
    for (int i=0;i<nl;i++) {
        if(pts[i].used) continue;
        int cnt=0; float sx=0,sy=0;
        for(int j=0;j<nl;j++) {
            if(!pts[j].used) {
                float dx=pts[j].x-pts[i].x, dy=pts[j].y-pts[i].y;
                if(dx*dx+dy*dy<40000) {
                    if(cnt<150){cpx[cnt]=pts[j].x;cpy[cnt]=pts[j].y;}
                    cnt++; sx+=pts[j].x; sy+=pts[j].y;
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
        int16_t gx,gy;
        l2g(bsx/bo, bsy/bo, gx, gy);
        send_opp(gx, gy);
    }

    n_pts = 0;
}
