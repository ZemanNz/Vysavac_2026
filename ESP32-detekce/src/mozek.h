#pragma once
#include <Arduino.h>

// ============================================================
//  mozek.h — Rozhodovací logika ESP32 (Master)
//
//  Čte data z LiDARu (nv_ proměnné z lidar_no_viz.h),
//  komunikuje přes UART (Serial1) s RBCX,
//  rozhoduje co robot dělá.
//
//  Tento soubor MUSÍ být includován PO lidar_no_viz.h
//  (potřebuje nv_g_rx, nv_g_ry, nv_g_h, nv_opp_* atd.)
// ============================================================

// === UART konfigurace pro komunikaci s RBCX ===
#define UART_RBCX_RX    16
#define UART_RBCX_TX    17
#define UART_RBCX_BAUD  115200

// === Délka zápasu ===
#define DELKA_ZAPASU_MS  90000   // 90 sekund

// =============================================================================
//  KOMUNIKAČNÍ PROTOKOL (musí odpovídat RBCX main_final.cpp!)
// =============================================================================

// ESP32 → RBCX (3 bajty)
typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
} EspCommand;

// RBCX → ESP32 (6 bajtů)
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t buttons;
    int16_t pocet_puku;
    int16_t param;
} RbcxStatus;

// Příkazy (ESP32 → RBCX)
enum CmdID : uint8_t {
    CMD_NOP           = 0x00,   // Nic nedělej (jen pošli stav)
    CMD_STOP          = 0x01,   // Zastav vše
    CMD_JED_SBIREJ    = 0x02,   // Jeď dopředu + sbírej (param = rychlost %)
    CMD_OTOC_VLEVO    = 0x03,   // Otoč se doleva (param = úhel °)
    CMD_OTOC_VPRAVO   = 0x04,   // Otoč se doprava (param = úhel °)
    CMD_COUVEJ        = 0x05,   // Couvej (param = vzdálenost mm)
    CMD_VYLOZ         = 0x06,   // Vyložení puků
};

// Statusy (RBCX → ESP32)
enum StatID : uint8_t {
    STAT_READY = 0x80,
    STAT_BUSY  = 0x81,
    STAT_DONE  = 0x82,
};

// =============================================================================
//  SENZOROVÁ DATA (plněná z lidar_no_viz.h každý frame)
// =============================================================================

struct SensorData {
    // --- Pozice robota ---
    float pos_x;         // mm (0..ARENA_SIZE)
    float pos_y;         // mm
    float heading;       // stupně (-180..180)

    // --- Navigace domů ---
    float home_dist;     // mm vzdálenost k domovu
    float home_angle;    // stupně (absolutní hodnota)
    char  home_dir;      // 'L' = otoč doleva, 'R' = doprava

    // --- Soupeř ---
    bool  opp_valid;     // vidíme soupeře?
    float opp_x, opp_y;  // globální pozice (mm)
    float opp_dist;      // mm vzdálenost
    float opp_angle;     // stupně (absolutní hodnota)
    char  opp_dir;       // 'L' / 'R'
};

// =============================================================================
//  STAV RBCX (přijatý přes UART)
// =============================================================================

struct RbcxData {
    bool    connected;       // přijali jsme někdy platná data?
    uint8_t status;          // STAT_READY / STAT_BUSY / STAT_DONE
    bool    btn_up;          // přední tlačítko UP (náraz)
    bool    btn_down;        // přední tlačítko DOWN (náraz)
    bool    btn_left;        // boční LEFT
    bool    btn_right;       // boční RIGHT
    int     pocet_puku;      // kolik puků naší barvy RBCX nasbíral
    unsigned long last_rx;   // millis() posledního příjmu
};

// =============================================================================
//  STAVOVÝ AUTOMAT
// =============================================================================

enum RobotState : uint8_t {
    STATE_WAIT_START,    // čekáme na startovní signál
    STATE_ALIGN,         // zarovnání podle headingu ke zdi
    STATE_SEARCH,        // lajnová strategie — sbíráme puky
    STATE_RETURN,        // navigace domů
    STATE_DUMP,          // vyložení puků
    STATE_EMERGENCY,     // čas končí — okamžitě domů
};

// =============================================================================
//  GLOBÁLNÍ PROMĚNNÉ (přístupné z main.cpp i odsud)
// =============================================================================

static SensorData senzory;
static RbcxData   rbcx;
static RobotState stav = STATE_WAIT_START;
static unsigned long cas_startu = 0;  // millis() kdy zápas začal (0 = ještě nezačal)

// =============================================================================
//  UART FUNKCE
//  Protokol na drátě (musí matchovat robotka rkUartSend/Receive):
//    [0xAA] [0x55] [payload...]
//  Žádný checksum — sync bajty + raw data.
//  rkUartSend na RBCX posílá: 0xAA 0x55 + sizeof(RbcxStatus) = 8 bajtů
//  rkUartReceive na RBCX čeká: 0xAA 0x55 + sizeof(EspCommand)  = 5 bajtů
// =============================================================================

#define SYNC0 0xAA
#define SYNC1 0x55

void mozek_uart_init() {
    Serial1.begin(UART_RBCX_BAUD, SERIAL_8N1, UART_RBCX_RX, UART_RBCX_TX);
    Serial.printf("[MOZEK] UART init: RX=%d TX=%d @ %d baud\n",
        UART_RBCX_RX, UART_RBCX_TX, UART_RBCX_BAUD);
}

// Odeslání příkazu na RBCX (s 0xAA 0x55 hlavičkou)
void posli_prikaz(uint8_t cmd, int16_t param = 0) {
    EspCommand c;
    c.cmd = cmd;
    c.param = param;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
    Serial.printf("[MOZEK] >>> CMD: 0x%02X  param=%d\n", cmd, param);
}

// --- Stavový automat pro příjem (neblokující, volat často) ---
enum UartRxState : uint8_t { RX_WAIT_SYNC0, RX_WAIT_SYNC1, RX_READ_PAYLOAD };
static UartRxState uart_rx_state = RX_WAIT_SYNC0;
static uint8_t     uart_rx_buf[sizeof(RbcxStatus)];
static size_t      uart_rx_count = 0;

bool prijmi_stav_rbcx() {
    while (Serial1.available()) {
        uint8_t c = Serial1.read();

        switch (uart_rx_state) {
            case RX_WAIT_SYNC0:
                if (c == SYNC0) uart_rx_state = RX_WAIT_SYNC1;
                break;

            case RX_WAIT_SYNC1:
                if (c == SYNC1) {
                    uart_rx_state = RX_READ_PAYLOAD;
                    uart_rx_count = 0;
                } else {
                    uart_rx_state = (c == SYNC0) ? RX_WAIT_SYNC1 : RX_WAIT_SYNC0;
                }
                break;

            case RX_READ_PAYLOAD:
                uart_rx_buf[uart_rx_count++] = c;
                if (uart_rx_count >= sizeof(RbcxStatus)) {
                    // Kompletní paket!
                    RbcxStatus st;
                    memcpy(&st, uart_rx_buf, sizeof(st));

                    rbcx.connected  = true;
                    rbcx.status     = st.status;
                    rbcx.btn_up     = (st.buttons >> 0) & 1;
                    rbcx.btn_down   = (st.buttons >> 1) & 1;
                    rbcx.btn_left   = (st.buttons >> 2) & 1;
                    rbcx.btn_right  = (st.buttons >> 3) & 1;
                    rbcx.pocet_puku = st.pocet_puku;
                    rbcx.last_rx    = millis();

                    uart_rx_state = RX_WAIT_SYNC0;
                    return true;
                }
                break;
        }
    }
    return false;
}


// Je RBCX připravený na nový příkaz?
bool rbcx_hotovo() {
    return rbcx.status == STAT_DONE || rbcx.status == STAT_READY;
}

// =============================================================================
//  AKTUALIZACE SENZOROVÝCH DAT (přepočet z nv_ proměnných)
// =============================================================================

void mozek_aktualizuj_senzory() {
    // --- Pozice & heading ---
    senzory.pos_x   = constrain(nv_g_rx, 0, NV_ARENA_SIZE);
    senzory.pos_y   = constrain(nv_g_ry, 0, NV_ARENA_SIZE);
    senzory.heading = nv_g_h * 180.0f / PI;

    // --- Navigace domů ---
    float dx = HOME_X - senzory.pos_x;
    float dy = HOME_Y - senzory.pos_y;
    senzory.home_dist = sqrtf(dx*dx + dy*dy);

    float angle_home = atan2f(dx, dy);
    float rel = (angle_home - nv_g_h) * 180.0f / PI;
    while (rel >  180.0f) rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
    senzory.home_angle = fabsf(rel);
    senzory.home_dir   = (rel >= 0) ? 'R' : 'L';

    // --- Soupeř ---
    senzory.opp_valid = nv_opp_valid;
    if (nv_opp_valid) {
        senzory.opp_x = nv_opp_gx;
        senzory.opp_y = nv_opp_gy;
        float dxo = nv_opp_gx - senzory.pos_x;
        float dyo = nv_opp_gy - senzory.pos_y;
        senzory.opp_dist = sqrtf(dxo*dxo + dyo*dyo);
        float ao = atan2f(dxo, dyo);
        float relo = (ao - nv_g_h) * 180.0f / PI;
        while (relo >  180.0f) relo -= 360.0f;
        while (relo < -180.0f) relo += 360.0f;
        senzory.opp_angle = fabsf(relo);
        senzory.opp_dir   = (relo >= 0) ? 'R' : 'L';
    }
}

// =============================================================================
//  JMÉNA STAVŮ (pro debug výpis)
// =============================================================================

const char* stav_jmeno(RobotState s) {
    switch(s) {
        case STATE_WAIT_START: return "WAIT";
        case STATE_ALIGN:      return "ALIGN";
        case STATE_SEARCH:     return "SEARCH";
        case STATE_RETURN:     return "RETURN";
        case STATE_DUMP:       return "DUMP";
        case STATE_EMERGENCY:  return "EMERG";
        default:               return "???";
    }
}

// =============================================================================
//  STAVOVÝ AUTOMAT — ROZHODOVACÍ LOGIKA
// =============================================================================

void mozek_rozhoduj() {
    // Přijmi stav z RBCX (neblokující)
    prijmi_stav_rbcx();

    // Zbývající čas zápasu
    unsigned long ubehnuto = (cas_startu > 0) ? (millis() - cas_startu) : 0;
    unsigned long zbyva_ms = (ubehnuto < DELKA_ZAPASU_MS) ? (DELKA_ZAPASU_MS - ubehnuto) : 0;

    // === EMERGENCY: pokud zbývá < 10s → okamžitě domů ===
    if (cas_startu > 0 && zbyva_ms < 10000
        && stav != STATE_RETURN && stav != STATE_DUMP && stav != STATE_EMERGENCY) {
        Serial.println("[MOZEK] !!! EMERGENCY — čas končí !!!");
        stav = STATE_EMERGENCY;
    }

    // === Hlavní rozhodování podle stavu ===
    switch (stav) {

        case STATE_WAIT_START:
            // TODO: Čekání na startovní signál (tlačítko / UART / časovač)
            // Až bude signál:
            //   cas_startu = millis();
            //   stav = STATE_ALIGN;
            break;

        case STATE_ALIGN:
            // TODO: Zarovnání headingu rovnoběžně se zdí
            // Pomalá jízda + sledování heading → ±1.5° od 0/90/180/270
            // Po zarovnání → stav = STATE_SEARCH
            break;

        case STATE_SEARCH:
            // TODO: Lajnová strategie
            // CMD_JED_SBIREJ(60) → sleduj SLAM pozici
            // Na konci lajny → STOP, otoč 90°, posuň, otoč 90°, nová lajna
            //
            // Kontroly:
            //   - Soupeř < 30cm → STOP + vyhýbací manévr
            //   - Náraz (btn_up/btn_down) → COUVEJ + otoč
            //   - pocet_puku >= 5 → STATE_RETURN
            break;

        case STATE_RETURN:
            // TODO: Navigace domů
            // 1) Otoč se směrem k domovu (senzory.home_dir + home_angle)
            // 2) CMD_JED_SBIREJ(80) — rychle domů
            // 3) Až home_dist < 100mm → STOP → STATE_DUMP
            break;

        case STATE_DUMP:
            // TODO: Vyložení puků
            // CMD_VYLOZ → čekat na STAT_DONE → STATE_SEARCH
            break;

        case STATE_EMERGENCY:
            // TODO: Čas končí — emergency návrat
            // Stejné jako RETURN, ale bez zastavování
            break;
    }

    // === DEBUG VÝPIS (1× za sekundu) ===
    static unsigned long posledni_debug = 0;
    if (millis() - posledni_debug > 1000) {
        posledni_debug = millis();
        Serial.printf("[MOZEK] %s | t=%lus | RBCX:%s puky=%d | POS(%d,%d) H=%d° | HOME %dmm %d°%c",
            stav_jmeno(stav),
            zbyva_ms / 1000,
            rbcx.connected ? (rbcx_hotovo() ? "RDY" : "BSY") : "---",
            rbcx.pocet_puku,
            (int)senzory.pos_x, (int)senzory.pos_y,
            (int)senzory.heading,
            (int)senzory.home_dist,
            (int)senzory.home_angle,
            senzory.home_dir);
        if (senzory.opp_valid) {
            Serial.printf(" | OPP %dmm %d°%c",
                (int)senzory.opp_dist,
                (int)senzory.opp_angle,
                senzory.opp_dir);
        }
        Serial.println();
    }
}

// ============================================================================= 
//  INIT & UPDATE (volat z main.cpp)
// =============================================================================

void mozek_init() {
    mozek_uart_init();
    memset(&senzory, 0, sizeof(senzory));
    memset(&rbcx, 0, sizeof(rbcx));
    senzory.home_dir = 'R';
    senzory.opp_dir  = 'R';
    stav = STATE_WAIT_START;
    cas_startu = 0;
    Serial.println("[MOZEK] === MOZEK READY === čekám na start...");
}

// Volat KAŽDÝ loop() — po loop_lidar_nv()
void mozek_update() {
    mozek_aktualizuj_senzory();
    mozek_rozhoduj();
}
