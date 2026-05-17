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

// =============================================================================
//  KONFIGURACE
// =============================================================================

// UART k RBCX
#define UART_RBCX_RX    16
#define UART_RBCX_TX    17
#define UART_RBCX_BAUD  115200

// Časování
#define ZAPNOUT_NOUZOVY_NAVRAT false   // Změnit na true pro zapnutí hlídání času (nouzový návrat)
#define DELKA_ZAPASU_MS       180000   // 180 sekund (3 minuty)
#define CAS_NOUZOVEHO_NAVRATU 10000    // posledních 10s → nouzový návrat

// Rozměry robota (pro lajnovou navigaci)
#define SIRKA_ROBOTA_MM       300.0f   // šířka robota = šířka jedné lajny
#define DELKA_ROBOTA_MM       360.0f   // délka robota
#define BEZPECNA_VZDALENOST_ZDI    (NV_LIDAR_FROM_FRONT + 200.0f)  // Zvětšeno o 15cm
#define BEZPECNA_VZDALENOST_ZDIE_Y (NV_LIDAR_FROM_FRONT + 350.0f)  // Horní stěna (nájezd)
#define BEZPECNA_VZDALENOST_DOMOV_Y (NV_LIDAR_FROM_FRONT +  200.0f) // Spodní stěna (domov)
#define HOME_ZONA_MM          700.0f
#define MOZEK_HOME_X  (NV_ARENA_SIZE - HOME_ZONA_MM / 2.0f)
#define MOZEK_HOME_Y  (HOME_ZONA_MM / 2.0f)

// Vzdálenost soupeře pro vyhýbání (s hysterezí)
#define VZDALENOST_SOUPERE_STOP  500.0f  // mm — soupeř přímo v cestě → stop
#define VZDALENOST_SOUPERE_VOLNO 650.0f  // mm — soupeř pryč → rozjezd (+15 cm rezerva)
#define UHEL_SOUPERE_VPRED        45.0f  // ° — soupeř v tomto kuželu = "v cestě"
#define SOUPER_PRAH_LIDAR_MM    700.0f   // LiDAR boční: < tohle = soupeř tam je
#define SOUPER_PRAH_UZ_MM       400      // UZ zadní: < tohle = soupeř tam je

// Limity puků
#define PUKY_PLNY_ZASOBNIK  10   // kolik puků → jedem domů

// Dvoufázový dojezd (zpomalení před zdí)
#define ZPOMALENI_VZDALENOST_MM 200.0f
#define RYCHLOST_DOJEZDU        25
#define RYCHLOST_LAJNY         75
#define RYCHLOST_NAJEZDU       85

// Rezervy pro dynamickou jízdu (zvětšení vybírané plochy)
#define DYN_REZERVA_X_MM 150.0f   // o kolik se cílový úsek natáhne doleva a doprava
#define DYN_REZERVA_Y_MM 0.0f     // posun cílového úseku v ose Y (0 = jede přesně středem buňky)

// =============================================================================
//  KOMUNIKAČNÍ PROTOKOL (musí odpovídat RBCX main_final.cpp!)
// =============================================================================

// ESP32 → RBCX (3 bajty)
typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
    int16_t param2; // Přidáno pro cílovou vzdálenost (mm)
} EspCommand;

// RBCX → ESP32 (11 bajtů)
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t cmd_id;    // Přidáno: potvrzení odeslaného příkazu
    uint8_t buttons;
    int16_t pocet_puku;
    int16_t param;
    uint16_t uz1_mm;   // Ultrazvuk 1 (levý zadní) v mm
    uint16_t uz3_mm;   // Ultrazvuk 3 (pravý zadní) v mm
} RbcxStatus;

// Příkazy (ESP32 → RBCX)
enum CmdID : uint8_t {
    CMD_NOP             = 0x00,
    CMD_STOP            = 0x01,
    CMD_JED_SBIREJ      = 0x02,   // param = rychlost %
    CMD_OTOC_VLEVO      = 0x03,   // param = úhel °
    CMD_OTOC_VPRAVO     = 0x04,   // param = úhel °
    CMD_COUVEJ          = 0x05,   // param = vzdálenost mm
    CMD_VYLOZ           = 0x06,
    CMD_ZAVRI_ZASOBNIKY = 0x07,
    CMD_TOC_KONTINUALNE = 0x08,
    CMD_LIDAR_ERROR     = 0x09,
    CMD_OTEVRI_SOUPER   = 0x0A,
    CMD_ZAVRI_SOUPER    = 0x0B
};

// Statusy (RBCX → ESP32)
enum StatID : uint8_t {
    STAT_READY = 0x80,
    STAT_BUSY  = 0x81,
    STAT_DONE  = 0x82,
};

// =============================================================================
//  SENZOROVÁ DATA (z LiDARu, aktualizováno každý frame)
// =============================================================================

struct SenzoroveData {
    // Pozice robota
    float pozice_x;       // mm (0..ARENA_SIZE)
    float pozice_y;       // mm
    float heading;        // stupně (-180..180)

    // Navigace domů
    float domov_vzdalenost;   // mm
    float domov_uhel;         // stupně (absolutní hodnota)
    float domov_uhel_rel;     // stupně (se znaménkem, + = vpravo)
    char  domov_smer;         // 'L' nebo 'R'

    // Zdi / vzdálenosti
    float dist_vpredu;        // mm - vzdálenost od nárazníku vpřed (±15°)

    // Soupeř
    bool  souper_viden;
    float souper_x, souper_y;  // globální mm
    float souper_vzdalenost;   // mm
    float souper_uhel;         // stupně (absolutní)
    char  souper_smer;         // 'L' / 'R'
};

// =============================================================================
//  STAV RBCX (přijatý přes UART)
// =============================================================================

struct StavRbcx {
    bool    pripojeno;         // přijali jsme někdy platná data?
    uint8_t stav;              // STAT_READY / STAT_BUSY / STAT_DONE
    uint8_t cmd_id;            // Poslední příkaz, o kterém RBCX mluví
    bool    tlacitko_vpredu_up;
    bool    tlacitko_vpredu_down;
    bool    tlacitko_vlevo;
    bool    tlacitko_vpravo;
    int     pocet_puku;
    uint16_t uz1_mm;           // Ultrazvuk 1 (levý zadní) v mm
    uint16_t uz3_mm;           // Ultrazvuk 3 (pravý zadní) v mm
    unsigned long posledni_prijem;  // millis()
};

// =============================================================================
//  MAPA POKRYTÍ — pamatujeme kde už jsme byli
// =============================================================================
//
//  Hřiště rozdělíme na buňky o velikosti SIRKA_ROBOTA (300mm).
//  Každá buňka = true → už jsme tudy projeli.
//
//     Y ↑  [ ][ ][ ][ ][ ]   ← horní zeď (soupeřova)
//       |  [ ][ ][ ][ ][ ]
//       |  [ ][ ][ ][ ][ ]
//       |  [ ][ ][ ][ ][ ]
//       |  [ ][ ][ ][ ][ ]   ← spodní zeď (naše, HOME vpravo dole)
//       +-------------------→ X

#define POCET_BUNEK_X  10
#define POCET_BUNEK_Y  10
#define BUNKA_MM       (NV_ARENA_SIZE / 10.0f)

// =============================================================================
//  LAJNOVÁ NAVIGACE — počítání a řízení lajn
// =============================================================================

struct LajnovaNavigace {
    // Aktuální lajna
    int    cislo_lajny;          // 0 = horní, roste dolů k domovu
    bool   smer_doprava;         // true = jedeme v +X, false = jedeme v -X

    // Kde jsme začali a kam jedeme na aktuální lajně
    float  lajna_start_x;       // X kde jsme začali tuto lajnu
    float  lajna_cil_x;         // X kam chceme dojet (zeď nebo zkrácený cíl)

    // Kolikrát jsme projeli celé kolo (tam + zpět) — pro zkracování
    int    dokoncena_kola;       // kolik plných kol (tam-zpět) je hotovo

    // Celkový počet lajn co jsme projeli
    int    celkem_lajn;

    // Pozice Y středu každé lajny (vypočteno na začátku)
    float  lajna_y[10];          // max 10 lajn (pro arény do 3000mm)
    int    pocet_lajn;           // kolik lajn se vejde
};

// =============================================================================
//  STAVOVÝ AUTOMAT — české dlouhé názvy
// =============================================================================

enum StavRobota : uint8_t {
    STAV_CEKAM_NA_START,            // Čekáme na startovní signál
    STAV_NAJEZD_NAHORU,             // Vertikalí nájezd z HOME nahoru po pravé straně
    STAV_JEDU_LAJNU,                // Jedeme po lajně — sbíráme puky
    STAV_PRECHOD_NA_DALSI_LAJNU,    // Otáčíme se + posouváme na další lajnu
    STAV_VYHYBAM_SE_SOUPERI,        // Soupeř v cestě — vyhýbací manévr
    STAV_VRACIM_SE_DOMU,            // Navigace k domácí zóně
    STAV_VYKLADAM_PUKY,             // Vyložení puků doma
    STAV_NOUZOVY_NAVRAT,            // Čas končí — řítíme se domů
    STAV_PRESUN_Y,                  // Dynamický přesun na ose Y
    STAV_PRESUN_X,                  // Dynamický přesun na ose X
    STAV_VYHODNOT_DYNAMICKY_CIL,    // Výpočet dalšího cíle pro dynamickou jízdu
};

const char* jmeno_stavu(StavRobota s) {
    switch(s) {
        case STAV_CEKAM_NA_START:         return "CEKAM_NA_START";
        case STAV_NAJEZD_NAHORU:          return "NAJEZD_NAHORU";
        case STAV_JEDU_LAJNU:             return "JEDU_LAJNU";
        case STAV_PRECHOD_NA_DALSI_LAJNU: return "PRECHOD_LAJNY";
        case STAV_VYHYBAM_SE_SOUPERI:     return "VYHYBAM_SE";
        case STAV_VRACIM_SE_DOMU:         return "VRACIM_DOMU";
        case STAV_VYKLADAM_PUKY:          return "VYKLADAM";
        case STAV_NOUZOVY_NAVRAT:         return "NOUZOVY_NAVRAT";
        case STAV_PRESUN_Y:               return "PRESUN_Y";
        case STAV_PRESUN_X:               return "PRESUN_X";
        case STAV_VYHODNOT_DYNAMICKY_CIL: return "DALSI_DYNAM_CIL";
        default:                          return "???";
    }
}

// =============================================================================
//  GLOBÁLNÍ PROMĚNNÉ
// =============================================================================

static SenzoroveData  senzory;
static StavRbcx       rbcx;
static StavRobota     stav = STAV_CEKAM_NA_START;
static StavRobota     stav_po_vyhybani = STAV_JEDU_LAJNU;
static unsigned long  cas_startu = 0;    // millis() kdy zápas začal (0 = nezačal)

// Vyhýbání soupeři — 2-fázová detekce (LiDAR → UZ)
static char  vyh_strana = 'R';          // 'R' = soupeř vpravo, 'L' = soupeř vlevo
static bool  vyh_souper_viden = false;   // viděli jsme soupeře na boku?
static int   vyh_lidar_volno = 0;        // kolikrát LiDAR řekl "volno" za sebou
static bool  vyh_lidar_ok = false;       // LiDAR 2× potvrdil
static int   vyh_uz_volno = 0;           // kolikrát UZ řekl "volno" za sebou
static bool  vyh_uz_videl = false;       // UZ reálně soupeře spatřil (ochrana proti malým robotům)
static unsigned long vyh_lidar_ok_ms = 0; // Kdy LiDAR naposledy potvrdil volno
static float vyh_posledni_bok = -1.0f;   // pro detekci nových LiDAR dat

// Mapa pokrytí
static bool mapa_pokryti[POCET_BUNEK_X][POCET_BUNEK_Y];

// Lajnová navigace
static LajnovaNavigace navigace;

// Sub-krok pro vícekrokové manévry
static int krok = 0;

// Dynamický režim (2. jízda po vyložení)
static bool dynamicky_rezim = false;
static bool uz_vylozil = false;
static float dyn_start_x = 0, dyn_end_x = 0, dyn_y = 0;

// Časovače pro kroky (odpovídají _t_krok3 a _c_zbytek v simulátoru)
static unsigned long cas_krok_ms = 0;
static unsigned long vyklad_zbyva_ms = 1500;
static unsigned long cas_posledniho_prikazu = 0;
static uint8_t posledni_odeslany_prikaz = 0;
static int mozek_aktualni_rychlost = 0;


// =============================================================================
//  UART FUNKCE

//  Protokol: [0xAA] [0x55] [payload...] — matchuje robotka rkUartSend/Receive
// =============================================================================

#define SYNC0 0xAA
#define SYNC1 0x55

void mozek_uart_init() {
    Serial1.begin(UART_RBCX_BAUD, SERIAL_8N1, UART_RBCX_RX, UART_RBCX_TX);
    Serial.printf("[MOZEK] UART init: RX=%d TX=%d @ %d baud\n",
        UART_RBCX_RX, UART_RBCX_TX, UART_RBCX_BAUD);
}

void posli_prikaz(uint8_t cmd, int16_t param = 0, int16_t param2 = 0) {
    EspCommand c;
    c.cmd = cmd;
    c.param = param;
    c.param2 = param2;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
    cas_posledniho_prikazu = millis();
    posledni_odeslany_prikaz = cmd;
    
    // Resetuj sledovanou rychlost, pokud posíláme jiný příkaz než jízdu nebo korekci
    if (cmd != CMD_JED_SBIREJ && cmd != CMD_LIDAR_ERROR) {
        mozek_aktualni_rychlost = 0;
    }
    
    Serial.printf("[MOZEK] >>> CMD: 0x%02X  param=%d  param2=%d\n", cmd, param, param2);
}

// Stavový automat pro příjem (neblokující)
enum UartRxStav : uint8_t { RX_CEKAM_SYNC0, RX_CEKAM_SYNC1, RX_CITAM_DATA };
static UartRxStav  uart_rx_stav = RX_CEKAM_SYNC0;
static uint8_t     uart_rx_buf[sizeof(RbcxStatus)];
static size_t      uart_rx_pocet = 0;

bool prijmi_stav_rbcx() {
    bool prijato = false;
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        switch (uart_rx_stav) {
            case RX_CEKAM_SYNC0:
                if (c == SYNC0) uart_rx_stav = RX_CEKAM_SYNC1;
                break;
            case RX_CEKAM_SYNC1:
                if (c == SYNC1) { uart_rx_stav = RX_CITAM_DATA; uart_rx_pocet = 0; }
                else            { uart_rx_stav = (c == SYNC0) ? RX_CEKAM_SYNC1 : RX_CEKAM_SYNC0; }
                break;
            case RX_CITAM_DATA:
                uart_rx_buf[uart_rx_pocet++] = c;
                if (uart_rx_pocet >= sizeof(RbcxStatus)) {
                    RbcxStatus st;
                    memcpy(&st, uart_rx_buf, sizeof(st));
                    rbcx.pripojeno           = true;
                    rbcx.stav                = st.status;
                    rbcx.cmd_id              = st.cmd_id;
                    rbcx.tlacitko_vpredu_up  = (st.buttons >> 0) & 1;
                    rbcx.tlacitko_vpredu_down= (st.buttons >> 1) & 1;
                    rbcx.tlacitko_vlevo      = (st.buttons >> 2) & 1;
                    rbcx.tlacitko_vpravo     = (st.buttons >> 3) & 1;
                    rbcx.pocet_puku          = st.pocet_puku;
                    rbcx.uz1_mm              = st.uz1_mm;
                    rbcx.uz3_mm              = st.uz3_mm;
                    rbcx.posledni_prijem     = millis();
                    uart_rx_stav = RX_CEKAM_SYNC0;
                    prijato = true;
                }
                break;
        }
    }
    return prijato;
}

bool rbcx_hotovo() {
    if (millis() - cas_posledniho_prikazu < 100) return false;

    // Počkáme, dokud RBCX skutečně nezačne/neskončí dělat TO, co jsme po něm naposledy chtěli
    if (rbcx.cmd_id != posledni_odeslany_prikaz) return false;

    return rbcx.stav == STAT_DONE || rbcx.stav == STAT_READY;
}

bool souper_v_ceste() {
    return senzory.souper_viden
        && senzory.souper_vzdalenost < VZDALENOST_SOUPERE_STOP
        && senzory.souper_uhel < UHEL_SOUPERE_VPRED;
}

// Soupeř je bezpečně daleko (hystereze +15 cm) — lze se rozjet
bool souper_volno() {
    if (!senzory.souper_viden) return true;
    return senzory.souper_vzdalenost >= VZDALENOST_SOUPERE_VOLNO
        || senzory.souper_uhel >= UHEL_SOUPERE_VPRED;
}

// Podívá se "LiDARem" zadaným absolutním směrem, jestli tam není soupeř
bool souper_v_smeru(float target_heading_deg, float max_dist = VZDALENOST_SOUPERE_STOP, float fov = UHEL_SOUPERE_VPRED) {
    if (!senzory.souper_viden) return false;
    float dx = senzory.souper_x - senzory.pozice_x;
    float dy = senzory.souper_y - senzory.pozice_y;
    float vzd = sqrtf(dx*dx + dy*dy);
    if (vzd > max_dist) return false;
    float angle_sup = atan2f(dx, dy) * 180.0f / M_PI;
    float rel = angle_sup - target_heading_deg;
    while (rel > 180.0f) rel -= 360.0f;
    while (rel <= -180.0f) rel += 360.0f;
    return fabsf(rel) < fov;
}

bool naraz_vpredu() {
    return rbcx.tlacitko_vpredu_up || rbcx.tlacitko_vpredu_down;
}

// =============================================================================
//  AKTUALIZACE SENZOROVÝCH DAT (z nv_ proměnných lidar_no_viz.h)
// =============================================================================

void mozek_aktualizuj_senzory() {
    senzory.pozice_x  = constrain(nv_g_rx, 0, NV_ARENA_SIZE);
    senzory.pozice_y  = constrain(nv_g_ry, 0, NV_ARENA_SIZE);
    senzory.heading   = nv_g_h * 180.0f / PI;
    senzory.dist_vpredu = nv_dist_front;

    // Domov
    float dx = MOZEK_HOME_X - senzory.pozice_x;
    float dy = MOZEK_HOME_Y - senzory.pozice_y;
    senzory.domov_vzdalenost = sqrtf(dx*dx + dy*dy);
    float angle_home = atan2f(dx, dy);
    float rel = (angle_home - nv_g_h) * 180.0f / PI;
    while (rel >  180.0f) rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
    senzory.domov_uhel_rel = rel;
    senzory.domov_uhel = fabsf(rel);
    senzory.domov_smer = (rel >= 0) ? 'R' : 'L';

    // Soupeř
    senzory.souper_viden = nv_opp_valid;
    if (nv_opp_valid) {
        senzory.souper_x = nv_opp_gx;
        senzory.souper_y = nv_opp_gy;
        float dxo = nv_opp_gx - senzory.pozice_x;
        float dyo = nv_opp_gy - senzory.pozice_y;
        senzory.souper_vzdalenost = sqrtf(dxo*dxo + dyo*dyo);
        float ao = atan2f(dxo, dyo);
        float relo = (ao - nv_g_h) * 180.0f / PI;
        while (relo >  180.0f) relo -= 360.0f;
        while (relo < -180.0f) relo += 360.0f;
        senzory.souper_uhel = fabsf(relo);
        senzory.souper_smer = (relo >= 0) ? 'R' : 'L';
    }
}

// =============================================================================
//  MAPA POKRYTÍ — sledování kde jsme byli
// =============================================================================

// Převod mm souřadnic na index buňky
int bunka_x(float mm) { return constrain((int)(mm / BUNKA_MM), 0, POCET_BUNEK_X - 1); }
int bunka_y(float mm) { return constrain((int)(mm / BUNKA_MM), 0, POCET_BUNEK_Y - 1); }

// Označ aktuální pozici robota jako projetou
void aktualizuj_pokryti() {
    float r = SIRKA_ROBOTA_MM / 2.0f;
    for (float dx = -r; dx <= r; dx += (BUNKA_MM/2.0f)) {
        for (float dy = -r; dy <= r; dy += (BUNKA_MM/2.0f)) {
            int bx = bunka_x(senzory.pozice_x + dx);
            int by = bunka_y(senzory.pozice_y + dy);
            mapa_pokryti[bx][by] = true;
        }
    }
}

// Kolik buněk ještě nebylo navštíveno
int nepokrytych_bunek() {
    int n = 0;
    for (int x = 0; x < POCET_BUNEK_X; x++)
        for (int y = 0; y < POCET_BUNEK_Y; y++)
            if (!mapa_pokryti[x][y]) n++;
    return n;
}

// Najdi nejbližší neprojetou buňku (vrací střed buňky v mm)
bool najdi_nepokryte(float &cil_x, float &cil_y) {
    float nejblizsi = 1e9;
    bool nasel = false;
    for (int x = 0; x < POCET_BUNEK_X; x++) {
        for (int y = 0; y < POCET_BUNEK_Y; y++) {
            if (!mapa_pokryti[x][y]) {
                float cx = (x + 0.5f) * BUNKA_MM;
                float cy = (y + 0.5f) * BUNKA_MM;
                float d = sqrtf((cx - senzory.pozice_x)*(cx - senzory.pozice_x)
                              + (cy - senzory.pozice_y)*(cy - senzory.pozice_y));
                if (d < nejblizsi) {
                    nejblizsi = d;
                    cil_x = cx;
                    cil_y = cy;
                    nasel = true;
                }
            }
        }
    }
    return nasel;
}

// Najdi největší nevyčištěný úsek (odpovídá simulator.py: vypocti_dalsi_cil)
// Vrací true pokud něco našel, výsledek v out parametrech
bool vypocti_dalsi_cil(float &out_start_x, float &out_end_x, float &out_y, int &out_row) {
    int nej_span = 0;
    int nej_sx = -1, nej_ex = -1, nej_y_row = -1;

    // Ignorujeme vnější čtverečky u všech stěn (a nahoře 2 řádky), protože k nim robot nemá dobrý přístup
    for (int by = 1; by < POCET_BUNEK_Y - 2; by++) {
        int prvni_x = -1;
        int posledni_x = -1;
        
        for (int x_idx = 1; x_idx < POCET_BUNEK_X - 1; x_idx++) {
            if (!mapa_pokryti[x_idx][by]) {
                if (prvni_x == -1) {
                    prvni_x = x_idx;
                }
                posledni_x = x_idx;
            }
        }
        
        if (prvni_x != -1) {
            int span = posledni_x - prvni_x + 1;
            if (span > nej_span) {
                nej_span = span;
                nej_sx = prvni_x;
                nej_ex = posledni_x;
                nej_y_row = by;
            }
        }
    }

    if (nej_span == 0) return false;

    // Středy buněk → mm
    out_start_x = nej_sx * BUNKA_MM + BUNKA_MM / 2.0f;
    out_end_x   = nej_ex * BUNKA_MM + BUNKA_MM / 2.0f;
    out_y       = nej_y_row * BUNKA_MM + BUNKA_MM / 2.0f;
    out_row     = nej_y_row;

    // Aplikování rezerv (zvětšení cílového úseku)
    out_start_x -= DYN_REZERVA_X_MM;
    out_end_x   += DYN_REZERVA_X_MM;
    out_y       += DYN_REZERVA_Y_MM;

    // CLAMPING — robot nesmí jet blíž ke zdi
    out_start_x = constrain(out_start_x, BEZPECNA_VZDALENOST_ZDI, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI);
    out_end_x   = constrain(out_end_x,   BEZPECNA_VZDALENOST_ZDI, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI);
    out_y       = constrain(out_y,        BEZPECNA_VZDALENOST_DOMOV_Y, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDIE_Y);

    return true;
}

// Debug: vypiš mapu pokrytí do Serial (volat občas)
void vypis_mapu_pokryti() {
    Serial.println("[MAPA]");
    for (int y = POCET_BUNEK_Y - 1; y >= 0; y--) {
        Serial.print("  ");
        for (int x = 0; x < POCET_BUNEK_X; x++) {
            // Označ buňku kde je robot
            int rx = bunka_x(senzory.pozice_x);
            int ry = bunka_y(senzory.pozice_y);
            if (x == rx && y == ry) Serial.print("[R]");
            else if (mapa_pokryti[x][y]) Serial.print("[#]");
            else Serial.print("[ ]");
        }
        Serial.println();
    }
    Serial.printf("  Pokryto: %d/%d bunek\n",
        POCET_BUNEK_X * POCET_BUNEK_Y - nepokrytych_bunek(),
        POCET_BUNEK_X * POCET_BUNEK_Y);
}

// =============================================================================
//  LAJNOVÁ NAVIGACE — inicializace a řízení
// =============================================================================

// Spočítej pozice lajn (volá se na začátku)
void inicializuj_lajny() {
    // Lajny jdou shora dolů, každá má šířku SIRKA_ROBOTA
    // Lajna 0 = nahoře (soupeřova strana), poslední = dole (naše strana)
    // Robot nejdříve vyjede nahoru po pravé straně, pak zig-zaguje dolů.
    navigace.pocet_lajn = (int)(NV_ARENA_SIZE / SIRKA_ROBOTA_MM);
    for (int i = 0; i < navigace.pocet_lajn; i++) {
        // Střed lajny: od horní zdi dolů
        navigace.lajna_y[i] = NV_ARENA_SIZE - (i + 0.5f) * SIRKA_ROBOTA_MM;
    }

    // Začínáme od lajny 0 (nahoře), směr doleva
    // (robot přijel nahoru po pravé straně, první lajna jede doleva)
    navigace.cislo_lajny = 0;
    navigace.smer_doprava = false;
    navigace.celkem_lajn = 0;
    navigace.dokoncena_kola = 0;

    Serial.printf("[NAV] Inicializováno %d lajn (šířka %.0fmm)\n",
        navigace.pocet_lajn, SIRKA_ROBOTA_MM);
    for (int i = 0; i < navigace.pocet_lajn; i++) {
        Serial.printf("  Lajna %d: Y=%.0fmm\n", i, navigace.lajna_y[i]);
    }
}

// Nastav cíl aktuální lajny (kam má robot dojet na ose X)
void nastav_cil_lajny() {
    if (navigace.smer_doprava) {
        // Jedeme doprava (+X), cíl = pravá zeď (nebo zkrácený)
        navigace.lajna_start_x = senzory.pozice_x;
        navigace.lajna_cil_x = NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI;
    } else {
        // Jedeme doleva (-X), cíl = levá zeď (nebo zkrácený)
        navigace.lajna_start_x = senzory.pozice_x;
        navigace.lajna_cil_x = BEZPECNA_VZDALENOST_ZDI;
    }
}

// Dorazili jsme na konec aktuální lajny?
bool dosahli_konce_lajny() {
    if (navigace.smer_doprava) {
        return senzory.pozice_x >= navigace.lajna_cil_x;
    } else {
        return senzory.pozice_x <= navigace.lajna_cil_x;
    }
}

// Posun na další lajnu (aktualizuj číslo a směr)
// Ne-wrapujeme! Volající musí zkontrolovat cislo_lajny >= pocet_lajn
void dalsi_lajna() {
    navigace.cislo_lajny++;
    navigace.smer_doprava = !navigace.smer_doprava;
    navigace.celkem_lajn++;
}


// =============================================================================
//  ROZHODOVACÍ LOGIKA — STAVOVÝ AUTOMAT
// =============================================================================
//
//  Volání: mozek_rozhoduj() — voláno každý loop() frame (cca 20ms)
//
//  Každý stav může mít vnitřní pod-kroky (proměnná `krok`).
//  Při přechodu do nového stavu vždy nastavíme krok = 0.
//
//  Tok:
//    CEKAM_NA_START → JEDU_LAJNU ↔ PRECHOD_NA_DALSI_LAJNU
//                                ↔ VYHYBAM_SE_SOUPERI
//                    → VRACIM_SE_DOMU → VYKLADAM_PUKY → zpět na JEDU_LAJNU
//                    → NOUZOVY_NAVRAT (kdykoliv, pokud čas < 10s)
//



void zmen_stav(StavRobota novy) {
    Serial.printf("[MOZEK] STAV: %s → %s\n", jmeno_stavu(stav), jmeno_stavu(novy));
    if (novy == STAV_VYHYBAM_SE_SOUPERI) {
        stav_po_vyhybani = stav;
    }
    stav = novy;
    krok = 0;
    cas_krok_ms = millis();
}

// =============================================================================
//  POMOCNÉ FUNKCE PRO ROTACI A KOREKCI (LiDAR)
// =============================================================================

float vypocti_rozdil_uhlu(float cil, float aktualni) {
    float r = cil - aktualni;
    while (r > 180.0f) r -= 360.0f;
    while (r < -180.0f) r += 360.0f;
    return r;
}

float najdi_nejblizsi_rovnobezku(float aktualni_heading_deg) {
    float a = aktualni_heading_deg;
    while (a < 0) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    if (a >= 315 || a < 45) return 0.0f;
    if (a >= 45 && a < 135) return 90.0f;
    if (a >= 135 && a < 225) return 180.0f;
    return 270.0f;
}

void mozek_otoc_se_na(float target_deg) {
    while (target_deg < 0) target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;

    Serial.printf("[MOZEK] Blokujici rotace na %.1f°\n", target_deg);
    int16_t aktualni_rychlost = 0; 
    while (true) {
        loop_lidar_nv();
        mozek_aktualizuj_senzory();
        prijmi_stav_rbcx();
        
        float heading_deg = senzory.heading;
        float rozdil = vypocti_rozdil_uhlu(target_deg, heading_deg);
        
        if (fabs(rozdil) <= 2.5f) { // T_TOLERANCE_DEG
            posli_prikaz(CMD_STOP);
            Serial.printf("[MOZEK] Rotace dokoncena (%.1f°)\n", heading_deg);
            break; 
        }

        // Tříúrovňová rychlost otáčení (34%, 12%, 3%)
        int16_t pozadovana_rychlost = (rozdil > 0) ? 30 : -30;
        if (fabs(rozdil) <= 50.0f) {
            pozadovana_rychlost = (rozdil > 0) ? 12 : -12;
        }
        if (fabs(rozdil) <= 10.0f) {
            pozadovana_rychlost = (rozdil > 0) ? 3 : -3;
        }

        if (pozadovana_rychlost != aktualni_rychlost) {
            posli_prikaz(CMD_TOC_KONTINUALNE, pozadovana_rychlost);
            aktualni_rychlost = pozadovana_rychlost;
        }
        delay(5);
    }
}

void mozek_otoc_o_90(bool vlevo) {
    mozek_aktualizuj_senzory();
    float base = najdi_nejblizsi_rovnobezku(senzory.heading);
    mozek_otoc_se_na(base + (vlevo ? -90.0f : 90.0f));
}

void mozek_otoc_o_180() {
    mozek_aktualizuj_senzory();
    float base = najdi_nejblizsi_rovnobezku(senzory.heading);
    mozek_otoc_se_na(base + 180.0f);
}

void mozek_otoc_relativne(float uhel_deg) {
    mozek_aktualizuj_senzory();
    mozek_otoc_se_na(senzory.heading + uhel_deg);
}

void posli_korekci(int16_t param) {
    EspCommand c;
    c.cmd = CMD_LIDAR_ERROR;
    c.param = param;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
}

static float mozek_cilovy_uhel_jizdy = 0.0f;
static float mozek_startovni_y_prejezdu = 0.0f;
static float mozek_startovni_lidar_prejezdu = 0.0f;
static unsigned long mozek_posledni_lidar_error_ms = 0;

void mozek_start_jizdy(int rychlost, bool snap_to_grid = true) {
    mozek_aktualizuj_senzory();
    if (snap_to_grid) {
        mozek_cilovy_uhel_jizdy = najdi_nejblizsi_rovnobezku(senzory.heading);
    } else {
        mozek_cilovy_uhel_jizdy = senzory.heading;
    }
    posli_prikaz(CMD_JED_SBIREJ, rychlost);
    mozek_aktualni_rychlost = rychlost;
}


// Forward deklarace (definice je níže)
void mozek_start_zapasu();

// Zruší zbývající část aktuálního dynamického cíle, pokud narazíme do zdi
void zrus_aktualni_dynamicky_cil() {
    int start_bx = bunka_x(dyn_start_x);
    int end_bx = bunka_x(dyn_end_x);
    int by = bunka_y(dyn_y);
    if (start_bx > end_bx) {
        int tmp = start_bx; start_bx = end_bx; end_bx = tmp;
    }
    for (int bx = start_bx; bx <= end_bx; bx++) {
        mapa_pokryti[bx][by] = true;
    }
    Serial.printf("[MOZEK] Dynamický cíl (Y=%d, X=%d..%d) zrušen (nastaven jako pokrytý)\n", by, start_bx, end_bx);
}

void mozek_rozhoduj() {
    // Přijmi stav z RBCX (neblokující)
    prijmi_stav_rbcx();

    // Aktualizuj mapu pokrytí (pouze pokud už zápas běží)
    if (stav != STAV_CEKAM_NA_START) {
        aktualizuj_pokryti();
    }

    // Zbývající čas
    unsigned long ubehnuto = (cas_startu > 0) ? (millis() - cas_startu) : 0;
    unsigned long zbyva_ms = (ubehnuto < DELKA_ZAPASU_MS) ? (DELKA_ZAPASU_MS - ubehnuto) : 0;

    // Posílání korekcí během jízdy
    if (posledni_odeslany_prikaz == CMD_JED_SBIREJ && rbcx.stav == STAT_BUSY && cas_startu > 0) {
        if (millis() - mozek_posledni_lidar_error_ms > 30) {
            float rozdil = vypocti_rozdil_uhlu(mozek_cilovy_uhel_jizdy, senzory.heading);
            int16_t param_err = (int16_t)roundf(rozdil * 10.0f);
            posli_korekci(param_err);
            mozek_posledni_lidar_error_ms = millis();
        }
    }

    // ╔══════════════════════════════════════════════════════════╗
    // ║  NOUZOVÝ NÁVRAT — má nejvyšší prioritu, přeruší cokoliv ║
    // ╚══════════════════════════════════════════════════════════╝
    if (ZAPNOUT_NOUZOVY_NAVRAT && cas_startu > 0 && zbyva_ms < CAS_NOUZOVEHO_NAVRATU
        && stav != STAV_NOUZOVY_NAVRAT && stav != STAV_VYKLADAM_PUKY
        && !uz_vylozil) {
        Serial.println("[MOZEK] !!! ČAS KONČÍ — NOUZOVÝ NÁVRAT !!!");
        posli_prikaz(CMD_STOP);
        zmen_stav(STAV_NOUZOVY_NAVRAT);
    }

    // ╔══════════════════════════════════════════════════════════╗
    // ║  HLAVNÍ ROZHODOVÁNÍ                                     ║
    // ╚══════════════════════════════════════════════════════════╝
    switch (stav) {

    // ──────────────────────────────────────────────────────
    //  ČEKÁM NA START
    // ──────────────────────────────────────────────────────
    case STAV_CEKAM_NA_START: {
        // Automatický start dalších jízd, pokud už zápas běží a zbývá čas
        if (cas_startu > 0) {
            unsigned long zbyva_ms = 0;
            if (millis() - cas_startu < DELKA_ZAPASU_MS) {
                zbyva_ms = DELKA_ZAPASU_MS - (millis() - cas_startu);
            }
            if (zbyva_ms > CAS_NOUZOVEHO_NAVRATU) {
                Serial.println("[MOZEK] Automatický start další dynamické jízdy!");
                mozek_start_zapasu();
                break; // Nečekáme na tlačítko
            }
        }

        // Start se spouští tlačítkem UP na RBCX, ale až po jeho PUŠTĚNÍ a malé prodlevě
        static bool btn_up_predchozi = false;
        static unsigned long uvolneno_v_ms = 0;
        bool btn_up_nyni = rbcx.tlacitko_vpredu_up;
        
        // Detekce sestupné hrany (uvolnění)
        if (!btn_up_nyni && btn_up_predchozi && rbcx.pripojeno) {
            uvolneno_v_ms = millis();
            Serial.println("[MOZEK] Tlačítko UP uvolněno, odpočet 1s do startu...");
        }
        btn_up_predchozi = btn_up_nyni;

        // Odpočet 1s po puštění tlačítka (aby se eliminoval náraz při rozjezdu)
        if (uvolneno_v_ms > 0 && (millis() - uvolneno_v_ms > 1000)) {
            uvolneno_v_ms = 0;
            Serial.println("[MOZEK] STARTUJI ZÁPAS!");
            mozek_start_zapasu();
        }
        break;
    }

    // ──────────────────────────────────────────────────────
    //  NÁJEZD NAHORU
    // ──────────────────────────────────────────────────────
    case STAV_NAJEZD_NAHORU:
        switch (krok) {
            case 10: // Čekání na zavření našeho zásobníku
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_OTEVRI_SOUPER);
                    krok = 11;
                }
                break;
            case 11: // Čekání na otevření soupeřova zásobníku
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(RYCHLOST_NAJEZDU);
                    krok = 0;
                }
                break;
            case 0: {
                // [A] Plný zásobník
                if (rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK) {
                    Serial.printf("[MOZEK] Plný zásobník (%d puků) → DOMŮ\n", rbcx.pocet_puku);
                    posli_prikaz(CMD_STOP);
                    zmen_stav(STAV_VRACIM_SE_DOMU);
                    break;
                }
                // [B] Náraz vpředu
                if (naraz_vpredu()) {
                    Serial.println("[MOZEK] Náraz vpředu u nájezdu → couvám a pak doleva");
                    posli_prikaz(CMD_COUVEJ, 100); // Couvni 10cm
                    krok = 1;
                    break;
                }
                // [C] Soupeř v cestě
                if (souper_v_ceste()) {
                    Serial.printf("[MOZEK] Soupeř v cestě při nájezdu! → začínám lajny brzy\n");
                    posli_prikaz(CMD_STOP);
                    cas_krok_ms = millis();
                    krok = 1;
                    break;
                }

                // Pojistka z lidaru: jaká je fyzická vzdálenost nárazníku od zdi?
                float limit_dist_bumper_y = BEZPECNA_VZDALENOST_ZDIE_Y; 
                
                bool dojeli_pozice = (senzory.pozice_y >= NV_ARENA_SIZE - limit_dist_bumper_y);
                bool dojeli_lidar  = (senzory.dist_vpredu <= limit_dist_bumper_y);

                if (dojeli_pozice || dojeli_lidar) {
                    posli_prikaz(CMD_STOP);
                    Serial.printf("[MOZEK] Dosažen cíl nájezdu! (Pozice: %d, Lidar: %d)\n", dojeli_pozice, dojeli_lidar);
                    krok = 1;
                } else if (mozek_aktualni_rychlost > RYCHLOST_DOJEZDU) {
                    // Logika zpomalení před zdí
                    float zpomaleni_y = limit_dist_bumper_y + ZPOMALENI_VZDALENOST_MM;
                    bool blizko_pozice = (senzory.pozice_y >= NV_ARENA_SIZE - zpomaleni_y);
                    bool blizko_lidar  = (senzory.dist_vpredu <= zpomaleni_y);
                    if (blizko_pozice || blizko_lidar) {
                        Serial.println("[MOZEK] Zpomaluji u nájezdu...");
                        mozek_start_jizdy(RYCHLOST_DOJEZDU);
                    }
                }
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    mozek_otoc_o_90(true);
                    krok = 2;
                }
                break;
            case 2:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_ZAVRI_SOUPER);
                    krok = 3;
                }
                break;
            case 3:
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    mozek_start_jizdy(RYCHLOST_NAJEZDU);
                    Serial.println("[MOZEK] Nahoře! Lajna 0 → DOLEVA");
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  JEDU LAJNU
    // ──────────────────────────────────────────────────────
    case STAV_JEDU_LAJNU:
        // [A] Plný zásobník
        if (rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK) {
            Serial.printf("[MOZEK] Plný zásobník (%d puků) → DOMŮ\n", rbcx.pocet_puku);
            posli_prikaz(CMD_STOP);
            zmen_stav(STAV_VRACIM_SE_DOMU);
            break;
        }
        // [B] Náraz vpředu (tlačítka) → couvni a přejeď na další lajnu
        if (naraz_vpredu()) {
            Serial.println("[MOZEK] Náraz vpředu → couvám 10cm...");
            posli_prikaz(CMD_COUVEJ, 100);
            
            // Počkáme na dokončení couvání v novém sub-stavu
            if (dynamicky_rezim) {
                zrus_aktualni_dynamicky_cil();
                zmen_stav(STAV_VYHODNOT_DYNAMICKY_CIL);
            } else {
                if (senzory.pozice_y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0f))) {
                    zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
                } else {
                    zmen_stav(STAV_VRACIM_SE_DOMU);
                }
            }
            break;
        }

        // [C] Soupeř v cestě
        if (souper_v_ceste()) {
            Serial.printf("[MOZEK] Soupeř v cestě! dist=%.0f → VYHÝBÁM SE\n", senzory.souper_vzdalenost);
            posli_prikaz(CMD_STOP);
            zmen_stav(STAV_VYHYBAM_SE_SOUPERI);
            break;
        }

        // [D] Blízko protější zdi
        {
            bool limit_x = dosahli_konce_lajny();
            float limit_dist_bumper_x = BEZPECNA_VZDALENOST_ZDI;
            bool limit_lidar = (senzory.dist_vpredu <= limit_dist_bumper_x);

            if (limit_x || limit_lidar) {
                posli_prikaz(CMD_STOP);
                if (dynamicky_rezim) {
                    Serial.printf("[MOZEK] Konec dynamickeho useku (Pozice:%d, Lidar:%d) na Y=%.0f\n", limit_x, limit_lidar, senzory.pozice_y);
                    if (!limit_x) zrus_aktualni_dynamicky_cil();
                    zmen_stav(STAV_VYHODNOT_DYNAMICKY_CIL);
                } else {
                    if (senzory.pozice_y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0f))) {
                        Serial.printf("[MOZEK] Konec lajny (Pozice:%d, Lidar:%d) na Y=%.0f → PŘECHOD\n", limit_x, limit_lidar, senzory.pozice_y);
                        zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
                    } else {
                        Serial.printf("[MOZEK] Lajna na dně Y=%.0f hotová → VRACÍM SE DOMŮ\n", senzory.pozice_y);
                        zmen_stav(STAV_VRACIM_SE_DOMU);
                    }
                }
                break;
            } else if (mozek_aktualni_rychlost > RYCHLOST_DOJEZDU) {
                // Logika zpomalení před zdí
                float zpomaleni_x = limit_dist_bumper_x + ZPOMALENI_VZDALENOST_MM;
                bool blizko_lidar = (senzory.dist_vpredu <= zpomaleni_x);
                bool blizko_x = false;
                if (navigace.smer_doprava) {
                    blizko_x = senzory.pozice_x >= navigace.lajna_cil_x - ZPOMALENI_VZDALENOST_MM;
                } else {
                    blizko_x = senzory.pozice_x <= navigace.lajna_cil_x + ZPOMALENI_VZDALENOST_MM;
                }

                if (blizko_lidar || blizko_x) {
                    Serial.println("[MOZEK] Zpomaluji u zdi...");
                    mozek_start_jizdy(RYCHLOST_DOJEZDU);
                }
            }
        }
        break;

    // ──────────────────────────────────────────────────────
    //  PŘECHOD NA DALŠÍ LAJNU
    // ──────────────────────────────────────────────────────
    case STAV_PRECHOD_NA_DALSI_LAJNU:
        switch (krok) {
            case 0: {
                if (!rbcx_hotovo()) break; // Počkej na couvání

                if (souper_v_smeru(180.0f, 500.0f, 45.0f)) {
                    Serial.println("[MOZEK] Lidar vidí soupeře pod námi! Otáčím zpět.");
                    navigace.smer_doprava = !navigace.smer_doprava;
                    mozek_otoc_o_180();
                    krok = 10;
                    break;
                }
                // Už necouváme před otáčením (na přání uživatele) -> couváme teď automaticky při nárazu
                if (navigace.smer_doprava)
                    mozek_otoc_o_90(false);
                else
                    mozek_otoc_o_90(true);
                krok = 2; 
                break;
            }
            case 1:
                // Nepoužito (dříve čekání na couvnutí)
                break;
            case 2:
                if (rbcx_hotovo()) {
                    mozek_startovni_y_prejezdu = senzory.pozice_y;
                    mozek_startovni_lidar_prejezdu = senzory.dist_vpredu;
                    
                    bool vykladat_souper = false;
                    if (senzory.pozice_y >= 1000.0f) {
                        vykladat_souper = navigace.smer_doprava;  // Y >= 1000: vykládat na pravé stěně
                    } else {
                        vykladat_souper = !navigace.smer_doprava; // Y < 1000: vykládat na levé stěně (ochrana domovské zóny)
                    }

                    if (vykladat_souper) {
                        posli_prikaz(CMD_OTEVRI_SOUPER);
                        krok = 25; // Čekej na otevření
                    } else {
                        posli_prikaz(CMD_JED_SBIREJ, 25, 250); 
                        cas_krok_ms = millis(); 
                        krok = 3;
                    }
                }
                break;
            case 25: // Čekání na otevření zásobníku
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 25, 250); 
                    cas_krok_ms = millis(); 
                    krok = 3;
                }
                break;
            case 3: {
                // [A] Náraz (tlačítka) - MUSÍ BÝT PRVNÍ, protože náraz zastaví jízdu a rbcx_hotovo by pak bylo true
                if (naraz_vpredu()) {
                    Serial.println("[MOZEK] Prejezd PRERUSEN (Naraz!) -> couvám 10cm");
                    posli_prikaz(CMD_COUVEJ, 100);
                    krok = 4;
                    break;
                }

                // [B] Čekáme, až RBCX samo dojede do cíle (250mm)
                if (rbcx_hotovo()) {
                    float final_y = senzory.pozice_y;
                    float final_lidar = senzory.dist_vpredu;
                    Serial.printf("[MOZEK] Prejezd DOKONCEN (Enkodery): Lidar: %.1f, SLAM: %.1f\n", 
                        mozek_startovni_lidar_prejezdu - final_lidar, 
                        fabsf(mozek_startovni_y_prejezdu - final_y));
                    krok = 4;
                    break;
                }

                // [C] Soupeř v cestě
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Prejezd PRERUSEN (Souper!)");
                    krok = 4;
                    break;
                }

                // [D] Bezpečnostní pojistka u zdi (Lidar)
                if (senzory.dist_vpredu <= 300.0f) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Prejezd ZASTAVEN (ZED!)");
                    krok = 4;
                    break;
                }
                break;
            }
            case 4:  // Druhé otočení
                if (rbcx_hotovo()) {
                    float final_y = senzory.pozice_y;
                    float ujeto_total_slam = fabsf(mozek_startovni_y_prejezdu - final_y);
                    float ujeto_total_lidar = mozek_startovni_lidar_prejezdu - senzory.dist_vpredu;
                    Serial.printf("[MOZEK] Prejezd REALNE DOKONCEN: Ujeto LIDAR %.1f mm, SLAM %.1f mm (cil 200)\n", 
                        ujeto_total_lidar, ujeto_total_slam);

                    if (navigace.smer_doprava)
                        mozek_otoc_o_90(false);
                    else
                        mozek_otoc_o_90(true);
                    krok = 5;
                }
                break;
            case 5:  // Hotovo → nová lajna
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_ZAVRI_SOUPER);
                    krok = 6;
                }
                break;
            case 6:
                if (rbcx_hotovo()) {
                    dalsi_lajna();
                    nastav_cil_lajny();
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    Serial.printf("[MOZEK] Lajna %s\n", navigace.smer_doprava ? "→" : "←");
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
            case 10: // Únik zpět
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYHÝBÁM SE SOUPEŘI — 2-fázová detekce (LiDAR → UZ)
    //
    //  Logika:
    //    krok 0: Otoč se dolů, urči stranu soupeře
    //    krok 1: Jeď kolem, LiDAR 2× volno → UZ 2× volno
    //    krok 2: Dojezd 100mm rezervy
    //    krok 3: Otočit zpět na lajnu
    //    krok 4: Pokračuj v jízdě
    //    krok 10: Alternativní únik
    // ──────────────────────────────────────────────────────
    case STAV_VYHYBAM_SE_SOUPERI:
        switch (krok) {
            case 0: {
                // Určíme stranu: robot jede doprava → soupeř je vpravo → otočíme doleva (dolů)
                // Robot jede doleva → soupeř je vlevo → otočíme doprava (dolů)
                // Vždy se snažíme jet DOLŮ (směrem k naší zóně)
                bool ma_misto_dole = (senzory.pozice_y > BEZPECNA_VZDALENOST_ZDIE_Y + SIRKA_ROBOTA_MM);
                if (!ma_misto_dole) {
                    Serial.println("[MOZEK] Soupeř blokuje a nemám místo dolů! Otáčím zpět.");
                    mozek_otoc_o_180();
                    navigace.smer_doprava = !navigace.smer_doprava;
                    krok = 10;
                    break;
                }
                
                // Určíme stranu soupeře
                if (navigace.smer_doprava) {
                    vyh_strana = 'L';       // jeli jsme doprava, otočili doprava (dolů) → soupeř je vlevo
                    mozek_otoc_o_90(false); // otočíme doprava (dolů)
                } else {
                    vyh_strana = 'R';       // jeli jsme doleva, otočili doleva (dolů) → soupeř je vpravo
                    mozek_otoc_o_90(true);  // otočíme doleva (dolů)
                }
                
                Serial.printf("[MOZEK] Vyhýbání: soupeř %s, sleduju %s stranu\n",
                    vyh_strana == 'R' ? "VPRAVO" : "VLEVO",
                    vyh_strana == 'R' ? "pravou" : "levou");
                
                vyh_souper_viden = false;
                vyh_lidar_volno = 0;
                vyh_lidar_ok = false;
                vyh_uz_volno = 0;
                vyh_uz_videl = false;
                vyh_lidar_ok_ms = 0;
                vyh_posledni_bok = -1.0f;
                mozek_start_jizdy(25);  // pomalu kolem soupeře
                cas_krok_ms = millis();
                krok = 1;
                break;
            }
            
            case 1: { // Jedu kolem — 2-fázová detekce (LiDAR → UZ)
                float bok_lidar = (vyh_strana == 'R') ? nv_dist_right : nv_dist_left;
                uint16_t bok_uz = (vyh_strana == 'R') ? rbcx.uz3_mm : rbcx.uz1_mm;
                unsigned long cas_jizdy = millis() - cas_krok_ms;
                
                // Kontroluj jen při nové LiDAR hodnotě
                if (bok_lidar != vyh_posledni_bok) {
                    vyh_posledni_bok = bok_lidar;
                    
                    bool lidar_vidi = (bok_lidar < SOUPER_PRAH_LIDAR_MM);
                    bool uz_vidi = (bok_uz > 0 && bok_uz < SOUPER_PRAH_UZ_MM);
                    
                    Serial.printf("[MOZEK] LiDAR(%c):%4dmm(%s) UZ%c:%4dmm(%s) %s\n",
                        vyh_strana, (int)bok_lidar, lidar_vidi ? "OBJ" : " - ",
                        vyh_strana == 'R' ? '3' : '1', bok_uz, uz_vidi ? "OBJ" : " - ",
                        vyh_lidar_ok ? "[čekám UZ]" : "[čekám LiDAR]");
                    
                    // Nejdřív musíme soupeře VIDĚT
                    if (!vyh_souper_viden && (lidar_vidi || uz_vidi)) {
                        vyh_souper_viden = true;
                        Serial.printf("[MOZEK] *** Soupeř detekován na %s straně! ***\n",
                            vyh_strana == 'R' ? "pravé" : "levé");
                    }
                    
                    // Globální sledování, jestli UZ už soupeře viděl (kdykoliv během manévru)
                    if (uz_vidi && !vyh_uz_videl) {
                        vyh_uz_videl = true;
                        Serial.println("[MOZEK] *** UZ soupeře zaznamenal! ***");
                    }
                    
                    if (vyh_souper_viden) {
                        // FÁZE 1: LiDAR 2× volno
                        if (!vyh_lidar_ok) {
                            if (!lidar_vidi) {
                                vyh_lidar_volno++;
                                Serial.printf("[MOZEK]   → LiDAR volno %d/2\n", vyh_lidar_volno);
                            } else {
                                vyh_lidar_volno = 0;
                            }
                            if (vyh_lidar_volno >= 2) {
                                vyh_lidar_ok = true;
                                vyh_lidar_ok_ms = millis();
                                Serial.printf("[MOZEK] *** LiDAR potvrzeno → %s (čekám min 1.5s)\n",
                                    vyh_uz_videl ? "sleduju UZ" : "čekám, až ho UZ uvidí");
                            }
                        }
                        // FÁZE 2: UZ 2× volno (ale musí nejdřív soupeře reálně vidět a musí uplynout 1.5s!)
                        if (vyh_lidar_ok) {
                            if (!vyh_uz_videl) {
                                // Soupeř je mezi LiDARem a UZ, ještě k němu nedorazil
                                // Dál jedeme a čekáme, až ho UZ zaznamená
                            } else {
                                if (!uz_vidi) {
                                    if (millis() - vyh_lidar_ok_ms > 1500) {
                                        vyh_uz_volno++;
                                        Serial.printf("[MOZEK]   → UZ volno %d/2\n", vyh_uz_volno);
                                    }
                                } else {
                                    vyh_uz_volno = 0;
                                }
                                
                                if (vyh_uz_volno >= 2) {
                                    Serial.println("[MOZEK] *** SOUPEŘ PŘEJET! *** Popojíždím 100mm...");
                                    posli_prikaz(CMD_JED_SBIREJ, 25, 100); // 100mm rezerva
                                    krok = 2;
                                    break;
                                }
                            }
                        }
                    }
                }
                
                // Náraz vpředu
                if (naraz_vpredu()) {
                    posli_prikaz(CMD_COUVEJ, 100);
                    Serial.println("[MOZEK] Náraz při vyhýbání! Couvám a otáčím zpět.");
                    krok = 3;
                    break;
                }
                // Blízko spodní zdi
                if (senzory.dist_vpredu <= 300.0f || senzory.pozice_y <= BEZPECNA_VZDALENOST_DOMOV_Y) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Blízko zdi při vyhýbání → otáčím zpět.");
                    krok = 3;
                    break;
                }
                // Pojistka: max 8s
                if (cas_jizdy > 8000) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Timeout vyhýbání (8s) → otáčím zpět.");
                    krok = 3;
                    break;
                }
                break;
            }
            
            case 2: // Dojíždím 100mm rezervu
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Rezerva ujeta → otáčím zpět na lajnu.");
                    krok = 3;
                }
                break;
            
            case 3: // Otoč zpět na lajnu (ve STEJNÉM směru jako před vyhýbáním)
                if (rbcx_hotovo()) {
                    if (navigace.smer_doprava)
                        mozek_otoc_o_90(true);   // zpět doprava
                    else
                        mozek_otoc_o_90(false);  // zpět doleva
                    krok = 4;
                }
                break;
            case 4: // Rozjeď se a pokračuj
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    Serial.printf("[MOZEK] Pokračuji po lajně %s po vyhnutí\n", navigace.smer_doprava ? "→" : "←");
                    zmen_stav(stav_po_vyhybani);
                }
                break;
            case 10: // Alternativní únik (nemůže dolů)
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VRACÍM SE DOMŮ (stejné jako nouzový návrat)
    // ──────────────────────────────────────────────────────
    case STAV_VRACIM_SE_DOMU:
        switch (krok) {
            case 0: {
                if (!dynamicky_rezim) {
                    bool v_home_x = senzory.pozice_x >= NV_ARENA_SIZE - HOME_ZONA_MM;
                    bool v_home_y = senzory.pozice_y <= HOME_ZONA_MM;
                    if (v_home_x && v_home_y) {
                        Serial.println("[MOZEK] První jízda - jsem v HOME zóně, přeskakuji navigaci na bod a jdu rovnou vykládat.");
                        krok = 3;
                        break;
                    }
                }

                // Vypočteme si absolutní úhel k domovu
                float dx = MOZEK_HOME_X - senzory.pozice_x;
                float dy = MOZEK_HOME_Y - senzory.pozice_y;
                float angle_home_deg = atan2f(dx, dy) * 180.0f / PI;
                
                mozek_otoc_se_na(angle_home_deg);
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Vyhazuji soupeřovy puky před návratem...");
                    posli_prikaz(CMD_OTEVRI_SOUPER);
                    krok = 15;
                }
                break;
            case 15:
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(90, false); // nezarovnávat na mřížku os
                    krok = 2;
                }
                break;
            case 2:
                if (senzory.domov_vzdalenost < 150.0f) {
                    posli_prikaz(CMD_STOP);
                    krok = 3;
                } else if (senzory.dist_vpredu < 200.0f) {
                    Serial.println("[MOZEK] Zastaveni kvuli zdi pri navratu domu!");
                    posli_prikaz(CMD_STOP);
                    krok = 3;
                }
                break;
            case 3:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Navrat domu - zaviram souperuv zasobnik...");
                    posli_prikaz(CMD_ZAVRI_SOUPER);
                    krok = 13;
                }
                break;
            case 13:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Navrat domu - natoceni na vykladaci uhel (0°)...");
                    mozek_otoc_se_na(0.0f);
                    krok = 4;
                }
                break;
            case 4:
                if (rbcx_hotovo()) {
                    zmen_stav(STAV_VYKLADAM_PUKY);
                    krok = 30; // Přeskoč počáteční otáčení a jdi rovnou na otevírání zásobníků
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYKLÁDÁM PUKY
    // ──────────────────────────────────────────────────────
    case STAV_VYKLADAM_PUKY:
        switch (krok) {
            case 30: {
                // Kontrola: jsme opravdu v HOME zóně?
                bool v_home_x = senzory.pozice_x >= NV_ARENA_SIZE - HOME_ZONA_MM;
                bool v_home_y = senzory.pozice_y <= HOME_ZONA_MM;
                if (!(v_home_x && v_home_y)) {
                    Serial.printf("[MOZEK] VAROVANI: Asi nejsem úplně v HOME! (%.0f,%.0f), ale vykládám.\n", senzory.pozice_x, senzory.pozice_y);
                }
                Serial.println("[MOZEK] Otevírám zásobníky...");
                posli_prikaz(CMD_VYLOZ);
                krok = 31;
                break;
            }
            case 31:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Zásobníky otevřeny. Popojíždím 30 cm...");
                    mozek_start_jizdy(40);
                    cas_krok_ms = millis();
                    vyklad_zbyva_ms = 1500;
                    krok = 40;
                }
                break;

            case 40:
                if (naraz_vpredu()) {
                    Serial.println("[MOZEK] Náraz při popojíždění u vykládky! Místo už není, končím vykládání.");
                    posli_prikaz(CMD_STOP);
                    krok = 50;
                    break;
                }
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    vyklad_zbyva_ms -= (millis() - cas_krok_ms);
                    Serial.println("[MOZEK] Soupeř při vykládání! Čekám...");
                    krok = 45;
                } else if (cas_krok_ms > 0 && millis() - cas_krok_ms >= vyklad_zbyva_ms) {
                    posli_prikaz(CMD_STOP);
                    krok = 50;
                }
                break;

            case 45:
                if (souper_volno()) {
                    Serial.println("[MOZEK] Soupeř pryč, pokračuji ve vykládání.");
                    mozek_start_jizdy(40);
                    cas_krok_ms = millis();
                    krok = 40;
                }
                break;

            case 50:
                Serial.println("[MOZEK] Zavírám naše zásobníky...");
                posli_prikaz(CMD_ZAVRI_ZASOBNIKY);
                krok = 51;
                break;

            case 51:
                if (rbcx_hotovo()) {
                    Serial.printf("[MOZEK] Puky vyloženy! Pokryto %d/%d\n",
                        POCET_BUNEK_X * POCET_BUNEK_Y - nepokrytych_bunek(),
                        POCET_BUNEK_X * POCET_BUNEK_Y);
                    krok = 60;
                }
                break;

            case 60:
                Serial.println("[MOZEK] ═══ PŘIPRAVENA NA DALŠÍ KOLO ═══");
                uz_vylozil = true;
                zmen_stav(STAV_CEKAM_NA_START);
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  NOUZOVÝ NÁVRAT (1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_NOUZOVY_NAVRAT:
        switch (krok) {
            case 0: {
                // Vypočteme si absolutní úhel k domovu
                float dx = MOZEK_HOME_X - senzory.pozice_x;
                float dy = MOZEK_HOME_Y - senzory.pozice_y;
                float angle_home_deg = atan2f(dx, dy) * 180.0f / PI;
                
                mozek_otoc_se_na(angle_home_deg);
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Vyhazuji soupeřovy puky před nouzovým návratem...");
                    posli_prikaz(CMD_OTEVRI_SOUPER);
                    krok = 15;
                }
                break;
            case 15:
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(90, false); // nezarovnávat na mřížku os
                    krok = 2;
                }
                break;
            case 2:
                if (senzory.domov_vzdalenost < 150.0f) {
                    posli_prikaz(CMD_STOP);
                    krok = 3;
                } else if (senzory.dist_vpredu < 200.0f) {
                    Serial.println("[MOZEK] Nouzove zastaveni kvuli zdi pri navratu!");
                    posli_prikaz(CMD_STOP);
                    krok = 3;
                }
                break;
            case 3:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Nouzovy navrat - zaviram souperuv zasobnik...");
                    posli_prikaz(CMD_ZAVRI_SOUPER);
                    krok = 13;
                }
                break;
            case 13:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Nouzovy navrat - natoceni na vykladaci uhel (0°)...");
                    mozek_otoc_se_na(0.0f);
                    krok = 4;
                }
                break;
            case 4:
                if (rbcx_hotovo()) {
                    zmen_stav(STAV_VYKLADAM_PUKY);
                    krok = 30; // Přeskoč počáteční otáčení a jdi rovnou na otevírání zásobníků
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  PŘESUN Y — dynamický režim (1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_PRESUN_Y:
        switch (krok) {
            case 0: {
                float dy = dyn_y - senzory.pozice_y;
                if (fabsf(dy) < 30.0f) {
                    krok = 3;
                    break;
                }
                float tar_h = (dy < 0) ? 180.0f : 0.0f;
                float h_err = senzory.heading - tar_h;
                while (h_err > 180.0f) h_err -= 360.0f;
                while (h_err <= -180.0f) h_err += 360.0f;
                Serial.printf("[MOZEK] Přesun Y: cíl=%.0f, aktuální=%.0f\n", dyn_y, senzory.pozice_y);
                if (fabsf(h_err) > 3.0f) {
                    if (h_err > 0) mozek_otoc_relativne(-(fabsf(h_err)));
                    else mozek_otoc_relativne(fabsf(h_err));
                }
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    krok = 2;
                }
                break;
            case 2: {
                if (naraz_vpredu()) {
                    Serial.println("[MOZEK] Náraz při přesunu Y! Ruším cíl a couvám 10cm...");
                    posli_prikaz(CMD_COUVEJ, 100);
                    zrus_aktualni_dynamicky_cil();
                    zmen_stav(STAV_VYHODNOT_DYNAMICKY_CIL);
                    break;
                }
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Soupeř v cestě (PRESUN_Y)! Čekám...");
                    krok = 20;
                    break;
                }
                float dy = dyn_y - senzory.pozice_y;
                if (fabsf(dy) <= 25.0f) {
                    posli_prikaz(CMD_STOP);
                    Serial.printf("[MOZEK] Dosaženo Y=%.0f\n", senzory.pozice_y);
                    krok = 3;
                } else if (mozek_aktualni_rychlost > RYCHLOST_DOJEZDU) {
                    if (fabsf(dy) <= 25.0f + ZPOMALENI_VZDALENOST_MM) {
                        Serial.println("[MOZEK] Zpomaluji u přesunu Y...");
                        mozek_start_jizdy(RYCHLOST_DOJEZDU);
                    }
                }
                break;
            }
            case 20:
                if (souper_volno()) {
                    Serial.println("[MOZEK] Cesta Y volná, pokračuji.");
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    krok = 2;
                }
                break;
            case 3:
                if (rbcx_hotovo()) {
                    zmen_stav(STAV_PRESUN_X);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  PŘESUN X — dynamický režim (1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_PRESUN_X:
        switch (krok) {
            case 0: {
                float d_start = fabsf(dyn_start_x - senzory.pozice_x);
                float d_end = fabsf(dyn_end_x - senzory.pozice_x);
                float target_x;
                if (d_start < d_end) {
                    target_x = dyn_start_x;
                    navigace.smer_doprava = true;
                    navigace.lajna_cil_x = dyn_end_x;
                } else {
                    target_x = dyn_end_x;
                    navigace.smer_doprava = false;
                    navigace.lajna_cil_x = dyn_start_x;
                }
                float dx = target_x - senzory.pozice_x;
                Serial.printf("[MOZEK] Přesun X: cíl=%.0f, aktuální=%.0f\n", target_x, senzory.pozice_x);
                if (fabsf(dx) < 30.0f) {
                    krok = 3;
                    break;
                }
                float tar_h = (dx > 0) ? 90.0f : -90.0f;
                float h_err = senzory.heading - tar_h;
                while (h_err > 180.0f) h_err -= 360.0f;
                while (h_err <= -180.0f) h_err += 360.0f;
                if (fabsf(h_err) > 3.0f) {
                    if (h_err > 0) mozek_otoc_relativne(-(fabsf(h_err)));
                    else mozek_otoc_relativne(fabsf(h_err));
                }
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    krok = 2;
                }
                break;
            case 2: {
                if (naraz_vpredu()) {
                    Serial.println("[MOZEK] Náraz při přesunu X! Ruším cíl a couvám 10cm...");
                    posli_prikaz(CMD_COUVEJ, 100);
                    zrus_aktualni_dynamicky_cil();
                    zmen_stav(STAV_VYHODNOT_DYNAMICKY_CIL);
                    break;
                }
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Soupeř v cestě (PRESUN_X)! Čekám...");
                    krok = 21;
                    break;
                }
                float t_x = navigace.smer_doprava ? dyn_start_x : dyn_end_x;
                float dx = t_x - senzory.pozice_x;
                if (fabsf(dx) <= 25.0f) {
                    posli_prikaz(CMD_STOP);
                    Serial.printf("[MOZEK] Dosaženo X=%.0f, startuji čištění k %.0f\n", senzory.pozice_x, navigace.lajna_cil_x);
                    krok = 3;
                } else if (mozek_aktualni_rychlost > RYCHLOST_DOJEZDU) {
                    if (fabsf(dx) <= 25.0f + ZPOMALENI_VZDALENOST_MM) {
                        Serial.println("[MOZEK] Zpomaluji u přesunu X...");
                        mozek_start_jizdy(RYCHLOST_DOJEZDU);
                    }
                }
                break;
            }
            case 21:
                if (souper_volno()) {
                    Serial.println("[MOZEK] Cesta X volná, pokračuji.");
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    krok = 2;
                }
                break;
            case 3: {
                if (rbcx_hotovo()) {
                    float tar_h = navigace.smer_doprava ? 90.0f : -90.0f;
                    float h_err = senzory.heading - tar_h;
                    while (h_err > 180.0f) h_err -= 360.0f;
                    while (h_err <= -180.0f) h_err += 360.0f;
                    if (fabsf(h_err) > 3.0f) {
                        if (h_err > 0) mozek_otoc_relativne(-(fabsf(h_err)));
                        else mozek_otoc_relativne(fabsf(h_err));
                    }
                    krok = 4;
                }
                break;
            }
            case 4:
                if (rbcx_hotovo()) {
                    mozek_start_jizdy(RYCHLOST_LAJNY);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VÝPOČET DALŠÍHO DYNAMICKÉHO CÍLE
    // ──────────────────────────────────────────────────────
    case STAV_VYHODNOT_DYNAMICKY_CIL:
        if (rbcx_hotovo()) {
            int dummy_row;
            if (vypocti_dalsi_cil(dyn_start_x, dyn_end_x, dyn_y, dummy_row)) {
                Serial.printf("[MOZEK] ═══ DALŠÍ DYNAMICKÝ ÚSEK ═══\n");
                Serial.printf("[MOZEK] Úsek: Y=%.0f, X=%.0f až %.0f\n", dyn_y, dyn_start_x, dyn_end_x);
                zmen_stav(STAV_PRESUN_Y);
            } else {
                Serial.println("[MOZEK] Mapa vyčištěna! Jedeme domů.");
                zmen_stav(STAV_VRACIM_SE_DOMU);
            }
        }
        break;

    } // switch (stav)

    // ╔══════════════════════════════════════════════════════════╗
    // ║  DEBUG VÝPIS — 1× za sekundu                            ║
    // ╚══════════════════════════════════════════════════════════╝
    static unsigned long posledni_debug = 0;
    static unsigned long posledni_mapa = 0;
    if (millis() - posledni_debug > 1000) {
        posledni_debug = millis();
        Serial.printf("[MOZEK] %s k=%d | t=%lus | RBCX:%s puky=%d | POS(%d,%d) H=%d° | FRONT %dmm (pts:%d) | HOME %dmm %d°%c | L%d/%d %s",
            jmeno_stavu(stav),
            krok,
            zbyva_ms / 1000,
            rbcx.pripojeno ? (rbcx_hotovo() ? "RDY" : "BSY") : "---",
            rbcx.pocet_puku,
            (int)senzory.pozice_x, (int)senzory.pozice_y,
            (int)senzory.heading,
            (int)senzory.dist_vpredu,
            nv_acc_front_count,
            (int)senzory.domov_vzdalenost,
            (int)senzory.domov_uhel,
            senzory.domov_smer,
            navigace.cislo_lajny, navigace.pocet_lajn,
            navigace.smer_doprava ? "→" : "←");
        if (senzory.souper_viden) {
            Serial.printf(" | SOU %dmm %d°%c",
                (int)senzory.souper_vzdalenost,
                (int)senzory.souper_uhel,
                senzory.souper_smer);
        }
        Serial.println();
    }

    // Vypiš mapu pokrytí každých 10s
    if (millis() - posledni_mapa > 10000) {
        posledni_mapa = millis();
        vypis_mapu_pokryti();
    }
}

// =============================================================================
//  START ZÁPASU (voláno zvenku — tlačítkem, UART, atd.)
// =============================================================================

void mozek_start_zapasu() {
    if (stav != STAV_CEKAM_NA_START) return;

    if (cas_startu == 0) {
        // ═══ PRVNÍ START ═══
        memset(mapa_pokryti, 0, sizeof(mapa_pokryti)); // Vymazání mapy (robot se mohl hýbat před startem)
        cas_startu = millis();
        inicializuj_lajny();
        dynamicky_rezim = false;
        uz_vylozil = false;
        Serial.println("[MOZEK] ═══ ZÁPAS ZAHÁJEN — INICIALIZACE ZÁSOBNÍKŮ ═══");
        posli_prikaz(CMD_ZAVRI_ZASOBNIKY);
        zmen_stav(STAV_NAJEZD_NAHORU);
        krok = 10; // Krok pro úvodní zavření našeho zásobníku
    } else {
        // ═══ DRUHÝ A DALŠÍ START (dynamický režim) ═══
        dynamicky_rezim = true;
        int dummy_row;
        if (vypocti_dalsi_cil(dyn_start_x, dyn_end_x, dyn_y, dummy_row)) {
            Serial.printf("[MOZEK] ═══ DRUHÁ JÍZDA (DYNAMICKÁ) ═══\n");
            Serial.printf("[MOZEK] Úsek: Y=%.0f, X=%.0f až %.0f\n", dyn_y, dyn_start_x, dyn_end_x);
            zmen_stav(STAV_PRESUN_Y);
        } else {
            Serial.println("[MOZEK] Mapa už přejetá! Aréna čistá.");
            cas_startu = 0; // Ukončí zápas
        }
    }
}

// =============================================================================
//  INIT & UPDATE
// =============================================================================

void mozek_init() {
    mozek_uart_init();
    memset(&senzory, 0, sizeof(senzory));
    memset(&rbcx, 0, sizeof(rbcx));
    memset(mapa_pokryti, 0, sizeof(mapa_pokryti));
    senzory.domov_smer = 'R';
    senzory.souper_smer = 'R';
    stav = STAV_CEKAM_NA_START;
    krok = 0;
    cas_startu = 0;
    dynamicky_rezim = false;
    uz_vylozil = false;
    cas_krok_ms = 0;
    vyklad_zbyva_ms = 1500;
    inicializuj_lajny();
    Serial.println("[MOZEK] === MOZEK READY === čekám na start...");
}

void mozek_update() {
    mozek_aktualizuj_senzory();
    mozek_rozhoduj();
}
