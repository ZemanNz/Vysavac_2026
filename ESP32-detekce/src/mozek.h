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
#define DELKA_ZAPASU_MS       180000   // 180 sekund (3 minuty)
#define CAS_NOUZOVEHO_NAVRATU 10000    // posledních 10s → nouzový návrat

// Rozměry robota (pro lajnovou navigaci)
#define SIRKA_ROBOTA_MM       300.0f   // šířka robota = šířka jedné lajny
#define DELKA_ROBOTA_MM       360.0f   // délka robota
#define BEZPECNA_VZDALENOST_ZDI    (SIRKA_ROBOTA_MM / 2.0f + 350.0f)  // Zvětšeno o 15cm
#define BEZPECNA_VZDALENOST_ZDIE_Y (DELKA_ROBOTA_MM / 2.0f + 350.0f)  // Zvětšeno o 15cm
#define HOME_ZONA_MM          700.0f
#define MOZEK_HOME_X  (NV_ARENA_SIZE - HOME_ZONA_MM / 2.0f)
#define MOZEK_HOME_Y  (HOME_ZONA_MM / 2.0f)

// Vzdálenost soupeře pro vyhýbání (s hysterezí)
#define VZDALENOST_SOUPERE_STOP  500.0f  // mm — soupeř přímo v cestě → stop
#define VZDALENOST_SOUPERE_VOLNO 650.0f  // mm — soupeř pryč → rozjezd (+15 cm rezerva)
#define UHEL_SOUPERE_VPRED        45.0f  // ° — soupeř v tomto kuželu = "v cestě"

// Limity puků
#define PUKY_PLNY_ZASOBNIK  10   // kolik puků → jedem domů

// =============================================================================
//  KOMUNIKAČNÍ PROTOKOL (musí odpovídat RBCX main_final.cpp!)
// =============================================================================

// ESP32 → RBCX (3 bajty)
typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
} EspCommand;

// RBCX → ESP32 (7 bajtů)
typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t cmd_id;    // Přidáno: potvrzení odeslaného příkazu
    uint8_t buttons;
    int16_t pocet_puku;
    int16_t param;
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

void posli_prikaz(uint8_t cmd, int16_t param = 0) {
    EspCommand c;
    c.cmd = cmd;
    c.param = param;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
    cas_posledniho_prikazu = millis();
    posledni_odeslany_prikaz = cmd;
    Serial.printf("[MOZEK] >>> CMD: 0x%02X  param=%d\n", cmd, param);
}

// Stavový automat pro příjem (neblokující)
enum UartRxStav : uint8_t { RX_CEKAM_SYNC0, RX_CEKAM_SYNC1, RX_CITAM_DATA };
static UartRxStav  uart_rx_stav = RX_CEKAM_SYNC0;
static uint8_t     uart_rx_buf[sizeof(RbcxStatus)];
static size_t      uart_rx_pocet = 0;

bool prijmi_stav_rbcx() {
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
                    rbcx.posledni_prijem     = millis();
                    uart_rx_stav = RX_CEKAM_SYNC0;
                    return true;
                }
                break;
        }
    }
    return false;
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
    int nej_delka = 0;
    int nej_sx = -1, nej_ex = -1, nej_y_row = -1;

    for (int by = 0; by < POCET_BUNEK_Y; by++) {
        int delka_sekvence = 0;
        int start_x_idx = -1;
        
        for (int x_idx = 0; x_idx < POCET_BUNEK_X; x_idx++) {
            if (!mapa_pokryti[x_idx][by]) {
                if (delka_sekvence == 0) start_x_idx = x_idx;
                delka_sekvence++;
            } else {
                if (delka_sekvence > nej_delka) {
                    nej_delka = delka_sekvence;
                    nej_sx = start_x_idx;
                    nej_ex = x_idx - 1;
                    nej_y_row = by;
                }
                delka_sekvence = 0;
            }
        }
        // Dotažení sekvence na konci řádku
        if (delka_sekvence > nej_delka) {
            nej_delka = delka_sekvence;
            nej_sx = start_x_idx;
            nej_ex = POCET_BUNEK_X - 1;
            nej_y_row = by;
        }
    }

    if (nej_delka == 0) return false;

    // Středy buněk → mm
    out_start_x = nej_sx * BUNKA_MM + BUNKA_MM / 2.0f;
    out_end_x   = nej_ex * BUNKA_MM + BUNKA_MM / 2.0f;
    out_y       = nej_y_row * BUNKA_MM + BUNKA_MM / 2.0f;
    out_row     = nej_y_row;

    // CLAMPING — robot nesmí jet blíž ke zdi
    out_start_x = constrain(out_start_x, BEZPECNA_VZDALENOST_ZDI, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI);
    out_end_x   = constrain(out_end_x,   BEZPECNA_VZDALENOST_ZDI, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI);
    out_y       = constrain(out_y,        BEZPECNA_VZDALENOST_ZDIE_Y, NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDIE_Y);

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

// Forward deklarace (definice je níže)
void mozek_start_zapasu();

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

    // ╔══════════════════════════════════════════════════════════╗
    // ║  NOUZOVÝ NÁVRAT — má nejvyšší prioritu, přeruší cokoliv ║
    // ╚══════════════════════════════════════════════════════════╝
    if (cas_startu > 0 && zbyva_ms < CAS_NOUZOVEHO_NAVRATU
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
            case 0: {
                // [A] Plný zásobník
                if (rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK) {
                    Serial.printf("[MOZEK] Plný zásobník (%d puků) → DOMŮ\n", rbcx.pocet_puku);
                    posli_prikaz(CMD_STOP);
                    zmen_stav(STAV_VRACIM_SE_DOMU);
                    break;
                }
                // [B] Soupeř v cestě
                if (souper_v_ceste()) {
                    Serial.printf("[MOZEK] Soupeř v cestě při nájezdu! → začínám lajny brzy\n");
                    posli_prikaz(CMD_STOP);
                    posli_prikaz(CMD_OTOC_VLEVO, 90);
                    cas_krok_ms = millis();
                    krok = 1;
                    break;
                }
                // [C] Náraz vpředu
                if (naraz_vpredu()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Náraz vpředu u nájezdu → jdu rovnou doleva");
                    posli_prikaz(CMD_OTOC_VLEVO, 90);
                    cas_krok_ms = millis();
                    krok = 1;
                    break;
                }

                // Pojistka z lidaru: jaká je fyzická vzdálenost nárazníku od zdi?
                float limit_dist_bumper_y = BEZPECNA_VZDALENOST_ZDIE_Y - (DELKA_ROBOTA_MM / 2.0f); 
                
                bool dojeli_pozice = (senzory.pozice_y >= navigace.lajna_y[0] - SIRKA_ROBOTA_MM / 2.0f);
                bool dojeli_lidar  = (senzory.dist_vpredu <= limit_dist_bumper_y);

                if (dojeli_pozice || dojeli_lidar) {
                    posli_prikaz(CMD_STOP);
                    Serial.printf("[MOZEK] Dosažen cíl nájezdu! (Pozice: %d, Lidar: %d)\n", dojeli_pozice, dojeli_lidar);
                    krok = 1;
                }
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    delay(3000);
                    posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 2;
                }
                break;
            case 2:
                if (rbcx_hotovo()) {
                    delay(3000);
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
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
        // [B] Soupeř v cestě
        if (souper_v_ceste()) {
            Serial.printf("[MOZEK] Soupeř v cestě! dist=%.0f → VYHÝBÁM SE\n", senzory.souper_vzdalenost);
            posli_prikaz(CMD_STOP);
            zmen_stav(STAV_VYHYBAM_SE_SOUPERI);
            break;
        }

        // [C] Náraz vpředu (tlačítka) → couvni a přejeď na další lajnu
        if (naraz_vpredu()) {
            posli_prikaz(CMD_STOP);
            if (senzory.pozice_y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0f))) {
                Serial.println("[MOZEK] Náraz vpředu → couvám + další lajna");
                zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
            } else {
                Serial.println("[MOZEK] Náraz zcela dole → VYKLÁDÁM");
                zmen_stav(STAV_VYKLADAM_PUKY);
            }
            cas_krok_ms = millis();
            break;
        }

        // [D] Blízko protější zdi
        {
            bool limit_x = dosahli_konce_lajny();
            float limit_dist_bumper_x = BEZPECNA_VZDALENOST_ZDI - (DELKA_ROBOTA_MM / 2.0f);
            bool limit_lidar = (senzory.dist_vpredu <= limit_dist_bumper_x);

            if (limit_x || limit_lidar) {
                posli_prikaz(CMD_STOP);
                if (senzory.pozice_y > (SIRKA_ROBOTA_MM + (SIRKA_ROBOTA_MM / 2.0f))) {
                    Serial.printf("[MOZEK] Konec lajny (Pozice:%d, Lidar:%d) na Y=%.0f → PŘECHOD\n", limit_x, limit_lidar, senzory.pozice_y);
                    zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
                } else {
                    Serial.printf("[MOZEK] Lajna na dně Y=%.0f hotová → VYKLÁDÁM\n", senzory.pozice_y);
                    zmen_stav(STAV_VYKLADAM_PUKY);
                }
                cas_krok_ms = millis();
                break;
            }
        }
        break;

    // ──────────────────────────────────────────────────────
    //  PŘECHOD NA DALŠÍ LAJNU
    // ──────────────────────────────────────────────────────
    case STAV_PRECHOD_NA_DALSI_LAJNU:
        switch (krok) {
            case 0: {
                if (souper_v_smeru(180.0f, 500.0f, 45.0f)) {
                    Serial.println("[MOZEK] Lidar vidí soupeře pod námi! Otáčím zpět.");
                    navigace.smer_doprava = !navigace.smer_doprava;
                    posli_prikaz(CMD_OTOC_VLEVO, 180);
                    krok = 10;
                    break;
                }
                posli_prikaz(CMD_COUVEJ, 100);
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    delay(3000);
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 2;
                }
                break;
            case 2:
                if (rbcx_hotovo()) {
                    delay(3000);
                    posli_prikaz(CMD_JED_SBIREJ, 40);
                    cas_krok_ms = millis(); // Nutné pro měření času jízdy dolů (case 3)
                    krok = 3;
                }
                break;
            case 3: {
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    navigace.smer_doprava = !navigace.smer_doprava;
                    krok = 10;
                    break;
                }
                unsigned long cas_dolu = millis() - cas_krok_ms;
                bool jsem_dole = (senzory.pozice_y <= BEZPECNA_VZDALENOST_ZDIE_Y + SIRKA_ROBOTA_MM);
                if (cas_krok_ms > 0 && cas_dolu > 2000) {
                    float tar_h = navigace.smer_doprava ? -90.0f : 90.0f;
                    bool volno = !souper_v_smeru(tar_h, 600.0f, 45.0f);
                    if (volno || jsem_dole || cas_dolu > 8000) {
                        posli_prikaz(CMD_STOP);
                        cas_krok_ms = 0;
                        krok = 4;
                    }
                }
                break;
            }
            case 4:  // Druhé otočení
                if (rbcx_hotovo()) {
                    delay(3000);
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 5;
                }
                break;
            case 5:  // Hotovo → nová lajna
                if (rbcx_hotovo()) {
                    delay(3000);
                    dalsi_lajna();
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    Serial.printf("[MOZEK] Lajna %s\n", navigace.smer_doprava ? "→" : "←");
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
            case 10: // Únik zpět
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYHÝBÁM SE SOUPEŘI (1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_VYHYBAM_SE_SOUPERI:
        switch (krok) {
            case 0: {
                bool ma_misto_dole = (senzory.pozice_y > BEZPECNA_VZDALENOST_ZDIE_Y + SIRKA_ROBOTA_MM);
                if (!ma_misto_dole) {
                    // U spodní stěny — nemůžeme dolů → otočíme se a jedeme zpět
                    Serial.println("[MOZEK] Soupeř blokuje a nemám místo dolů! Otáčím zpět.");
                    posli_prikaz(CMD_OTOC_VLEVO, 180);  // Robot míří po lajně, 180° otočí
                    navigace.smer_doprava = !navigace.smer_doprava;
                    krok = 10;
                    break;
                }
                // Zkontroluj LiDARem dolů
                if (souper_v_smeru(180.0f, 500.0f, 45.0f)) {
                    Serial.println("[MOZEK] Lidar vidí soupeře pod námi! Vracím se starou lajnou.");
                    navigace.smer_doprava = !navigace.smer_doprava;
                    posli_prikaz(CMD_OTOC_VLEVO, 180);
                    krok = 10;
                    break;
                }
                // Natočení dolů
                if (navigace.smer_doprava)
                    posli_prikaz(CMD_OTOC_VPRAVO, 90);
                else
                    posli_prikaz(CMD_OTOC_VLEVO, 90);
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 40);
                    cas_krok_ms = millis();
                    krok = 2;
                }
                break;
            case 2: {
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Soupeř se připletl do úhybu! Vracím se.");
                    // Robot míří DOLŮ (180°). Otoč 90° zpět na lajnu:
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);  // 180°→-90° (LEFT)
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);   // 180°→90° (RIGHT)
                    navigace.smer_doprava = !navigace.smer_doprava;
                    krok = 10;
                    break;
                }
                unsigned long cas_dolu_v = millis() - cas_krok_ms;
                if (cas_krok_ms > 0 && cas_dolu_v > 1500) {
                    float tar_h = navigace.smer_doprava ? 90.0f : -90.0f;
                    if (!souper_v_smeru(tar_h, VZDALENOST_SOUPERE_VOLNO, 45.0f)) {
                        posli_prikaz(CMD_STOP);
                        krok = 3;
                    } else if (cas_dolu_v > 6000) {
                        posli_prikaz(CMD_STOP);
                        krok = 3;
                    }
                }
                break;
            }
            case 3:
                if (rbcx_hotovo()) {
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    krok = 4;
                }
                break;
            case 4:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(stav_po_vyhybani);
                }
                break;
            case 10: // Alternativní únik
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VRACÍM SE DOMŮ (couvání, 1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_VRACIM_SE_DOMU:
        switch (krok) {
            case 0: { // Otoč se ZÁDY k domovu
                float rel_zacouvani = senzory.domov_uhel_rel - 180.0f;
                while (rel_zacouvani > 180.0f) rel_zacouvani -= 360.0f;
                while (rel_zacouvani <= -180.0f) rel_zacouvani += 360.0f;
                if (fabsf(rel_zacouvani) > 10.0f) {
                    int16_t uhel = (int16_t)fabsf(rel_zacouvani);
                    if (rel_zacouvani >= 0)
                        posli_prikaz(CMD_OTOC_VPRAVO, uhel);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, uhel);
                    krok = 1;
                } else {
                    krok = 2;
                }
                break;
            }
            case 1:
                if (rbcx_hotovo()) krok = 2;
                break;
            case 2:
                posli_prikaz(CMD_COUVEJ, (int16_t)senzory.domov_vzdalenost);
                krok = 3;
                break;
            case 3: {
                if (senzory.domov_vzdalenost < 150.0f) {
                    Serial.println("[MOZEK] Jsme zacouvani doma!");
                    posli_prikaz(CMD_STOP);
                    zmen_stav(STAV_VYKLADAM_PUKY);
                    krok = 21;
                } else {
                    float rel = senzory.domov_uhel_rel - 180.0f;
                    while (rel > 180.0f) rel -= 360.0f;
                    while (rel <= -180.0f) rel += 360.0f;
                    if (fabsf(rel) > 20.0f) {
                        posli_prikaz(CMD_STOP);
                        krok = 0;  // znovu zamiř a couvej
                    }
                }
                break;
            }
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYKLÁDÁM PUKY (1:1 se simulátorem)
    // ──────────────────────────────────────────────────────
    case STAV_VYKLADAM_PUKY:
        switch (krok) {
            case 0:  // Urči cestu
                if (!navigace.smer_doprava) {
                    Serial.println("[MOZEK] Vyklad: cesta A (z levé strany)");
                    posli_prikaz(CMD_OTOC_VPRAVO, 180);
                    krok = 10;
                } else {
                    Serial.println("[MOZEK] Vyklad: cesta B (z pravé strany)");
                    krok = 20;
                }
                break;

            // === Cesta A: Z levé strany ===
            case 10:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    krok = 11;
                }
                break;
            case 11:  // Jedeme do HOME, hlídáme soupeře
                if (souper_v_ceste()) {
                    posli_prikaz(CMD_STOP);
                    Serial.println("[MOZEK] Soupeř blokuje cestu domů! Čekám...");
                    krok = 12;
                } else if (senzory.pozice_x >= NV_ARENA_SIZE - BEZPECNA_VZDALENOST_ZDI) {
                    posli_prikaz(CMD_STOP);
                    krok = 20;
                }
                break;
            case 12:  // Čekáme na uvolnění cesty
                if (souper_volno()) {
                    Serial.println("[MOZEK] Cesta volná, pokračuji domů.");
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    krok = 11;
                }
                break;

            // === Společná fáze: Natočení nahoru a dump ===
            case 20:
                posli_prikaz(CMD_OTOC_VLEVO, 90);
                krok = 21;
                break;
            case 21:
                if (rbcx_hotovo()) {
                    float h_err = senzory.heading;
                    if (fabsf(h_err) > 0.5f) {
                        Serial.println("[MOZEK] Srovnávám orientaci...");
                        if (h_err > 0)
                            posli_prikaz(CMD_OTOC_VLEVO, (int16_t)h_err);
                        else
                            posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)fabsf(h_err));
                        krok = 22;
                    } else {
                        krok = 30;
                    }
                }
                break;
            case 22:
                if (rbcx_hotovo()) krok = 30;
                break;

            case 30: {
                // Kontrola: jsme opravdu v HOME zóně?
                bool v_home_x = senzory.pozice_x >= NV_ARENA_SIZE - HOME_ZONA_MM;
                bool v_home_y = senzory.pozice_y <= HOME_ZONA_MM;
                if (!(v_home_x && v_home_y)) {
                    Serial.printf("[MOZEK] Nejsem v HOME! (%.0f,%.0f) → DOMŮ\n", senzory.pozice_x, senzory.pozice_y);
                    zmen_stav(STAV_VRACIM_SE_DOMU);
                    break;
                }
                Serial.println("[MOZEK] Otevírám zásobníky...");
                posli_prikaz(CMD_VYLOZ);
                krok = 31;
                break;
            }
            case 31:
                if (rbcx_hotovo()) {
                    Serial.println("[MOZEK] Zásobníky otevřeny. Popojíždím 30 cm...");
                    posli_prikaz(CMD_JED_SBIREJ, 40);
                    cas_krok_ms = millis();
                    vyklad_zbyva_ms = 1500;
                    krok = 40;
                }
                break;

            case 40:
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
                    posli_prikaz(CMD_JED_SBIREJ, 40);
                    cas_krok_ms = millis();
                    krok = 40;
                }
                break;

            case 50:
                Serial.println("[MOZEK] Zavírám zásobníky...");
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
            case 0:
                if (senzory.domov_uhel > 10.0f) {
                    if (senzory.domov_smer == 'L')
                        posli_prikaz(CMD_OTOC_VLEVO, (int16_t)senzory.domov_uhel);
                    else
                        posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)senzory.domov_uhel);
                    krok = 1;
                } else {
                    krok = 2;
                }
                break;
            case 1:
                if (rbcx_hotovo()) krok = 2;
                break;
            case 2:
                posli_prikaz(CMD_JED_SBIREJ, 90);
                krok = 3;
                break;
            case 3:
                if (senzory.domov_vzdalenost < 150.0f) {
                    posli_prikaz(CMD_STOP);
                    zmen_stav(STAV_VYKLADAM_PUKY);
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
                    if (h_err > 0) posli_prikaz(CMD_OTOC_VLEVO, (int16_t)fabsf(h_err));
                    else posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)fabsf(h_err));
                }
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    krok = 2;
                }
                break;
            case 2: {
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
                }
                break;
            }
            case 20:
                if (souper_volno()) {
                    Serial.println("[MOZEK] Cesta Y volná, pokračuji.");
                    posli_prikaz(CMD_JED_SBIREJ, 60);
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
                    if (h_err > 0) posli_prikaz(CMD_OTOC_VLEVO, (int16_t)fabsf(h_err));
                    else posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)fabsf(h_err));
                }
                krok = 1;
                break;
            }
            case 1:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    krok = 2;
                }
                break;
            case 2: {
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
                }
                break;
            }
            case 21:
                if (souper_volno()) {
                    Serial.println("[MOZEK] Cesta X volná, pokračuji.");
                    posli_prikaz(CMD_JED_SBIREJ, 60);
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
                        if (h_err > 0) posli_prikaz(CMD_OTOC_VLEVO, (int16_t)fabsf(h_err));
                        else posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)fabsf(h_err));
                    }
                    krok = 4;
                }
                break;
            }
            case 4:
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
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
        Serial.println("[MOZEK] ═══ ZÁPAS ZAHÁJEN — NÁJEZD NAHORU ═══");
        posli_prikaz(CMD_JED_SBIREJ, 60);
        zmen_stav(STAV_NAJEZD_NAHORU);
    } else {
        // ═══ DRUHÝ A DALŠÍ START (dynamický režim) ═══
        dynamicky_rezim = true;
        int dummy_row;
        if (vypocti_dalsi_cil(dyn_start_x, dyn_end_x, dyn_y, dummy_row)) {
            Serial.printf("[MOZEK] ═══ DRUHÁ JÍZDA (DYNAMICKÁ) ═══\n");
            Serial.printf("[MOZEK] Úsek: Y=%.0f, X=%.0f až %.0f\n", dyn_y, dyn_start_x, dyn_end_x);
            zmen_stav(STAV_PRESUN_Y);
        } else {
            Serial.println("[MOZEK] Mapa už přejetá!");
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
