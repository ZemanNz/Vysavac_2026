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
#define DELKA_ZAPASU_MS       90000    // 90 sekund
#define CAS_NOUZOVEHO_NAVRATU 10000    // posledních 10s → nouzový návrat

// Rozměry robota (pro lajnovou navigaci)
#define SIRKA_ROBOTA_MM       300.0f   // šířka robota = šířka jedné lajny
#define BEZPECNA_VZDALENOST_ZDI  500.0f  // nechceme jet blíž k protější zdi

// Vzdálenost soupeře pro vyhýbání
#define VZDALENOST_SOUPERE_STOP  400.0f  // mm — soupeř přímo v cestě → stop
#define UHEL_SOUPERE_VPRED        45.0f  // ° — soupeř v tomto kuželu = "v cestě"

// Limity puků
#define PUKY_PLNY_ZASOBNIK  5    // kolik puků → jedem domů

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
    CMD_NOP           = 0x00,
    CMD_STOP          = 0x01,
    CMD_JED_SBIREJ    = 0x02,   // param = rychlost %
    CMD_OTOC_VLEVO    = 0x03,   // param = úhel °
    CMD_OTOC_VPRAVO   = 0x04,   // param = úhel °
    CMD_COUVEJ        = 0x05,   // param = vzdálenost mm
    CMD_VYLOZ         = 0x06,
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
    char  domov_smer;         // 'L' nebo 'R'

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

#define BUNKA_MM    SIRKA_ROBOTA_MM
#define POCET_BUNEK_X  ((int)(NV_ARENA_SIZE / BUNKA_MM))   // 5 pro 1500mm
#define POCET_BUNEK_Y  ((int)(NV_ARENA_SIZE / BUNKA_MM))   // 5 pro 1500mm

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
        default:                          return "???";
    }
}

// =============================================================================
//  GLOBÁLNÍ PROMĚNNÉ
// =============================================================================

static SenzoroveData  senzory;
static StavRbcx       rbcx;
static StavRobota     stav = STAV_CEKAM_NA_START;
static unsigned long  cas_startu = 0;    // millis() kdy zápas začal (0 = nezačal)

// Mapa pokrytí
static bool mapa_pokryti[POCET_BUNEK_X][POCET_BUNEK_Y];

// Lajnová navigace
static LajnovaNavigace navigace;

// Sub-krok pro vícekrokové manévry (otáčení, vyhýbání, vykládání...)
// Každý stav co potřebuje víc kroků si ho používá interně.
static int krok = 0;

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
    return rbcx.stav == STAT_DONE || rbcx.stav == STAT_READY;
}

bool souper_v_ceste() {
    return senzory.souper_viden
        && senzory.souper_vzdalenost < VZDALENOST_SOUPERE_STOP
        && senzory.souper_uhel < UHEL_SOUPERE_VPRED;
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

    // Domov
    float dx = HOME_X - senzory.pozice_x;
    float dy = HOME_Y - senzory.pozice_y;
    senzory.domov_vzdalenost = sqrtf(dx*dx + dy*dy);
    float angle_home = atan2f(dx, dy);
    float rel = (angle_home - nv_g_h) * 180.0f / PI;
    while (rel >  180.0f) rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
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
    int bx = bunka_x(senzory.pozice_x);
    int by = bunka_y(senzory.pozice_y);
    mapa_pokryti[bx][by] = true;
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
    stav = novy;
    krok = 0;
}

void mozek_rozhoduj() {
    // Přijmi stav z RBCX (neblokující)
    prijmi_stav_rbcx();

    // Aktualizuj mapu pokrytí
    aktualizuj_pokryti();

    // Zbývající čas
    unsigned long ubehnuto = (cas_startu > 0) ? (millis() - cas_startu) : 0;
    unsigned long zbyva_ms = (ubehnuto < DELKA_ZAPASU_MS) ? (DELKA_ZAPASU_MS - ubehnuto) : 0;

    // ╔══════════════════════════════════════════════════════════╗
    // ║  NOUZOVÝ NÁVRAT — má nejvyšší prioritu, přeruší cokoliv ║
    // ╚══════════════════════════════════════════════════════════╝
    if (cas_startu > 0 && zbyva_ms < CAS_NOUZOVEHO_NAVRATU
        && stav != STAV_NOUZOVY_NAVRAT && stav != STAV_VYKLADAM_PUKY) {
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
    //  Robot stojí, čeká na signál. Nic nedělá.
    // ──────────────────────────────────────────────────────
    case STAV_CEKAM_NA_START:
        // TODO: Startovní mechanismus
        // Varianty:
        //   - Tlačítko LEFT na RBCX (rbcx.tlacitko_vlevo)
        //   - Příkaz přes UART
        //   - Automaticky po N sekundách
        //
        // Až přijde signál:
        //   cas_startu = millis();
        //   inicializuj_lajny();
        //   posli_prikaz(CMD_JED_SBIREJ, 60);  // jeď nahoru
        //   zmen_stav(STAV_NAJEZD_NAHORU);
        break;

    // ──────────────────────────────────────────────────────
    //  NÁJEZD NAHORU
    //  Robot startuje v HOME (pravý dolní roh) a jede vertikálně
    //  nahoru po pravé straně arény (heading=0° = +Y).
    //  Když Y dosáhne horní lajny → otoč doleva → JEDU_LAJNU
    // ──────────────────────────────────────────────────────
    case STAV_NAJEZD_NAHORU:
        switch (krok) {
            case 0:
                // Jedeme nahoru — čekáme než Y dosáhne lajny 0 (nahoře)
                if (senzory.pozice_y >= navigace.lajna_y[0] - SIRKA_ROBOTA_MM / 2.0f) {
                    posli_prikaz(CMD_STOP);
                    posli_prikaz(CMD_OTOC_VLEVO, 90);  // 0° → -90° (doleva)
                    krok = 1;
                }
                break;
            case 1:
                if (rbcx_hotovo()) {
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  JEDU LAJNU
    //  Hlavní stav — robot jede dopředu, sbírá puky.
    //  Průběžně kontroluje:
    //    - Dosáhl konce lajny? → přechod
    //    - Soupeř v cestě? → vyhýbání
    //    - Náraz (tlačítka)? → couvni + přechod
    //    - Plný zásobník? → domů
    // ──────────────────────────────────────────────────────
    case STAV_JEDU_LAJNU:

        // [A] Plný zásobník → jedeme domů vysypat
        if (rbcx.pocet_puku >= PUKY_PLNY_ZASOBNIK) {
            Serial.printf("[MOZEK] Plný zásobník (%d puků) → DOMŮ\n", rbcx.pocet_puku);
            posli_prikaz(CMD_STOP);
            zmen_stav(STAV_VRACIM_SE_DOMU);
            break;
        }

        // [B] Soupeř v cestě → vyhýbání
        if (souper_v_ceste()) {
            Serial.printf("[MOZEK] Soupeř v cestě! dist=%.0f uhel=%.0f → VYHÝBÁM SE\n",
                senzory.souper_vzdalenost, senzory.souper_uhel);
            posli_prikaz(CMD_STOP);
            zmen_stav(STAV_VYHYBAM_SE_SOUPERI);
            break;
        }

        // [C] Náraz vpředu (tlačítka) → couvni a přejeď na další lajnu
        if (naraz_vpredu()) {
            posli_prikaz(CMD_STOP);
            if (navigace.cislo_lajny + 1 >= navigace.pocet_lajn) {
                Serial.println("[MOZEK] Náraz na poslední lajně → DOMŮ");
                zmen_stav(STAV_VRACIM_SE_DOMU);
            } else {
                Serial.println("[MOZEK] Náraz vpředu → couvám + další lajna");
                zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
                // krok=0 → začne couváním
            }
            break;
        }

        // [D] Blízko protější zdi (bezpečná vzdálenost)
        if (dosahli_konce_lajny()) {
            posli_prikaz(CMD_STOP);
            if (navigace.cislo_lajny + 1 >= navigace.pocet_lajn) {
                Serial.printf("[MOZEK] Poslední lajna %d hotová → DOMŮ\n",
                    navigace.cislo_lajny);
                zmen_stav(STAV_VRACIM_SE_DOMU);
            } else {
                Serial.printf("[MOZEK] Konec lajny %d (X=%.0f) → PŘECHOD\n",
                    navigace.cislo_lajny, senzory.pozice_x);
                zmen_stav(STAV_PRECHOD_NA_DALSI_LAJNU);
                krok = 1;  // přeskoč couvání
            }
            break;
        }

        // [E] Jinak: jedeme dál (příkaz už byl poslán)
        break;

    // ──────────────────────────────────────────────────────
    //  PŘECHOD NA DALŠÍ LAJNU
    //  Vícekrokový manévr:
    //    krok 0: Couvni 100mm (odjet od zdi)
    //    krok 1: Otoč se doleva 90°
    //    krok 2: Popojeď o šířku robota (300mm)
    //    krok 3: Otoč se doleva 90°
    //    krok 4: Nastav novou lajnu → JEDU_LAJNU
    // ──────────────────────────────────────────────────────
    case STAV_PRECHOD_NA_DALSI_LAJNU:
        switch (krok) {
            case 0:  // Couvni
                posli_prikaz(CMD_COUVEJ, 100);
                krok = 1;
                break;
            case 1:  // Čekej na dokončení couvání → otoč DOLŮ
                if (rbcx_hotovo()) {
                    // Otáčíme podle směru jízdy aby robot šel vždy DOLŮ:
                    //   doleva  → VLEVO  (heading klesá → tváří se dolů)
                    //   doprava → VPRAVO (heading roste → tváří se dolů)
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 2;
                }
                break;
            case 2:  // Čekej na otočení
                if (rbcx_hotovo()) {
                    posli_prikaz(CMD_JED_SBIREJ, 40);  // pomalá jízda o šířku
                    krok = 3;
                }
                break;
            case 3:  // Popojeď o šířku robota
                // Čekáme než ujedeme ~300mm (sledujeme SLAM pozici)
                // TODO: sledovat ujetou vzdálenost přes SLAM
                // Zatím zjednodušeně: čas
                {
                    static unsigned long cas_kroku3 = 0;
                    if (cas_kroku3 == 0) cas_kroku3 = millis();
                    if (millis() - cas_kroku3 > 2000) {  // ~2s při 40%
                        posli_prikaz(CMD_STOP);
                        cas_kroku3 = 0;
                        krok = 4;
                    }
                }
                break;
            case 4:  // Druhé otočení (stejný směr — tváříme se na novou lajnu)
                if (rbcx_hotovo()) {
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 5;
                }
                break;
            case 5:  // Hotovo → nová lajna nebo domů
                if (rbcx_hotovo()) {
                    dalsi_lajna();
                    if (navigace.cislo_lajny >= navigace.pocet_lajn) {
                        Serial.println("[MOZEK] Všechny lajny hotové → DOMŮ");
                        posli_prikaz(CMD_STOP);
                        zmen_stav(STAV_VRACIM_SE_DOMU);
                    } else {
                        nastav_cil_lajny();
                        posli_prikaz(CMD_JED_SBIREJ, 60);
                        zmen_stav(STAV_JEDU_LAJNU);
                    }
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYHÝBÁM SE SOUPEŘI
    //  Soupeř stojí v cestě → zkrátíme lajnu, otočíme nahoru
    //  a přeskočíme na další lajnu.
    //    krok 0: Couvni 150mm
    //    krok 1: Otoč nahoru 90° (vlevo/vpravo podle směru)
    //    krok 2: Pokračuj na další lajně
    // ──────────────────────────────────────────────────────
    case STAV_VYHYBAM_SE_SOUPERI:
        switch (krok) {
            case 0:
                posli_prikaz(CMD_COUVEJ, 150);
                krok = 1;
                break;
            case 1:
                if (rbcx_hotovo()) {
                    if (navigace.smer_doprava)
                        posli_prikaz(CMD_OTOC_VPRAVO, 90);
                    else
                        posli_prikaz(CMD_OTOC_VLEVO, 90);
                    krok = 2;
                }
                break;
            case 2:
                if (rbcx_hotovo()) {
                    dalsi_lajna();
                    if (navigace.cislo_lajny >= navigace.pocet_lajn) {
                        Serial.println("[MOZEK] Všechny lajny hotové → DOMŮ");
                        posli_prikaz(CMD_STOP);
                        zmen_stav(STAV_VRACIM_SE_DOMU);
                    } else {
                        nastav_cil_lajny();
                        posli_prikaz(CMD_JED_SBIREJ, 60);
                        zmen_stav(STAV_JEDU_LAJNU);
                    }
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VRACÍM SE DOMŮ
    //  Navigace k domácí zóně:
    //    krok 0: Otoč se směrem k domovu
    //    krok 1: Jeď domů
    //    krok 2: Jsme tam → VYKLADAM
    // ──────────────────────────────────────────────────────
    case STAV_VRACIM_SE_DOMU:
        switch (krok) {
            case 0:  // Otoč se k domovu
                if (senzory.domov_uhel > 10.0f) {
                    // Potřebujeme se otočit
                    if (senzory.domov_smer == 'L')
                        posli_prikaz(CMD_OTOC_VLEVO, (int16_t)senzory.domov_uhel);
                    else
                        posli_prikaz(CMD_OTOC_VPRAVO, (int16_t)senzory.domov_uhel);
                    krok = 1;
                } else {
                    // Už míříme správně
                    krok = 2;
                }
                break;
            case 1:  // Čekej na otočení
                if (rbcx_hotovo()) krok = 2;
                break;
            case 2:  // Jeď domů
                posli_prikaz(CMD_JED_SBIREJ, 70);
                krok = 3;
                break;
            case 3:  // Sleduj vzdálenost k domovu
                if (senzory.domov_vzdalenost < 150.0f) {
                    // Jsme doma!
                    posli_prikaz(CMD_STOP);
                    zmen_stav(STAV_VYKLADAM_PUKY);
                }
                // Pokud jsme špatně namířeni (drift), koriguj
                else if (senzory.domov_uhel > 30.0f) {
                    posli_prikaz(CMD_STOP);
                    krok = 0;  // znovu zamiř
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  VYKLÁDÁM PUKY
    //  Doma → otevři zásobníky → popojeď → zavři → zpět sbírat
    //    krok 0: CMD_VYLOZ → čekej na DONE
    //    krok 1: Hotovo → zpět na SEARCH
    // ──────────────────────────────────────────────────────
    case STAV_VYKLADAM_PUKY:
        switch (krok) {
            case 0:
                posli_prikaz(CMD_VYLOZ);
                krok = 1;
                break;
            case 1:
                if (rbcx_hotovo()) {
                    Serial.printf("[MOZEK] Vyloženo! Jedu sbírat dál. (kolo %d, pokryto %d/%d)\n",
                        navigace.dokoncena_kola,
                        POCET_BUNEK_X * POCET_BUNEK_Y - nepokrytych_bunek(),
                        POCET_BUNEK_X * POCET_BUNEK_Y);
                    // Po vyložení jedem na nepokrytá místa
                    nastav_cil_lajny();
                    posli_prikaz(CMD_JED_SBIREJ, 60);
                    zmen_stav(STAV_JEDU_LAJNU);
                }
                break;
        }
        break;

    // ──────────────────────────────────────────────────────
    //  NOUZOVÝ NÁVRAT
    //  Čas končí! Jedeme domů co nejrychleji.
    //  Stejné jako VRACIM_SE_DOMU ale vyšší rychlost.
    //    krok 0: Otoč se k domovu
    //    krok 1: Čekej na otočení
    //    krok 2: Jeď
    //    krok 3: Dojel → VYLOŽ
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
                posli_prikaz(CMD_JED_SBIREJ, 90);  // plný plyn!
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

    } // switch (stav)

    // ╔══════════════════════════════════════════════════════════╗
    // ║  DEBUG VÝPIS — 1× za sekundu                            ║
    // ╚══════════════════════════════════════════════════════════╝
    static unsigned long posledni_debug = 0;
    static unsigned long posledni_mapa = 0;
    if (millis() - posledni_debug > 1000) {
        posledni_debug = millis();
        Serial.printf("[MOZEK] %s k=%d | t=%lus | RBCX:%s puky=%d | POS(%d,%d) H=%d° | HOME %dmm %d°%c | L%d/%d %s",
            jmeno_stavu(stav),
            krok,
            zbyva_ms / 1000,
            rbcx.pripojeno ? (rbcx_hotovo() ? "RDY" : "BSY") : "---",
            rbcx.pocet_puku,
            (int)senzory.pozice_x, (int)senzory.pozice_y,
            (int)senzory.heading,
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
    inicializuj_lajny();
    Serial.println("[MOZEK] === MOZEK READY === čekám na start...");
}

void mozek_update() {
    mozek_aktualizuj_senzory();
    mozek_rozhoduj();
}
