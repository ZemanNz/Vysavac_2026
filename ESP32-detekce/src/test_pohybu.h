#pragma once
#include <Arduino.h>

// =============================================================================
//  TEST POHYBU A ZAROVNÁNÍ (Blokující přístup)
//  Tento soubor dočasně nahrazuje mozek.h pro testování rotace podle LiDARu.
// =============================================================================

#define UART_RBCX_RX    16
#define UART_RBCX_TX    17
#define UART_RBCX_BAUD  115200

#define SYNC0 0xAA
#define SYNC1 0x55

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
} EspCommandTest;

enum CmdIDTest : uint8_t {
    CMD_STOP            = 0x01,
    CMD_JED_SBIREJ      = 0x02,
    CMD_OTOC_VLEVO      = 0x03,
    CMD_OTOC_VPRAVO     = 0x04,
    CMD_TOC_KONTINUALNE = 0x08 
};

void test_uart_init() {
    Serial1.begin(UART_RBCX_BAUD, SERIAL_8N1, UART_RBCX_RX, UART_RBCX_TX);
    Serial.println("[TEST] UART init pro test_pohybu");
}

void test_posli_prikaz(uint8_t cmd, int16_t param = 0) {
    EspCommandTest c;
    c.cmd = cmd;
    c.param = param;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
    Serial.printf("[TEST] >>> CMD: 0x%02X  param=%d\n", cmd, param);
}

// =============================================================================
//  KONFIGURACE (vráceny původní funkční rychlosti 10 a 3)
// =============================================================================

static float T_TOLERANCE_DEG  = 2.5f;
static float T_SLOWDOWN_DEG   = 12.0f;
static int16_t T_CRUISE_SPEED = 10;
static int16_t T_SLOW_SPEED   = 3;

// =============================================================================
//  POMOCNÉ MATEMATICKÉ FUNKCE
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

// Pomocná funkce pro blokující čekání (udržuje aktuální LiDAR data)
void pockej_ms(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        loop_lidar_nv();
        delay(5);
    }
}

// =============================================================================
//  HLAVNÍ BLOKUJÍCÍ FUNKCE PRO ROTACI
// =============================================================================

// Jedna "hezká" blokující funkce, která obslouží celou rotaci
void otoc_se(bool vlevo, bool jen_zarovnat = false) {
    // 1. Zjištění aktuálního úhlu a výpočet cíle
    float heading_deg = nv_g_h * 180.0f / PI;
    float base = najdi_nejblizsi_rovnobezku(heading_deg);
    float target_deg = base;
    if (!jen_zarovnat) {
        target_deg += vlevo ? -90.0f : 90.0f;
    }
    
    // Normalizace
    while (target_deg < 0) target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;

    Serial.printf("\n[TEST] ---> Start otoceni. Aktualni: %.1f°, Cil: %.1f°\n", heading_deg, target_deg);

    // Prvotní odeslání rychlosti nastaveno na 0, aby první smyčka hned odeslala správnou počáteční rychlost
    int16_t aktualni_rychlost = 0; 

    // 2. Blokující smyčka - běží, dokud se nedosáhne cíle
    while (true) {
        // Udržujeme LiDAR neustále aktualizovaný
        loop_lidar_nv();
        
        heading_deg = nv_g_h * 180.0f / PI;
        float rozdil = vypocti_rozdil_uhlu(target_deg, heading_deg);
        
        // Cíl dosažen?
        if (fabs(rozdil) <= T_TOLERANCE_DEG) {
            test_posli_prikaz(CMD_STOP);
            Serial.printf("[TEST] <--- Dosažen cíl! Aktualni uhel: %.1f°. HOTOVO.\n\n", heading_deg);
            break; // Vyskočíme ze smyčky a program může pokračovat
        }

        // Zpomalování
        int16_t pozadovana_rychlost = (rozdil > 0) ? T_CRUISE_SPEED : -T_CRUISE_SPEED;
        if (fabs(rozdil) <= T_SLOWDOWN_DEG) {
            pozadovana_rychlost = (rozdil > 0) ? T_SLOW_SPEED : -T_SLOW_SPEED;
        }

        // Posíláme příkaz přes UART jen při změně rychlosti
        if (pozadovana_rychlost != aktualni_rychlost) {
            test_posli_prikaz(CMD_TOC_KONTINUALNE, pozadovana_rychlost);
            aktualni_rychlost = pozadovana_rychlost;
        }

        delay(5);
    }
}

// =============================================================================
//  INICIALIZACE A SEKVENCE (voláno z main.cpp)
// =============================================================================

void test_pohybu_init() {
    test_uart_init();
    Serial.println("[TEST] Pripraven.");
}

void test_pohybu_sekvence() {
    Serial.println("[TEST] Cekam 5 sekund na boot a nacteni LiDARu...");
    pockej_ms(5000); 

    
    Serial.println("[TEST] Krok 1: Pouhe zarovnani podle zdi...");
    otoc_se(false, true);

    Serial.println("[TEST] Cekam 3 sekundy...");
    pockej_ms(3000);

    Serial.println("[TEST] Krok 2: Otoceni o 90° vlevo...");
    otoc_se(true, false);

    Serial.println("[TEST] Cekam 3 sekundy...");
    pockej_ms(3000);

    Serial.println("[TEST] Krok 3: Otoceni zpet (90° vpravo)...");
    otoc_se(false, false);

    Serial.println("[TEST] === VSECHNY TESTY DOKONCENY ===");
}
