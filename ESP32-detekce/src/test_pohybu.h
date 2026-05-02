#pragma once
#include <Arduino.h>

// =============================================================================
//  TEST POHYBU A ZAROVNÁNÍ (Řízení "shora" z ESP32)
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
    // Nový příkaz: zapne motory a nechá je točit, dokud ESP32 nepošle CMD_STOP
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
//  STAVOVÉ PROMĚNNÉ A KONFIGURACE
// =============================================================================

enum TestStav {
    TEST_IDLE,
    TEST_CEKAM_NA_TLA,
    TEST_TOCIM_SE,
    TEST_HOTOVO
};

static TestStav t_stav = TEST_CEKAM_NA_TLA;
static float cilovy_uhel = 0.0f;

static float T_TOLERANCE_DEG  = 2.5f;
static float T_SLOWDOWN_DEG   = 12.0f;
static int16_t T_CRUISE_SPEED = 10;
static int16_t T_SLOW_SPEED   = 3;

// =============================================================================
//  POMOCNÉ FUNKCE
// =============================================================================

// Pomocná funkce pro výpočet nejkratšího rozdílu mezi úhly (-180 až 180)
float vypocti_rozdil_uhlu(float cil, float aktualni) {
    float r = cil - aktualni;
    while (r > 180.0f) r -= 360.0f;
    while (r < -180.0f) r += 360.0f;
    return r;
}

// Funkce, která najde nejbližší "rovnoběžný" úhel vzhledem ke zdem (0, 90, 180, 270)
float najdi_nejblizsi_rovnobezku(float aktualni_heading_deg) {
    float a = aktualni_heading_deg;
    while (a < 0) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;

    if (a >= 315 || a < 45) return 0.0f;
    if (a >= 45 && a < 135) return 90.0f;
    if (a >= 135 && a < 225) return 180.0f;
    return 270.0f;
}

void test_start_otoceni(float target_deg) {
    float heading_deg = nv_g_h * 180.0f / PI;
    cilovy_uhel = target_deg;
    while (cilovy_uhel < 0) cilovy_uhel += 360.0f;
    while (cilovy_uhel >= 360.0f) cilovy_uhel -= 360.0f;

    Serial.printf("[TEST] Start otoceni. Aktualni: %.1f°, Cil: %.1f°\n", heading_deg, cilovy_uhel);
    float rozdil = vypocti_rozdil_uhlu(cilovy_uhel, heading_deg);
    test_posli_prikaz(CMD_TOC_KONTINUALNE, (rozdil > 0) ? T_CRUISE_SPEED : -T_CRUISE_SPEED);
    t_stav = TEST_TOCIM_SE;
}
// Otočí se o 90 stupňů (nebo 0 pro zarovnání) vzhledem k nejbližší "čisté" rovnoběžce
void otoc_o_90(bool vlevo, bool jen_zarovnat = false) {
    float heading_deg = nv_g_h * 180.0f / PI;
    float base = najdi_nejblizsi_rovnobezku(heading_deg);
    float target = base;
    if (!jen_zarovnat) target += vlevo ? -90.0f : 90.0f;
    test_start_otoceni(target);
}
// Vyrovná se podle nejbližší rovnoběžné stěny (0, 90, 180, 270)
void zarovnej_podle_steny() {
    otoc_o_90(false, true); // Použijeme pomocnou pro zarovnání (relativní 0)
}



// =============================================================================
//  HLAVNÍ TESTOVACÍ FUNKCE (Init a Update)
// =============================================================================

void test_pohybu_init() {
    test_uart_init();
    Serial.println("[TEST] Pripraven. Cekam na spusteni (za 5 vterin)...");
    t_stav = TEST_CEKAM_NA_TLA;
}

void kontroluj_zarovnavani() {
    // Odpočet 5 sekund po startu a pak začne test
    static unsigned long start_cas = millis();
    if (t_stav == TEST_CEKAM_NA_TLA) {
        if (millis() - start_cas > 5000) { 
            otoc_o_90(false); // Otočit 90 stupňů doleva (relativně k nejbližší zdi)
        }
    }

    if (t_stav == TEST_TOCIM_SE) {
        float heading_deg = nv_g_h * 180.0f / PI;
        float rozdil = vypocti_rozdil_uhlu(cilovy_uhel, heading_deg);
        
        static int16_t last_sent_speed = 0;
        
        int16_t target_speed = (rozdil > 0) ? T_CRUISE_SPEED : -T_CRUISE_SPEED;
        if (fabs(rozdil) <= T_SLOWDOWN_DEG) {
            target_speed = (rozdil > 0) ? T_SLOW_SPEED : -T_SLOW_SPEED;
        }

        if (target_speed != last_sent_speed) {
            test_posli_prikaz(CMD_TOC_KONTINUALNE, target_speed);
            last_sent_speed = target_speed;
        }

        // Pokud jsme v toleranci, ihned pošleme STOP
        if (fabs(rozdil) <= T_TOLERANCE_DEG) {
            test_posli_prikaz(CMD_STOP);
            Serial.printf("[TEST] Dosažen cíl! Aktualni uhel: %.1f°. HOTOVO.\n", heading_deg);
            t_stav = TEST_HOTOVO;
            last_sent_speed = 0;
        }
    }
}
