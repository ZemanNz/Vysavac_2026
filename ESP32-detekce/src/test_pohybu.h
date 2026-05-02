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
//  TESTOVACÍ LOGIKA
// =============================================================================

enum TestStav {
    TEST_IDLE,
    TEST_CEKAM_NA_TLA,
    TEST_TOCIM_SE,
    TEST_HOTOVO
};

static TestStav t_stav = TEST_CEKAM_NA_TLA;
static float cilovy_uhel = 0.0f;

// Funkce, která najde nejbližší "rovnoběžný" úhel vzhledem ke zdem (0, 90, 180, 270)
float najdi_nejblizsi_rovnobezku(float aktualni_heading_deg) {
    // Normalizace do 0-360
    while (aktualni_heading_deg < 0) aktualni_heading_deg += 360.0f;
    while (aktualni_heading_deg >= 360.0f) aktualni_heading_deg -= 360.0f;

    if (aktualni_heading_deg >= 315 || aktualni_heading_deg < 45) return 0.0f;
    if (aktualni_heading_deg >= 45 && aktualni_heading_deg < 135) return 90.0f;
    if (aktualni_heading_deg >= 135 && aktualni_heading_deg < 225) return 180.0f;
    return 270.0f;
}

void test_start_otoceni(float target_deg) {
    float heading_deg = nv_g_h * 180.0f / PI;
    cilovy_uhel = target_deg;
    
    // Normalizace cíle
    while (cilovy_uhel < 0) cilovy_uhel += 360.0f;
    while (cilovy_uhel >= 360.0f) cilovy_uhel -= 360.0f;

    Serial.printf("[TEST] Start otoceni. Aktualni: %.1f°, Cil: %.1f°\n", heading_deg, cilovy_uhel);
    
    float rozdil = cilovy_uhel - heading_deg;
    while (rozdil > 180.0f) rozdil -= 360.0f;
    while (rozdil < -180.0f) rozdil += 360.0f;
    
    if (rozdil > 0) {
        test_posli_prikaz(CMD_TOC_KONTINUALNE, 10); 
    } else {
        test_posli_prikaz(CMD_TOC_KONTINUALNE, -10);
    }
    
    t_stav = TEST_TOCIM_SE;
}

// Nová funkce: otočí se o 90 stupňů vzhledem k nejbližší "čisté" rovnoběžce
void test_otoc_o_90(bool vlevo) {
    float heading_deg = nv_g_h * 180.0f / PI;
    float aktualni_rovnobezka = najdi_nejblizsi_rovnobezku(heading_deg);
    
    float target;
    if (vlevo) {
        target = aktualni_rovnobezka + 90.0f;
    } else {
        target = aktualni_rovnobezka - 90.0f;
    }
    test_start_otoceni(target);
}

void test_pohybu_init() {
    test_uart_init();
    Serial.println("[TEST] Pripraven. Cekam na spusteni (napr. za 5 vterin)...");
    t_stav = TEST_CEKAM_NA_TLA;
}

void test_pohybu_start() {
    float heading_deg = nv_g_h * 180.0f / PI;
    float target = najdi_nejblizsi_rovnobezku(heading_deg);
    test_start_otoceni(target);
}

void test_pohybu_update() {
    // Odpočet 5 sekund po startu a pak začne test (zarovnání na nejbližší zeď)
    static unsigned long start_cas = millis();
    if (t_stav == TEST_CEKAM_NA_TLA) {
        if (millis() - start_cas > 5000) { 
            test_pohybu_start();
        }
    }

    if (t_stav == TEST_TOCIM_SE) {
        float heading_deg = nv_g_h * 180.0f / PI;
        
        float rozdil = cilovy_uhel - heading_deg;
        while (rozdil > 180.0f) rozdil -= 360.0f;
        while (rozdil < -180.0f) rozdil += 360.0f;
        
        // --- LOGIKA ZPOMALOVÁNÍ ---
        static int16_t last_sent_speed = 0;
        
        // Reset last_sent_speed pokud jsme právě začali (t_stav se změnil nebo jsme v prvním loopu rotace)
        // (Zde to řešíme tak, že pokud se t_stav změnil na TEST_TOCIM_SE, last_sent_speed by mělo být 0)
        
        int16_t target_speed = (rozdil > 0) ? 10 : -10;
        if (fabs(rozdil) <= 9.0f) {
            target_speed = (rozdil > 0) ? 3 : -3;
        }

        if (target_speed != last_sent_speed) {
            test_posli_prikaz(CMD_TOC_KONTINUALNE, target_speed);
            last_sent_speed = target_speed;
        }

        // Pokud jsme v toleranci např. 2.5 stupně, ihned pošleme STOP
        if (fabs(rozdil) <= 2.5f) {
            test_posli_prikaz(CMD_STOP);
            Serial.printf("[TEST] Dosažen cíl! Aktualni uhel: %.1f°. HOTOVO.\n", heading_deg);
            t_stav = TEST_HOTOVO;
            last_sent_speed = 0;
        }
    }
}
