#pragma once
#include <Arduino.h>

// =============================================================================
//  TEST NOUZOVÉHO NÁVRATU (Blokující přístup)
//  Samostatný test pro ověření logiky návratu domů podle SLAMu a následného vyložení.
// =============================================================================

#define UART_RBCX_RX    16
#define UART_RBCX_TX    17
#define UART_RBCX_BAUD  115200

#define SYNC0 0xAA
#define SYNC1 0x55

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
    int16_t param2;
} EspCommandTestNavrat;

typedef struct __attribute__((packed)) {
    uint8_t status;
    uint8_t cmd_id;
    uint8_t buttons;
    int16_t pocet_puku;
    int16_t param;
    uint16_t uz1_mm;
    uint16_t uz3_mm;
} RbcxStatusNavrat;

enum CmdIDTestNavrat : uint8_t {
    CMD_STOP            = 0x01,
    CMD_JED_SBIREJ      = 0x02,
    CMD_VYLOZ           = 0x06,
    CMD_ZAVRI_ZASOBNIKY = 0x07,
    CMD_TOC_KONTINUALNE = 0x08,
    CMD_LIDAR_ERROR     = 0x09
};

enum UartRxStavNavrat : uint8_t { RX_CEKAM_SYNC0, RX_CEKAM_SYNC1, RX_CITAM_DATA };
static UartRxStavNavrat navrat_rx_stav = RX_CEKAM_SYNC0;
static uint8_t navrat_rx_buf[sizeof(RbcxStatusNavrat)];
static size_t navrat_rx_pocet = 0;
static bool navrat_btn_up = false;

void navrat_uart_init() {
    Serial1.begin(UART_RBCX_BAUD, SERIAL_8N1, UART_RBCX_RX, UART_RBCX_TX);
    Serial.println("[NAVRAT] UART init pro test_navratu");
}

void navrat_posli_prikaz(uint8_t cmd, int16_t param = 0, int16_t param2 = 0) {
    EspCommandTestNavrat c;
    c.cmd = cmd;
    c.param = param;
    c.param2 = param2;
    Serial1.write(SYNC0);
    Serial1.write(SYNC1);
    Serial1.write((uint8_t*)&c, sizeof(c));
    Serial.printf("[NAVRAT] >>> CMD: 0x%02X  param=%d param2=%d\n", cmd, param, param2);
}

void navrat_prijmi_uart() {
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        switch (navrat_rx_stav) {
            case RX_CEKAM_SYNC0:
                if (c == SYNC0) navrat_rx_stav = RX_CEKAM_SYNC1;
                break;
            case RX_CEKAM_SYNC1:
                if (c == SYNC1) { navrat_rx_stav = RX_CITAM_DATA; navrat_rx_pocet = 0; }
                else { navrat_rx_stav = (c == SYNC0) ? RX_CEKAM_SYNC1 : RX_CEKAM_SYNC0; }
                break;
            case RX_CITAM_DATA:
                navrat_rx_buf[navrat_rx_pocet++] = c;
                if (navrat_rx_pocet >= sizeof(RbcxStatusNavrat)) {
                    RbcxStatusNavrat st;
                    memcpy(&st, navrat_rx_buf, sizeof(st));
                    navrat_btn_up = (st.buttons >> 0) & 1;
                    navrat_rx_stav = RX_CEKAM_SYNC0;
                }
                break;
        }
    }
}

// HOME z mozek.h
#define HOME_ZONA_MM          700.0f
#define MOZEK_HOME_X  (NV_ARENA_SIZE - HOME_ZONA_MM / 2.0f)
#define MOZEK_HOME_Y  (HOME_ZONA_MM / 2.0f)

float navrat_vypocti_rozdil_uhlu(float cil, float aktualni) {
    float r = cil - aktualni;
    while (r > 180.0f) r -= 360.0f;
    while (r < -180.0f) r += 360.0f;
    return r;
}

void navrat_pockej_ms(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        navrat_prijmi_uart();
        loop_lidar_nv();
        delay(5);
    }
}

void navrat_otoc_se_na(float target_deg) {
    while (target_deg < 0) target_deg += 360.0f;
    while (target_deg >= 360.0f) target_deg -= 360.0f;

    Serial.printf("\n[NAVRAT] ---> Start rotace na %.1f°\n", target_deg);

    int16_t aktualni_rychlost = 0; 
    while (true) {
        navrat_prijmi_uart();
        loop_lidar_nv();
        
        float heading_deg = nv_g_h * 180.0f / PI;
        float rozdil = navrat_vypocti_rozdil_uhlu(target_deg, heading_deg);
        
        if (fabs(rozdil) <= 2.5f) {
            navrat_posli_prikaz(CMD_STOP);
            Serial.printf("[NAVRAT] <--- Cíl rotace dosažen (%.1f°)\n", heading_deg);
            break; 
        }

        int16_t pozadovana_rychlost = (rozdil > 0) ? 30 : -30;
        if (fabs(rozdil) <= 60.0f) {
            pozadovana_rychlost = (rozdil > 0) ? 10 : -10;
        }
        if (fabs(rozdil) <= 12.0f) {
            pozadovana_rychlost = (rozdil > 0) ? 3 : -3;
        }

        if (pozadovana_rychlost != aktualni_rychlost) {
            navrat_posli_prikaz(CMD_TOC_KONTINUALNE, pozadovana_rychlost);
            aktualni_rychlost = pozadovana_rychlost;
        }
        delay(5);
    }
}

void navrat_cekej_na_start() {
    Serial.println("[NAVRAT] Cekam na stisknuti a pusteni tlacitka UP na RBCX pro START...");
    bool bylo_stisknuto = false;
    while (true) {
        navrat_prijmi_uart();
        loop_lidar_nv();
        
        if (navrat_btn_up) {
            bylo_stisknuto = true;
        } else if (bylo_stisknuto && !navrat_btn_up) {
            Serial.println("[NAVRAT] Tlacitko UP uvolneno. Za 1s startujeme...");
            navrat_pockej_ms(1000);
            break;
        }
        delay(5);
    }
}

void test_navratu_sekvence() {
    Serial.println("[NAVRAT] Inicializace dokoncena.");
    
    // Nejprve pockame na start z tlacitka
    navrat_cekej_na_start();

    Serial.println("[NAVRAT] --- STARTUJEME NOUZOVÝ NÁVRAT ---");
    
    loop_lidar_nv();
    float x = constrain(nv_g_rx, 0, NV_ARENA_SIZE);
    float y = constrain(nv_g_ry, 0, NV_ARENA_SIZE);

    float dx = MOZEK_HOME_X - x;
    float dy = MOZEK_HOME_Y - y;
    float vzdalenost = sqrtf(dx*dx + dy*dy);
    float angle_home_rad = atan2f(dx, dy);
    float angle_home_deg = angle_home_rad * 180.0f / PI;

    Serial.printf("[NAVRAT] Pozice: X=%.0f, Y=%.0f. Domov: vzdal=%.0f, abs. uhel=%.1f°\n", x, y, vzdalenost, angle_home_deg);

    Serial.println("[NAVRAT] Krok 1: Otaceni k domovu...");
    navrat_otoc_se_na(angle_home_deg);

    navrat_pockej_ms(1000);

    Serial.println("[NAVRAT] Krok 2: Jizda k domovu...");
    navrat_posli_prikaz(CMD_JED_SBIREJ, 90);

    unsigned long last_update = 0;

    while (true) {
        navrat_prijmi_uart();
        loop_lidar_nv();
        x = constrain(nv_g_rx, 0, NV_ARENA_SIZE);
        y = constrain(nv_g_ry, 0, NV_ARENA_SIZE);
        dx = MOZEK_HOME_X - x;
        dy = MOZEK_HOME_Y - y;
        vzdalenost = sqrtf(dx*dx + dy*dy);

        if (millis() - last_update > 30) {
            float curr_h_deg = nv_g_h * 180.0f / PI;
            float rozdil = navrat_vypocti_rozdil_uhlu(angle_home_deg, curr_h_deg); 
            int16_t param_err = (int16_t)roundf(rozdil * 10.0f);
            navrat_posli_prikaz(CMD_LIDAR_ERROR, param_err);
            last_update = millis();
        }

        if (vzdalenost < 150.0f) {
            Serial.printf("[NAVRAT] Dosažen domov! (Vzdalenost=%.0f)\n", vzdalenost);
            navrat_posli_prikaz(CMD_STOP);
            break;
        }

        if (nv_dist_front < 200.0f) {
            Serial.printf("[NAVRAT] Nouzove zastaveni kvuli zdi! (Vzdalenost vpřed=%.0f)\n", nv_dist_front);
            navrat_posli_prikaz(CMD_STOP);
            break;
        }

        delay(5);
    }

    Serial.println("[NAVRAT] Krok 2.5: Otaceni na vykladaci uhel (0 stupnu)...");
    navrat_pockej_ms(500);
    navrat_otoc_se_na(0.0f);
    navrat_pockej_ms(500);

    Serial.println("[NAVRAT] Krok 3: Vyložení puků z nouzového návratu...");
    navrat_pockej_ms(500);

    Serial.println("[NAVRAT] Otevírám zásobníky...");
    navrat_posli_prikaz(CMD_VYLOZ);
    navrat_pockej_ms(500);

    Serial.println("[NAVRAT] Popojíždím pro vyložení...");
    navrat_posli_prikaz(CMD_JED_SBIREJ, 40);
    navrat_pockej_ms(1500);
    navrat_posli_prikaz(CMD_STOP);

    navrat_pockej_ms(200);

    Serial.println("[NAVRAT] Zavírám zásobníky...");
    navrat_posli_prikaz(CMD_ZAVRI_ZASOBNIKY);
    navrat_pockej_ms(500);

    Serial.println("[NAVRAT] === NÁVRAT A VYLOŽENÍ DOKONČENY ===");
}

void test_navratu_init() {
    navrat_uart_init();
    Serial.println("[NAVRAT] Pripraven.");
}
