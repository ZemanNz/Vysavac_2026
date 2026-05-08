#pragma once

// =============================================================================
//  souper.h — Test vyhýbání soupeři
//
//  Senzory pro detekci přejetí soupeře:
//    - LiDAR (nv_dist_right) — vpředu robota, ±3° kužel
//    - Ultrazvuk U3 (rbcx.uz3_mm) — vzadu robota, z RBCX desky
//
//  Logika: LiDAR 2× volno → sleduj už jen UZ → UZ 2× volno → přejet!
// =============================================================================

#define SOUPER_RYCHLOST_LAJNA   60
#define SOUPER_RYCHLOST_VYHYBKA 25     // pomalé přejíždění
#define SOUPER_REZERVA_MM       150    // kolik mm ještě popojet po přejetí soupeře
#define SOUPER_PRAH_LIDAR_MM    700.0f // LiDAR: pravá > tohle = soupeř tam není
#define SOUPER_PRAH_UZ_MM       400    // Ultrazvuk: > tohle = soupeř tam není

enum SouperTestStav : uint8_t {
    ST_CEKAM,
    ST_JEDU_LAJNU,
    ST_VYHYBAM_JEDU,    // jedu kolem, sleduju LiDAR + UZ
    ST_VYHYBAM_DOJIZDIM,// soupeř zmizel, jedu ještě 150mm
    ST_HOTOVO,
};

static SouperTestStav st_stav = ST_CEKAM;
static int st_krok = 0;
static bool st_souper_byl_viden = false;  // viděli jsme soupeře na pravé straně?
static int  st_lidar_volno_pocet = 0;    // kolikrát LiDAR řekl "volno" za sebou
static bool st_lidar_potvrzeno = false;  // LiDAR 2× potvrdil → díváme se jen na UZ
static int  st_uz_volno_pocet = 0;       // kolikrát UZ řekl "volno" za sebou
static float st_posledni_prava = -1.0f;  // pro detekci nových LiDAR dat

const char* st_jmeno(SouperTestStav s) {
    switch(s) {
        case ST_CEKAM:            return "CEKAM";
        case ST_JEDU_LAJNU:       return "JEDU_LAJNU";
        case ST_VYHYBAM_JEDU:     return "JEDU_KOLEM";
        case ST_VYHYBAM_DOJIZDIM: return "DOJIZDIM";
        case ST_HOTOVO:           return "HOTOVO";
        default:                  return "???";
    }
}

void st_zmen(SouperTestStav novy) {
    Serial.printf("[SOUPER] %s → %s\n", st_jmeno(st_stav), st_jmeno(novy));
    st_stav = novy;
    st_krok = 0;
}

void souper_init() {
    mozek_uart_init();
    memset(&senzory, 0, sizeof(senzory));
    memset(&rbcx, 0, sizeof(rbcx));
    senzory.domov_smer = 'R';
    senzory.souper_smer = 'R';
    st_stav = ST_CEKAM;
    st_krok = 0;
    st_souper_byl_viden = false;
    st_lidar_volno_pocet = 0;
    st_lidar_potvrzeno = false;
    st_uz_volno_pocet = 0;
    Serial.println("[SOUPER] === READY === čekám na UP...");
}

void souper_rozhoduj() {
    prijmi_stav_rbcx();

    // Korekce jízdy z LiDARu
    if (posledni_odeslany_prikaz == CMD_JED_SBIREJ && rbcx.stav == STAT_BUSY) {
        if (millis() - mozek_posledni_lidar_error_ms > 30) {
            float rozdil = vypocti_rozdil_uhlu(mozek_cilovy_uhel_jizdy, senzory.heading);
            posli_korekci((int16_t)roundf(rozdil * 10.0f));
            mozek_posledni_lidar_error_ms = millis();
        }
    }

    switch (st_stav) {

    // ─── ČEKÁM NA START ───
    case ST_CEKAM: {
        static bool btn_pred = false;
        static unsigned long uvolneno = 0;
        static unsigned long posledni_debug = 0;
        bool btn = rbcx.tlacitko_vpredu_up;

        // Vypisuj senzory každých 500ms
        if (millis() - posledni_debug > 500) {
            posledni_debug = millis();
            Serial.printf("[SENZORY] UZ1=%4dmm  UZ3=%4dmm (z RBCX) | LiDAR R=%4dmm\n",
                rbcx.uz1_mm, rbcx.uz3_mm, (int)nv_dist_right);
        }

        if (!btn && btn_pred && rbcx.pripojeno) {
            uvolneno = millis();
            Serial.println("[SOUPER] UP uvolněno, start za 1s...");
        }
        btn_pred = btn;

        if (uvolneno > 0 && (millis() - uvolneno > 1000)) {
            uvolneno = 0;
            Serial.println("[SOUPER] === JEDU! ===");
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
        }
        break;
    }

    // ─── JEDU LAJNU ───
    case ST_JEDU_LAJNU:
        // Soupeř → rovnou otoč doleva a jeď kolem
        if (souper_v_ceste()) {
            Serial.printf("[SOUPER] Soupeř! dist=%.0f → OTÁČÍM DOLEVA\n",
                senzory.souper_vzdalenost);
            
            // Blokující otočka doleva
            mozek_otoc_o_90(true);
            Serial.println("[SOUPER] Otočeno. Jedu kolem soupeře, sleduju PRAVOU stranu...");
            
            st_souper_byl_viden = false;
            st_lidar_volno_pocet = 0;
            st_lidar_potvrzeno = false;
            st_uz_volno_pocet = 0;
            st_posledni_prava = -1.0f;
            mozek_start_jizdy(SOUPER_RYCHLOST_VYHYBKA);
            st_zmen(ST_VYHYBAM_JEDU);
            break;
        }
        // Náraz → konec
        if (naraz_vpredu()) {
            posli_prikaz(CMD_STOP);
            st_zmen(ST_HOTOVO);
            break;
        }
        // Zeď → konec
        if (senzory.dist_vpredu <= BEZPECNA_VZDALENOST_ZDI) {
            Serial.printf("[SOUPER] Zeď (%.0fmm) → STOP\n", senzory.dist_vpredu);
            posli_prikaz(CMD_STOP);
            st_zmen(ST_HOTOVO);
            break;
        }
        // Zpomalení
        if (mozek_aktualni_rychlost > RYCHLOST_DOJEZDU &&
            senzory.dist_vpredu <= BEZPECNA_VZDALENOST_ZDI + ZPOMALENI_VZDALENOST_MM) {
            mozek_start_jizdy(RYCHLOST_DOJEZDU);
        }
        break;

    // ─── JEDU KOLEM SOUPEŘE — 2-fázová detekce (LiDAR → UZ) ───
    case ST_VYHYBAM_JEDU: {
        float prava_lidar = nv_dist_right;
        uint16_t prava_uz = rbcx.uz3_mm;
        
        // Kontroluj jen když přišla nová hodnota z LiDARu
        if (prava_lidar != st_posledni_prava) {
            st_posledni_prava = prava_lidar;
            
            bool lidar_vidi = (prava_lidar < SOUPER_PRAH_LIDAR_MM);
            bool uz_vidi = (prava_uz > 0 && prava_uz < SOUPER_PRAH_UZ_MM);
            
            // Výpis stavu
            Serial.printf("[SOUPER] LiDAR:%4dmm(%s) UZ3:%4dmm(%s) %s\n",
                (int)prava_lidar, lidar_vidi ? "OBJ" : " - ",
                prava_uz,         uz_vidi    ? "OBJ" : " - ",
                st_lidar_potvrzeno ? "[čekám na UZ]" : "[čekám na LiDAR]");
            
            // 1) Nejdřív musíme soupeře VIDĚT (LiDAR nebo UZ)
            if (!st_souper_byl_viden && (lidar_vidi || uz_vidi)) {
                st_souper_byl_viden = true;
                Serial.println("[SOUPER] *** Soupeř detekován na pravé straně! ***");
            }
            
            if (!st_souper_byl_viden) break;
            
            // 2) FÁZE 1: Čekáme na LiDAR 2× volno
            if (!st_lidar_potvrzeno) {
                if (!lidar_vidi) {
                    st_lidar_volno_pocet++;
                    Serial.printf("[SOUPER]   → LiDAR volno %d/2\n", st_lidar_volno_pocet);
                } else {
                    if (st_lidar_volno_pocet > 0) Serial.println("[SOUPER]   → LiDAR reset");
                    st_lidar_volno_pocet = 0;
                }
                
                if (st_lidar_volno_pocet >= 2) {
                    st_lidar_potvrzeno = true;
                    Serial.println("[SOUPER] *** LiDAR potvrzeno → sleduju už jen UZ3 ***");
                }
            }
            
            // 3) FÁZE 2: LiDAR hotovo → čekáme na UZ 2× volno
            if (st_lidar_potvrzeno) {
                if (!uz_vidi) {
                    st_uz_volno_pocet++;
                    Serial.printf("[SOUPER]   → UZ3 volno %d/2\n", st_uz_volno_pocet);
                } else {
                    if (st_uz_volno_pocet > 0) Serial.println("[SOUPER]   → UZ3 reset");
                    st_uz_volno_pocet = 0;
                }
                
                if (st_uz_volno_pocet >= 2) {
                    Serial.println("[SOUPER] *** SOUPEŘ PŘEJET! *** (LiDAR + UZ3 potvrzeno)");
                    posli_prikaz(CMD_STOP);
                    Serial.println("[SOUPER] Zastavuji... čekám 5 sekund.");
                    delay(5000);
                    Serial.printf("[SOUPER] Popojíždím ještě %dmm pro rezervu...\n", SOUPER_REZERVA_MM);
                    posli_prikaz(CMD_JED_SBIREJ, SOUPER_RYCHLOST_VYHYBKA, SOUPER_REZERVA_MM);
                    st_zmen(ST_VYHYBAM_DOJIZDIM);
                    break;
                }
            }
        }
        
        // Náraz vpředu
        if (naraz_vpredu()) {
            Serial.println("[SOUPER] Náraz při vyhýbání!");
            posli_prikaz(CMD_STOP);
            delay(500);
            mozek_otoc_o_90(false);
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
            break;
        }
        // Zeď blízko
        if (senzory.dist_vpredu <= 300.0f) {
            Serial.println("[SOUPER] Zeď při vyhýbání!");
            posli_prikaz(CMD_STOP);
            delay(500);
            mozek_otoc_o_90(false);
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
            break;
        }
        break;
    }

    // ─── DOJÍŽDÍM 150mm PO PŘEJETÍ SOUPEŘE ───
    case ST_VYHYBAM_DOJIZDIM:
        if (rbcx_hotovo()) {
            Serial.println("[SOUPER] Rezerva ujeta → otáčím DOPRAVA");
            mozek_otoc_o_90(false);
            Serial.println("[SOUPER] Otočeno DOPRAVA → jedu dál po lajně.");
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
        }
        // Náraz
        if (naraz_vpredu()) {
            posli_prikaz(CMD_STOP);
            delay(500);
            mozek_otoc_o_90(false);
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
        }
        break;

    // ─── HOTOVO ───
    case ST_HOTOVO:
        break;
    }
}

void souper_update() {
    mozek_aktualizuj_senzory();
    souper_rozhoduj();
}
