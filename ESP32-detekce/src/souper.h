#pragma once

// =============================================================================
//  souper.h — Test vyhýbání soupeři
//
//  UP   → soupeř bude VPRAVO (otočíme DOLEVA, sledujeme pravou stranu)
//  DOWN → soupeř bude VLEVO  (otočíme DOPRAVA, sledujeme levou stranu)
//
//  2-fázová detekce: LiDAR 2× volno → UZ 2× volno → přejet!
// =============================================================================

#define SOUPER_RYCHLOST_LAJNA   60
#define SOUPER_RYCHLOST_VYHYBKA 25     // pomalé přejíždění
#define SOUPER_REZERVA_MM       100    // kolik mm ještě popojet po přejetí soupeře
#define SOUPER_PRAH_LIDAR_MM    700.0f // LiDAR: > tohle = soupeř tam není
#define SOUPER_PRAH_UZ_MM       400    // Ultrazvuk: > tohle = soupeř tam není

enum SouperTestStav : uint8_t {
    ST_CEKAM,
    ST_JEDU_LAJNU,
    ST_VYHYBAM_JEDU,    // jedu kolem, sleduju LiDAR + UZ
    ST_VYHYBAM_DOJIZDIM,// soupeř zmizel, jedu ještě 100mm
    ST_HOTOVO,
};

static SouperTestStav st_stav = ST_CEKAM;
static int st_krok = 0;
static bool st_souper_byl_viden = false;
static int  st_lidar_volno_pocet = 0;
static bool st_lidar_potvrzeno = false;
static int  st_uz_volno_pocet = 0;
static float st_posledni_bok = -1.0f;   // poslední LiDAR boční hodnota
static char  st_souper_strana = 'R';     // 'R' = soupeř vpravo, 'L' = soupeř vlevo

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

// Vrátí boční LiDAR vzdálenost podle strany soupeře
float st_lidar_bok() {
    return (st_souper_strana == 'R') ? nv_dist_right : nv_dist_left;
}

// Vrátí boční UZ vzdálenost podle strany soupeře
uint16_t st_uz_bok() {
    return (st_souper_strana == 'R') ? rbcx.uz3_mm : rbcx.uz1_mm;
}

// Vrátí jméno UZ senzoru
const char* st_uz_jmeno() {
    return (st_souper_strana == 'R') ? "UZ3" : "UZ1";
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
    st_souper_strana = 'R';
    Serial.println("[SOUPER] === READY === UP=soupeř vpravo, DOWN=soupeř vlevo");
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

    // ─── ČEKÁM NA START (UP nebo DOWN) ───
    case ST_CEKAM: {
        static bool btn_up_pred = false;
        static bool btn_dn_pred = false;
        static unsigned long uvolneno = 0;
        static unsigned long posledni_debug = 0;
        bool btn_up = rbcx.tlacitko_vpredu_up;
        bool btn_dn = rbcx.tlacitko_vpredu_down;

        // Vypisuj senzory každých 500ms
        if (millis() - posledni_debug > 500) {
            posledni_debug = millis();
            Serial.printf("[SENZORY] UZ1=%4dmm UZ3=%4dmm | LiDAR L=%4dmm R=%4dmm | strana=%c\n",
                rbcx.uz1_mm, rbcx.uz3_mm,
                (int)nv_dist_left, (int)nv_dist_right,
                st_souper_strana);
        }

        // UP → soupeř bude VPRAVO (objíždíme DOLEVA)
        if (!btn_up && btn_up_pred && rbcx.pripojeno) {
            st_souper_strana = 'R';
            uvolneno = millis();
            Serial.println("[SOUPER] UP → soupeř VPRAVO, start za 1s...");
        }
        // DOWN → soupeř bude VLEVO (objíždíme DOPRAVA)
        if (!btn_dn && btn_dn_pred && rbcx.pripojeno) {
            st_souper_strana = 'L';
            uvolneno = millis();
            Serial.println("[SOUPER] DOWN → soupeř VLEVO, start za 1s...");
        }
        btn_up_pred = btn_up;
        btn_dn_pred = btn_dn;

        if (uvolneno > 0 && (millis() - uvolneno > 1000)) {
            uvolneno = 0;
            Serial.printf("[SOUPER] === JEDU! === (soupeř bude %s)\n",
                st_souper_strana == 'R' ? "VPRAVO" : "VLEVO");
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
        }
        break;
    }

    // ─── JEDU LAJNU ───
    case ST_JEDU_LAJNU:
        if (souper_v_ceste()) {
            bool otoc_doleva = (st_souper_strana == 'R');
            Serial.printf("[SOUPER] Soupeř! dist=%.0f → OTÁČÍM %s\n",
                senzory.souper_vzdalenost,
                otoc_doleva ? "DOLEVA" : "DOPRAVA");
            
            mozek_otoc_o_90(otoc_doleva);
            Serial.printf("[SOUPER] Otočeno. Jedu kolem, sleduju %s stranu...\n",
                st_souper_strana == 'R' ? "PRAVOU" : "LEVOU");
            
            st_souper_byl_viden = false;
            st_lidar_volno_pocet = 0;
            st_lidar_potvrzeno = false;
            st_uz_volno_pocet = 0;
            st_posledni_bok = -1.0f;
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
        float bok_lidar = st_lidar_bok();
        uint16_t bok_uz = st_uz_bok();
        
        // Kontroluj jen když přišla nová hodnota z LiDARu
        if (bok_lidar != st_posledni_bok) {
            st_posledni_bok = bok_lidar;
            
            bool lidar_vidi = (bok_lidar < SOUPER_PRAH_LIDAR_MM);
            bool uz_vidi = (bok_uz > 0 && bok_uz < SOUPER_PRAH_UZ_MM);
            
            Serial.printf("[SOUPER] LiDAR(%c):%4dmm(%s) %s:%4dmm(%s) %s\n",
                st_souper_strana,
                (int)bok_lidar, lidar_vidi ? "OBJ" : " - ",
                st_uz_jmeno(), bok_uz, uz_vidi ? "OBJ" : " - ",
                st_lidar_potvrzeno ? "[čekám na UZ]" : "[čekám na LiDAR]");
            
            // 1) Nejdřív musíme soupeře VIDĚT
            if (!st_souper_byl_viden && (lidar_vidi || uz_vidi)) {
                st_souper_byl_viden = true;
                Serial.printf("[SOUPER] *** Soupeř detekován na %s straně! ***\n",
                    st_souper_strana == 'R' ? "pravé" : "levé");
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
                    Serial.printf("[SOUPER] *** LiDAR potvrzeno → sleduju už jen %s ***\n", st_uz_jmeno());
                }
            }
            
            // 3) FÁZE 2: LiDAR hotovo → čekáme na UZ 2× volno
            if (st_lidar_potvrzeno) {
                if (!uz_vidi) {
                    st_uz_volno_pocet++;
                    Serial.printf("[SOUPER]   → %s volno %d/2\n", st_uz_jmeno(), st_uz_volno_pocet);
                } else {
                    if (st_uz_volno_pocet > 0) Serial.printf("[SOUPER]   → %s reset\n", st_uz_jmeno());
                    st_uz_volno_pocet = 0;
                }
                
                if (st_uz_volno_pocet >= 2) {
                    Serial.printf("[SOUPER] *** SOUPEŘ PŘEJET! *** (LiDAR + %s potvrzeno)\n", st_uz_jmeno());
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
            mozek_otoc_o_90(st_souper_strana == 'R');  // otočíme zpět
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
            break;
        }
        // Zeď blízko
        if (senzory.dist_vpredu <= 300.0f) {
            Serial.println("[SOUPER] Zeď při vyhýbání!");
            posli_prikaz(CMD_STOP);
            delay(500);
            mozek_otoc_o_90(st_souper_strana == 'R');  // otočíme zpět
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
            break;
        }
        break;
    }

    // ─── DOJÍŽDÍM 100mm PO PŘEJETÍ SOUPEŘE ───
    case ST_VYHYBAM_DOJIZDIM:
        if (rbcx_hotovo()) {
            // Otočíme zpět na lajnu (opačný směr než při vyhýbání)
            bool otoc_zpet = (st_souper_strana == 'R');  // R → otočíme doprava
            Serial.printf("[SOUPER] Rezerva ujeta → otáčím %s\n",
                otoc_zpet ? "DOPRAVA" : "DOLEVA");
            mozek_otoc_o_90(!otoc_zpet);  // opačně než při vyhýbání
            Serial.println("[SOUPER] Otočeno → jedu dál po lajně.");
            mozek_start_jizdy(SOUPER_RYCHLOST_LAJNA);
            st_zmen(ST_JEDU_LAJNU);
        }
        // Náraz
        if (naraz_vpredu()) {
            posli_prikaz(CMD_STOP);
            delay(500);
            mozek_otoc_o_90(st_souper_strana == 'R');
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
