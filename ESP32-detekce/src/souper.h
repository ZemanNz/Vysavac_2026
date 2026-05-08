#pragma once

// =============================================================================
//  souper.h — Test vyhýbání soupeři
//
//  UP → jeď → [soupeř!] → otoč DOLEVA → jeď a sleduj PRAVOU stranu →
//  [soupeř zmizel z pravé strany!] → popojeď 200mm → otoč DOPRAVA → jeď dál
// =============================================================================

#define SOUPER_RYCHLOST_LAJNA   60
#define SOUPER_RYCHLOST_VYHYBKA 25     // pomalé přejíždění
#define SOUPER_REZERVA_MM       150    // kolik mm ještě popojet po přejetí soupeře
#define SOUPER_PRAH_VOLNO_MM    700.0f // pravá strana > tohle = soupeř tam není

enum SouperTestStav : uint8_t {
    ST_CEKAM,
    ST_JEDU_LAJNU,
    ST_VYHYBAM_JEDU,    // jedu dolů, sleduju pravou stranu
    ST_VYHYBAM_DOJIZDIM,// soupeř zmizel, jedu ještě 200mm
    ST_HOTOVO,
};

static SouperTestStav st_stav = ST_CEKAM;
static int st_krok = 0;
static bool st_souper_byl_viden = false;  // viděli jsme soupeře na pravé straně?
static int  st_volno_pocet = 0;           // kolikrát za sebou byla pravá strana volná
static float st_posledni_prava = -1.0f;   // poslední známá hodnota (pro detekci nových dat)

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
        bool btn = rbcx.tlacitko_vpredu_up;

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
            st_volno_pocet = 0;
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

    // ─── JEDU KOLEM SOUPEŘE — sleduju pravou stranu ───
    case ST_VYHYBAM_JEDU: {
        float prava = nv_dist_right;
        
        // Kontroluj JEN když přišla nová hodnota z LiDÁRu (jiná než minule)
        if (prava != st_posledni_prava) {
            st_posledni_prava = prava;
            
            // Vypiš novou hodnotu
            Serial.printf("[SOUPER] Pravá: %4dmm  %s\n",
                (int)prava,
                (prava < SOUPER_PRAH_VOLNO_MM) ? "<<< SOUPEŘ >>>" : "--- volno ---");
            
            // Nejdřív musíme soupeře VIDĚT na pravé straně
            if (!st_souper_byl_viden && prava < SOUPER_PRAH_VOLNO_MM) {
                st_souper_byl_viden = true;
                Serial.printf("[SOUPER] *** Soupeř detekován na pravé straně! (%dmm) ***\n", (int)prava);
            }
            
            // Pak čekáme, až ZMIZÍ — 2× za sebou s NOVOU hodnotou
            if (st_souper_byl_viden) {
                if (prava >= SOUPER_PRAH_VOLNO_MM) {
                    st_volno_pocet++;
                    Serial.printf("[SOUPER]   → volno %d/2\n", st_volno_pocet);
                } else {
                    if (st_volno_pocet > 0) {
                        Serial.printf("[SOUPER]   → reset (zase soupeř)\n");
                    }
                    st_volno_pocet = 0;
                }
                
                if (st_volno_pocet >= 2) {
                    Serial.printf("[SOUPER] *** SOUPEŘ PŘEJET! *** Pravá: %dmm > práh %dmm (2× potvrzeno)\n",
                        (int)prava, (int)SOUPER_PRAH_VOLNO_MM);
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

    // ─── DOJÍŽDÍM 200mm PO PŘEJETÍ SOUPEŘE ───
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
