import re

with open('ESP32-detekce/src/mozek.h', 'r') as f:
    content = f.read()

# Add CMD_LIDAR_ERROR and CMD_TOC_KONTINUALNE to CmdID enum
content = re.sub(
    r'CMD_ZAVRI_ZASOBNIKY = 0x07,',
    'CMD_ZAVRI_ZASOBNIKY = 0x07,\n    CMD_TOC_KONTINUALNE = 0x08,\n    CMD_LIDAR_ERROR     = 0x09',
    content
)

# Add helper functions before mozek_uart_init
helpers = """
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

    Serial.printf("[MOZEK] Blokujici rotace na %.1f°\\n", target_deg);
    int16_t aktualni_rychlost = 0; 
    while (true) {
        loop_lidar_nv();
        mozek_aktualizuj_senzory();
        
        float heading_deg = senzory.heading;
        float rozdil = vypocti_rozdil_uhlu(target_deg, heading_deg);
        
        if (fabs(rozdil) <= 2.5f) { // T_TOLERANCE_DEG
            posli_prikaz(CMD_STOP);
            Serial.printf("[MOZEK] Rotace dokoncena (%.1f°)\\n", heading_deg);
            break; 
        }

        int16_t pozadovana_rychlost = (rozdil > 0) ? 10 : -10; // T_CRUISE_SPEED
        if (fabs(rozdil) <= 12.0f) { // T_SLOWDOWN_DEG
            pozadovana_rychlost = (rozdil > 0) ? 3 : -3; // T_SLOW_SPEED
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
static unsigned long mozek_posledni_lidar_error_ms = 0;

void mozek_start_jizdy(int rychlost) {
    mozek_aktualizuj_senzory();
    mozek_cilovy_uhel_jizdy = najdi_nejblizsi_rovnobezku(senzory.heading);
    posli_prikaz(CMD_JED_SBIREJ, rychlost);
}

// =============================================================================
//  UART FUNKCE
"""

content = content.replace('// =============================================================================\n//  UART FUNKCE', helpers)

# Inside mozek_rozhoduj(), add the lidar error logic
lidar_logic = """    // Zbývající čas
    unsigned long ubehnuto = (cas_startu > 0) ? (millis() - cas_startu) : 0;
    unsigned long zbyva_ms = (ubehnuto < DELKA_ZAPASU_MS) ? (DELKA_ZAPASU_MS - ubehnuto) : 0;

    // Posílání korekcí během jízdy
    if (posledni_odeslany_prikaz == CMD_JED_SBIREJ && rbcx.stav != STAT_DONE && cas_startu > 0) {
        if (millis() - mozek_posledni_lidar_error_ms > 30) {
            float rozdil = vypocti_rozdil_uhlu(mozek_cilovy_uhel_jizdy, senzory.heading);
            int16_t param_err = (int16_t)roundf(rozdil * 10.0f);
            posli_korekci(param_err);
            mozek_posledni_lidar_error_ms = millis();
        }
    }
"""
content = content.replace('    // Zbývající čas\n    unsigned long ubehnuto = (cas_startu > 0) ? (millis() - cas_startu) : 0;\n    unsigned long zbyva_ms = (ubehnuto < DELKA_ZAPASU_MS) ? (DELKA_ZAPASU_MS - ubehnuto) : 0;\n', lidar_logic)


# Replace driving commands
content = re.sub(r'posli_prikaz\(CMD_JED_SBIREJ,\s*(\d+)\);', r'mozek_start_jizdy(\1);', content)

# Replace rotation commands
content = re.sub(r'posli_prikaz\(CMD_OTOC_VLEVO,\s*90\);', r'mozek_otoc_o_90(true);', content)
content = re.sub(r'posli_prikaz\(CMD_OTOC_VPRAVO,\s*90\);', r'mozek_otoc_o_90(false);', content)
content = re.sub(r'posli_prikaz\(CMD_OTOC_VLEVO,\s*180\);', r'mozek_otoc_o_180();', content)
content = re.sub(r'posli_prikaz\(CMD_OTOC_VPRAVO,\s*180\);', r'mozek_otoc_o_180();', content)

content = re.sub(r'posli_prikaz\(CMD_OTOC_VLEVO,\s*\(int16_t\)(.*?)\);', r'mozek_otoc_relativne(-(\1));', content)
content = re.sub(r'posli_prikaz\(CMD_OTOC_VPRAVO,\s*\(int16_t\)(.*?)\);', r'mozek_otoc_relativne(\1);', content)

content = re.sub(r'posli_prikaz\(CMD_OTOC_VLEVO,\s*uhel\);', r'mozek_otoc_relativne(-uhel);', content)
content = re.sub(r'posli_prikaz\(CMD_OTOC_VPRAVO,\s*uhel\);', r'mozek_otoc_relativne(uhel);', content)

with open('ESP32-detekce/src/mozek.h', 'w') as f:
    f.write(content)
