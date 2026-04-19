#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robotka.h"
#include "stepper_motor.h"
#include "funkce.h"
#include "asynchroni_pohyb.h"

// =============================================================================
//  MAIN_FINAL — Hlavní řídící program RBCX (Slave)
// =============================================================================
//
//  VLÁKNO 1 (hlavní): while(true) — vykonává pohyby podle příkazů
//  VLÁKNO 2 (UART):   přijímá příkazy z ESP32, odesílá stav
//
// =============================================================================

char nase_barva = 'R';
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// =============================================================================
//  KOMUNIKAČNÍ PROTOKOL
// =============================================================================

// --- ESP32 → RBCX (3 bajty) ---
typedef struct __attribute__((packed)) {
    uint8_t cmd;
    int16_t param;
} EspCommand;

// --- RBCX → ESP32 (6 bajtů) ---
typedef struct __attribute__((packed)) {
    uint8_t status;       // STAT_READY / STAT_BUSY / STAT_DONE
    uint8_t buttons;      // Bit 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    int16_t pocet_puku;   // Počet nasbíraných puků naší barvy
    int16_t param;        // Doplňkový parametr (záleží na kontextu)
} RbcxStatus;

// Příkazy (ESP32 → RBCX)
enum CmdID : uint8_t {
    CMD_NOP           = 0x00,  // Nic nedělej (jen pošli stav)
    CMD_STOP          = 0x01,  // Zastav vše
    CMD_JED_SBIREJ    = 0x02,  // Jeď dopředu + sbírej (param = rychlost %)
    CMD_OTOC_VLEVO    = 0x03,  // Otoč se doleva (param = úhel °)
    CMD_OTOC_VPRAVO   = 0x04,  // Otoč se doprava (param = úhel °)
    CMD_COUVEJ        = 0x05,  // Couvej (param = vzdálenost mm)
    CMD_VYLOZ         = 0x06,  // Vyložení puků
};

// Statusy (RBCX → ESP32)
enum StatID : uint8_t {
    STAT_READY        = 0x80,
    STAT_BUSY         = 0x81,
    STAT_DONE         = 0x82,
};

// Helper: jméno příkazu pro výpis
const char* cmd_name(uint8_t cmd) {
    switch(cmd) {
        case CMD_NOP:        return "NOP";
        case CMD_STOP:       return "STOP";
        case CMD_JED_SBIREJ: return "JED_SBIREJ";
        case CMD_OTOC_VLEVO: return "OTOC_VLEVO";
        case CMD_OTOC_VPRAVO:return "OTOC_VPRAVO";
        case CMD_COUVEJ:     return "COUVEJ";
        case CMD_VYLOZ:      return "VYLOZ";
        default:             return "???";
    }
}
const char* stav_name(uint8_t s) {
    switch(s) {
        case STAT_READY: return "READY";
        case STAT_BUSY:  return "BUSY";
        case STAT_DONE:  return "DONE";
        default:         return "???";
    }
}

// =============================================================================
//  GLOBÁLNÍ STAV
// =============================================================================

volatile uint8_t aktivni_prikaz = CMD_NOP;
volatile int16_t aktivni_param  = 0;
volatile bool    novy_prikaz    = false;
volatile uint8_t aktualni_stav  = STAT_READY;  // Co právě děláme

// =============================================================================
//  SESTAVENÍ STATUSU (přečte tlačítka + puky + stav)
// =============================================================================

RbcxStatus sestav_stav(int16_t extra_param = 0) {
    auto& btns = rb::Manager::get().buttons();
    
    RbcxStatus s;
    s.status = aktualni_stav;
    
    // Tlačítka jako bitová maska
    s.buttons = 0;
    if (btns.up())    s.buttons |= (1 << 0);  // bit 0
    if (btns.down())  s.buttons |= (1 << 1);  // bit 1
    if (btns.left())  s.buttons |= (1 << 2);  // bit 2
    if (btns.right()) s.buttons |= (1 << 3);  // bit 3
    
    s.pocet_puku = pocet_nasich_puku;
    s.param = extra_param;
    
    return s;
}

void posli_stav(int16_t extra_param = 0) {
    RbcxStatus s = sestav_stav(extra_param);
    rkUartSend(&s, sizeof(s));
}

// =============================================================================
//  UART VLÁKNO
// =============================================================================

void uart_vlakno(void *pvParameters) {
    EspCommand cmd;
    unsigned long posledni_stav = 0;
    unsigned long posledni_serial = 0;

    while (true) {
        // --- Příjem příkazů ---
        if (rkUartReceive(&cmd, sizeof(cmd))) {
            Serial.printf("\n>>> UART PRIJEM: %s  param=%d\n", cmd_name(cmd.cmd), cmd.param);

            if (cmd.cmd == CMD_NOP) {
                posli_stav();
            } else {
                zastav_jizdu = true;
                aktivni_prikaz = cmd.cmd;
                aktivni_param  = cmd.param;
                novy_prikaz    = true;
            }
        }

        // --- Periodické odesílání stavu přes UART (každých 200ms) ---
        if (millis() - posledni_stav > 200) {
            posledni_stav = millis();
            posli_stav();
        }

        // --- Periodický výpis na Serial Monitor (každou 1s) ---
        auto& btns = rb::Manager::get().buttons();
        if (millis() - posledni_serial > 1000) {
            posledni_serial = millis();
            Serial.printf("[STAV] %s | BTN: U=%d D=%d L=%d R=%d | puky=%d\n",
                stav_name(aktualni_stav),
                btns.up(), btns.down(), btns.left(), btns.right(),
                pocet_nasich_puku);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =============================================================================
//  SETUP + HLAVNÍ SMYČKA
// =============================================================================

void setup() {
    Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);
    delay(50);

    init_stepper();

    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    Wire.begin(21, 22, 400000);
    Wire.setTimeOut(1);
    rkColorSensorInit("front", Wire, tcs);

    rkUartInit();

    xTaskCreate(uart_vlakno, "UartVlakno", 4096, NULL, 2, NULL);

    rkLedGreen(true);
    Serial.println("=== RBCX READY ===");

    // =========================================================================
    //  HLAVNÍ SMYČKA
    // =========================================================================

    while (true) {

        if (novy_prikaz) {
            novy_prikaz = false;
            uint8_t cmd = aktivni_prikaz;
            int16_t param = aktivni_param;

            Serial.printf("\n========== PRIKAZ: %s  param=%d ==========\n", cmd_name(cmd), param);

            switch (cmd) {

                case CMD_STOP:
                    rkMotorsSetPower(0, 0);
                    rkLedYellow(false);
                    Serial.println(">> Motory zastaveny");
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;

                case CMD_JED_SBIREJ:
                    rkLedYellow(true);
                    zastav_jizdu = false;
                    aktualni_stav = STAT_BUSY;
                    Serial.printf(">> Jedu dopredu na %d%% a sbiram puky...\n", param);
                    
                    jed_a_sbirej((float)param);
                    
                    rkLedYellow(false);
                    Serial.printf(">> Zastaveno. Nasbirano %d nasich puku.\n", pocet_nasich_puku);
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;

                case CMD_OTOC_VLEVO:
                    aktualni_stav = STAT_BUSY;
                    Serial.printf(">> Otacim se VLEVO o %d stupnu...\n", param);
                    turn_on_spot_left((float)param, 30);
                    Serial.println(">> Otoceno VLEVO.");
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;

                case CMD_OTOC_VPRAVO:
                    aktualni_stav = STAT_BUSY;
                    Serial.printf(">> Otacim se VPRAVO o %d stupnu...\n", param);
                    turn_on_spot_right((float)param, 30);
                    Serial.println(">> Otoceno VPRAVO.");
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;

                case CMD_COUVEJ:
                    aktualni_stav = STAT_BUSY;
                    Serial.printf(">> Couvam o %d mm...\n", param);
                    backward_acc((float)param, 40);
                    Serial.println(">> Couvnuto.");
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;

                case CMD_VYLOZ:
                    aktualni_stav = STAT_BUSY;
                    Serial.println(">> Vykladam puky...");
                    otevri_nas();
                    delay(1000);
                    zavri_nas();
                    Serial.printf(">> Vylozeno %d puku. Reset.\n", pocet_nasich_puku);
                    pocet_nasich_puku = 0;
                    aktualni_stav = STAT_DONE;
                    posli_stav();
                    aktualni_stav = STAT_READY;
                    break;
            }

            aktivni_prikaz = CMD_NOP;
        }

        // === MANUÁLNÍ OVLÁDÁNÍ (testování bez ESP32) ===
        if (rkButtonLeft(true)) {
            delay(500);
            Serial.println("\n[MANUAL] <<< LEFT >>> jed_a_sbirej(60)");
            zastav_jizdu = false;
            aktualni_stav = STAT_BUSY;
            rkLedYellow(true);
            jed_a_sbirej(60);
            rkLedYellow(false);
            Serial.println("[MANUAL] Zastaveno.");
            aktualni_stav = STAT_READY;
        }

        if (rkButtonRight(true)) {
            delay(500);
            Serial.println("\n[MANUAL] <<< RIGHT >>> otoc vpravo 90");
            aktualni_stav = STAT_BUSY;
            turn_on_spot_right(90, 30);
            Serial.println("[MANUAL] Otoceno.");
            aktualni_stav = STAT_READY;
        }

        delay(50);
    }
}

void loop() {
    delay(1000);
}
