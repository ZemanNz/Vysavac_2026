#pragma once

// =============================================================================
//  test_senzory.h — Test všech bočních senzorů (3× laser, 3× ultrazvuk)
//
//  Nepotřebuje LiDAR ani RBCX. Jen inicializuje lasery a ultrazvuky
//  a neustále vypisuje hodnoty na Serial Monitor.
//
//  Zapnutí: v main.cpp USE_SENZORY=1 (ostatní na 0)
// =============================================================================

#include "lasery.h"
#include "ultrazvuky.h"

void test_senzory_init() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== TEST SENZORŮ ===");
    Serial.println("Inicializuji lasery...");
    init_lasery();
    Serial.println("Inicializuji ultrazvuky...");
    init_ultrazvuky();
    Serial.println("Hotovo! Měřím...");
    Serial.println("─────────────────────────────────────────────────────────────");
    Serial.println("         LASERY (mm)            |       ULTRAZVUKY (cm)");
    Serial.println("   L1       L2       L3         |   U1       U2       U3");
    Serial.println("─────────────────────────────────────────────────────────────");
}

void test_senzory_loop() {
    // Lasery
    int l1 = zmer_laser_mm(0);
    int l2 = zmer_laser_mm(1);
    int l3 = zmer_laser_mm(2);

    // Ultrazvuky
    float u1 = zmer_vzdalenost_cm(0);
    float u2 = zmer_vzdalenost_cm(1);
    float u3 = zmer_vzdalenost_cm(2);

    Serial.printf("L1=%4dmm  L2=%4dmm  L3=%4dmm  |  U1=%6.1fcm  U2=%6.1fcm  U3=%6.1fcm\n",
        l1, l2, l3, u1, u2, u3);

    delay(200);  // 5× za sekundu
}
