#pragma once
#include "mozek.h"

/**
 * Testování přejezdu o 20 cm s využitím ENKODÉRŮ přímo na RBCX.
 * ESP32 jen pošle příkaz a pak sleduje výsledek.
 */
void test_prejezdu_akce() {
    Serial.println("\n=== START TESTU PREJEZDU (20 cm - ENKODERY) ===");
    
    // Stabilizace dat
    for(int i=0; i<5; i++) {
        loop_lidar_nv();
        mozek_aktualizuj_senzory();
        delay(50);
    }

    float start_lidar = senzory.dist_vpredu;
    float start_y = senzory.pozice_y;
    
    Serial.printf("START: Lidar(FRONT)=%.1f, Y=%.1f\n", start_lidar, start_y);
    Serial.println("Posílám příkaz: JED_SBIREJ rychlost=35%, vzdálenost=200mm");
    
    // Nový protokol: CMD, Rychlost, Vzdálenost
    posli_prikaz(CMD_JED_SBIREJ, 35, 200); 
    
    unsigned long start_time = millis();

    Serial.println("Čekám na rbcx_hotovo (RBCX si to řídí samo)...");
    while (!rbcx_hotovo()) {
        loop_lidar_nv();
        mozek_aktualizuj_senzory();
        
        // Jen pro info vypisujeme co si myslíme my
        if (millis() % 500 == 0) {
            Serial.printf("  [Test] Ujeto dle SLAM: %.1f mm\n", fabsf(start_y - senzory.pozice_y));
        }

        if (millis() - start_time > 10000) {
            posli_prikaz(CMD_STOP);
            Serial.println("!! TIMEOUT.");
            break;
        }
        delay(20);
    }
    
    Serial.println("\n=== VÝSLEDEK (Enkodéry zastavily) ===");
    Serial.printf("REÁLNĚ UJETO LIDAR: %.1f mm\n", start_lidar - senzory.dist_vpredu);
    Serial.printf("REÁLNĚ UJETO SLAM:  %.1f mm\n", fabsf(start_y - senzory.pozice_y));
    Serial.println("================\n");
}
