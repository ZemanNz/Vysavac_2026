#pragma once

#include <Arduino.h>
#include "robotka.h"
#include "stepper_motor.h"


void otevri_nas(){
    rkServosSetPosition(4, 80); // Servo 1 nastaví na 90°
}

void zavri_nas(){
    rkServosSetPosition(4, 16); // Servo 1 nastaví na 0°
}

void otevri_souper(){
    rkServosSetPosition(3, -90);
}

void zavri_souper(){
    rkServosSetPosition(3, 0);
}

void srovnej_trididlo() {

    // Točí motorem proti směru hodinových ručiček, dokud senzor nevrátí 0 
    // (resp. pro absolutní bezpečí raději kontroluji aby tam nebyl ani šum > 10)
    while (rkIrRight() > 10) {
        rotaceProtiSmeru();
    }
    
    // Zastavíme a vrátíme původní stav
    vypni_civky(); // Povolit cívky, aby motor přestal krokovat


    rkServosSetPosition(2, 75);

    delay(400);


    int pocet_kroku = (60 * 64) / 45;
  
    for(int i=0;i<pocet_kroku;i++){
      rotaceProtiSmeru();
    }

    vypni_civky(); // Povolit cívky, aby motor přestal krokovat

    delay(300);

    rkServosSetPosition(2, 0);

    delay(10);
}

// Proměnná "nase_barva", kterou definujeme v main.cpp
extern char nase_barva;

char urci_barvu_puku(float &r, float &g, float &b) {
    // 1) Ochrana před falešnou detekcí na prázdno (hodnoty si bývají velmi blízké, např R:114, G:114, B:107)
    // Pokud je rozdíl mezi nejvyšší a nejnižší barvou malý, bereme to hned jako prázdno.

    Serial.print("R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);



    if ((abs(r - g) < 20 && abs(r - b) < 20 && abs(b - g) < 20) || (r > 110 && g > 110 && b > 110)) {
        return 'N';
    }

    // Detekce červeného puku (vysoká R složka a R musí výrazně převyšovat ostatní)
    if (r > 175 && r > g + 30 && r > b + 30) {
        return 'R';
    }
    
    // Detekce modrého puku (B složka musí být dominantní s větší bariérou vůči R a G)
    if (b > 95 && b > r + 20 && b > g + 12) {
        return 'B';
    }
    
    return 'N'; // Neznámá barva / prázdno (jakýkoliv okolní šum)
}

void roztrid_puk(char barva) {
    if (barva == nase_barva) {
        // Pokud je barva shodná, krokovej motor se otočí o 120°
        otoc_motorem(120, false);
    } else {
        // V opačném případě se otočí o 120° opačně
        otoc_motorem(120, true);
    }
}

// Funkce běžící v samostatném vlákně
void tridici_vlakno(void *pvParameters) {
    int pocitadlo_puku = 0;
    float r = 0, g = 0, b = 0;

    // První naměřené hodnoty senzoru bezprostředně po jeho zapnutí
    // bývají občas totální nesmysly (tzv. "garbage readings", např. 255/255/255 nebo 0.6).
    // Proto ho nejprve necháme 3x změřit barvu jen "na prázdno" a data zahodíme.
    for (int i = 0; i < 3; i++) {
        rkColorSensorGetRGB("front", &r, &g, &b);
        vTaskDelay(pdMS_TO_TICKS(150)); // Malé zpoždění, ať má senzor čas na reakci
    }

    while (true) {
        // Přečtení hodnot ze senzoru přímo ve funkci
        if (rkColorSensorGetRGB("front", &r, &g, &b)) {
            // ROZHODOVÁNÍ O BARVĚ PROBÍHÁ POUZE JEDNOU:
            char barva = urci_barvu_puku(r, g, b);
            
            // Pokud zachytíme reálný puk (barva NENÍ neznámá)
            if (barva != 'N') {
                // Pošleme TŘÍDÍCÍ FUNKCI POUZE PÍSMENO zjištěné barvy
                roztrid_puk(barva);
                
                pocitadlo_puku++;
                
                // Po 5 roztřídění zavoláme tvoji novou srovnávací funkci
                if (pocitadlo_puku >= 5) {
                    srovnej_trididlo();
                    pocitadlo_puku = 0; // Vynulovat pro další cyklus
                }
                
                // Počkáme chvilku po roztřídění, ať si třídič "oddechne" (sníženo na 100ms z 500ms)
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        // Zpoždění aby vlákno neběželo na 100% zátěži CPU (FreeRTOS)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
