#pragma once

#include <Arduino.h>
#include "robotka.h"
#include "stepper_motor.h"


void otevri_nas(){
    rkServosSetPosition(4, 10); // Servo 1 nastaví na 90°
    delay(1200);
    rkServosSetPosition(4, 0);
}

void zavri_nas(){
    rkServosSetPosition(4, -10); // Servo 1 nastaví na 0°
    delay(1100);
    rkServosSetPosition(4, 0);
}

void otevri_souper(){
    rkServosSetPosition(3, -90);
}

void zavri_souper(){
    rkServosSetPosition(3, 0);
}

void hledej_nulu_vpravo() {

    // Točí motorem proti směru hodinových ručiček, dokud senzor nevrátí 0 
    // (resp. pro absolutní bezpečí raději kontroluji aby tam nebyl ani šum > 10)
    while (rkIrRight() > 10) {
        rotaceProtiSmeru();
    }
    
    // Zastavíme a vrátíme původní stav
    vypni_civky(); // Povolit cívky, aby motor přestal krokovat


    rkServosSetPosition(2, 5);
    delay(1000);
    rkServosSetPosition(2, 0);
    delay(10);

    int pocet_kroku = (50 * 64) / 45;
  
    for(int i=0;i<pocet_kroku;i++){
      rotaceProtiSmeru();
    }

    vypni_civky(); // Povolit cívky, aby motor přestal krokovat

    delay(100);

    rkServosSetPosition(2, -10);
    delay(1000);
    rkServosSetPosition(2, 0);
    delay(10);

    rkLedAll();
  

}