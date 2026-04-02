#include <Arduino.h>
#include "robotka.h"
#include "stepper_motor.h"

void setup() {
    rkConfig cfg;
    rkSetup(cfg);
    init_stepper(); // Inicializace krokoveho motoru
    printf("Robotka test tlacitek! Motor se otoci o 120 stupnu..\n");
}

void loop() {
    // Loop se opakuje do nekonecna
    
    // Pri stisknuti UP zapneme cervenou
    if (rkButtonIsPressed(BTN_UP)) {
        rkLedRed(true);
    } else {
        rkLedRed(false);
    }

    // Pri stisknuti DOWN zapneme zelenou
    if (rkButtonIsPressed(BTN_DOWN)) {
        rkLedGreen(true);
    } else {
        rkLedGreen(false);
    }

    // Tlacitka LEFT a RIGHT otoci motorem
    if (rkButtonIsPressed(BTN_LEFT)) {
        rkLedYellow(true);
        otoc_motorem(120, false); // po smeru hodinovych rucicek
        rkLedYellow(false);
        while(rkButtonIsPressed(BTN_LEFT)) delay(10); // Cekame na pusteni
    } 
    else if (rkButtonIsPressed(BTN_RIGHT)) {
        rkLedYellow(true);
        otoc_motorem(120, true); // proti smeru hodinovych rucicek
        rkLedYellow(false);
        while(rkButtonIsPressed(BTN_RIGHT)) delay(10); // Cekame na pusteni
    }

    // Mala pauza
    delay(10); 
}

