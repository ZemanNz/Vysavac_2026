#include <Arduino.h>
#include "robotka.h"

void setup() {
    rkConfig cfg;
    rkSetup(cfg);
    printf("Robotka test tlacitek a LED!\n");
}

void loop() {
    // Loop se opakuje do nekonecna
    
    // Pri stisknuti UP zapneme cervenou
    if (rkButtonIsPressed(BTN_UP)) {
        rkLedRed(true);
    } else {
        rkLedRed(false);
    }

    // Dalsi tlacitka (modrou vynechavame, protoze nefunguje)
    if (rkButtonIsPressed(BTN_DOWN)) {
        rkLedGreen(true);
    } else {
        rkLedGreen(false);
    }

    if (rkButtonIsPressed(BTN_LEFT) || rkButtonIsPressed(BTN_RIGHT)) {
        rkLedYellow(true);
    } else {
        rkLedYellow(false);
    }

    // Mala pauza
    delay(10); 
}

