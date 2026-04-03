#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robotka.h"
#include "stepper_motor.h"
#include "funkce.h"

// Zde se nastaví barva puku, kterou hledáme a sbíráme
char nase_barva = 'R';

// Vytvoření instance senzoru
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
    Serial.begin(115200);
    rkConfig cfg; 
    rkSetup(cfg);
    delay(50);

    init_stepper();

    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    
    // Zapnout I2C a inicializovat RGB senzor pod aliasem "front"
    Wire.begin(21, 22, 400000);
    Wire.setTimeOut(1);
    rkColorSensorInit("front", Wire, tcs);


    
    
    // Spuštění třídění v samostatném FreeRTOS vlákně
    xTaskCreate(
        tridici_vlakno,    // Funkce, která se má vykonávat (v funkce.h)
        "TridiciVlakno",   // Textový název pro debugging
        4096,              // Velikost zásobníku paměti
        NULL,              // Parametry
        1,                 // Priorita
        NULL               // Handle
    );
    
    // Potvrzení rozjezdu tasku rozsvícením zelené LED
    rkLedGreen(true);

    while(true) {
        if(rkButtonUp(true)) {
            forward(2000, 10);
            break;
        }
        if(rkButtonRight(true)) {
            forward(2000, 20);
            break;
        }
        if(rkButtonDown(true)) {
            srovnej_trididlo();
            // "Break" zde schválně nepíšu, abys mohl srovnat motor několikrát donekonečna,
            // než samotného robota pak odstartuješ přes nějaké jiné tlačítko.
        }
        if(rkButtonLeft(true)) {
            forward_acc(2000, 20);
            break;
        }
        delay(50);
    }

}

void loop() {
  delay(1000); // Zastavení na 1 vteřinu, aby se dalo číst (klidně si můžeš snížit pro rychlejší odezvu)
}
