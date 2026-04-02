#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robotka.h"
#include "stepper_motor.h"
#include "funkce.h"

// Zde se nastaví barva puku, kterou hledáme a sbíráme
char nase_barva = 'R';



void setup() {
    Serial.begin(115200);
    rkConfig cfg; 
    rkSetup(cfg);
    delay(50);
    
    otevri_nas();
    delay(3000);
    zavri_nas();
    delay(3000);

    rkServosSetPosition(3, 0);
    delay(1000);
    rkServosSetPosition(3, -90);
    delay(5000);
    rkServosSetPosition(3, 0);
    delay(1000);


}

void loop() {

  delay(1000); // Wait for a second before the next reading
}
