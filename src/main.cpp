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


    delay(1000);
    init_stepper();
    delay(1000);
    hledej_nulu_vpravo();

}

void loop() {
  // Přečte hodnoty ze senzorů
  uint16_t ir_levy = rkIrLeft();
  uint16_t ir_pravy = rkIrRight();

  // Vypíše hodnoty do sériového terminálu
  Serial.print("L: ");
  Serial.print(ir_levy);
  Serial.print(" | R: ");
  Serial.println(ir_pravy);

  // Pokud jsou to senzory s trimrem, pravděpodobně budou hlásit 
  // hodnoty blízko 0 (když něco vidí) a blízko 4095 (když nic nevidí), 
  // nebo naopak. Výpisem hned zjistíš, jak se reálně chovají.

  delay(1000); // Zastavení na 1 vteřinu, aby se dalo číst (klidně si můžeš snížit pro rychlejší odezvu)
}
