#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Definice pinů definovaných uživatelem
#define XSHUT_1 25
#define XSHUT_2 26
#define XSHUT_3 27

// I2C adresy pro jednotlivé senzory (výchozí u VL53L0X je vždy 0x29)
// Aby se na jedné sběrnici nehádaly, druhý a třetí po zapnutí hned přenastavíme na jiné adresy
#define ADRESA_LASER_1 0x30
#define ADRESA_LASER_2 0x31
#define ADRESA_LASER_3 0x32

// Fyzické instance senzorů do paměti ESPčka
Adafruit_VL53L0X laser1 = Adafruit_VL53L0X();
Adafruit_VL53L0X laser2 = Adafruit_VL53L0X();
Adafruit_VL53L0X laser3 = Adafruit_VL53L0X();

void init_lasery() {
    Serial.println("Startuji I2C lasery na sběrnici...");

    // 1) Nastavíme všechny XSHUT piny na výstup a pro jistotu je vypneme nízko,
    // čímž všechny 3 senzory fyzicky vyresetujeme (uvedeme do kómatu)
    pinMode(XSHUT_1, OUTPUT);
    pinMode(XSHUT_2, OUTPUT);
    pinMode(XSHUT_3, OUTPUT);
    
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(10); // Necháme je chvilku kompletně de-initializovat 
    
    // Potenciálně nutný double-check stavu - uvolnění (ale ještě se nespustí)
    // Tuto logiku VL53L0X často doporučuje
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(10);

    // ------------------------------------
    // INICIALIZACE 1. LASERU
    // ------------------------------------
    digitalWrite(XSHUT_1, HIGH); // Fyzicky zapneme pouze první senzor
    delay(10);
    
    // Protože jede samotný, poslouchá na 0x29. Funkce begin() vnitřně sama 
    // převede jeho naslouchání na novou adresu (0x30), kterou jí podstrčíme.
    if (!laser1.begin(ADRESA_LASER_1)) {
        Serial.println("Chyba: Nepodařilo se najít Laser 1!");
    } else {
        Serial.println("Laser 1 úspěšně inicializován.");
    }
    
    // ------------------------------------
    // INICIALIZACE 2. LASERU
    // ------------------------------------
    digitalWrite(XSHUT_2, HIGH); // Zapneme 2. senzor, probudí se na 0x29.
    delay(10);
    
    // 1. senzor už je na 0x30, takže se mu do toho neplete. Převedeme 2. senzor na 0x31.
    if (!laser2.begin(ADRESA_LASER_2)) {
        Serial.println("Chyba: Nepodařilo se najít Laser 2!");
    } else {
        Serial.println("Laser 2 úspěšně inicializován.");
    }

    // ------------------------------------
    // INICIALIZACE 3. LASERU
    // ------------------------------------
    digitalWrite(XSHUT_3, HIGH); // Zapneme 3. senzor, probudí se na 0x29
    delay(10);
    
    // Převedeme 3. senzor na 0x32
    if (!laser3.begin(ADRESA_LASER_3)) {
        Serial.println("Chyba: Nepodařilo se najít Laser 3!");
    } else {
        Serial.println("Laser 3 úspěšně inicializován.");
    }
}

int zmer_laser_mm(int index) {
    VL53L0X_RangingMeasurementData_t odpoved;
    
    // Knihovna vrátí status o úspěšnosti zaměření a vyplní "odpoved"
    if (index == 0) {
        laser1.rangingTest(&odpoved, false);
    } else if (index == 1) {
        laser2.rangingTest(&odpoved, false);
    } else if (index == 2) {
        laser3.rangingTest(&odpoved, false);
    } else {
        return -1;
    }
    
    // Pokud je hodnota RangeStatus cokoliv jiného než 4 (4 ušitě znamená "Phase Fail" = na nic v dosahu nenarazil),
    // bereme hodnotu jako stabilní měření
    if (odpoved.RangeStatus != 4) {
        return odpoved.RangeMilliMeter;
    }
    
    return -1; // -1 odpovídá tomu, že nevidí překážku v limitu dvou metrů
}
