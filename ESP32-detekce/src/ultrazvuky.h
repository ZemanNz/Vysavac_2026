#pragma once
#include <Arduino.h>

// Definice pinu
const int TRIG_PIN = 5;
const int ECHO_PINS[3] = {18, 19, 23};

void init_ultrazvuky() {
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    for (int i = 0; i < 3; i++) {
        pinMode(ECHO_PINS[i], INPUT);
    }
}

float zmer_vzdalenost_cm(int senzor_index) {
    if(senzor_index < 0 || senzor_index > 2) return -1.0;
    int echo_pin = ECHO_PINS[senzor_index];

    // Očistíme linku před odesláním zvuku
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Vyšleme spouštěcí impuls (10 mikrosekund)
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Standardní odchytnutí echa s timeoutem 30ms (~5 metrů max)
    long duration = pulseIn(echo_pin, HIGH, 30000);
    
    if (duration == 0) {
        return -1.0; // Nic před senzorem není / odletělo do prostoru
    }
    
    // Rychlost zvuku při 20°C je přibližně 343 m/s = 0.0343 cm/us
    // Trasa je dvojnásobná (tam a zpět), takže výsledek musíme vydělit 2
    float precision_distance = (duration * 0.0343) / 2.0;
    return precision_distance;
}
