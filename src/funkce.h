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