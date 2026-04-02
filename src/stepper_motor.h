#pragma once
#include <Arduino.h>

const int in1 =  25;
const int in2 =  26;
const int in3 =  27;
const int in4 =  14;
int rychlost = 1; // vetsi cislo = nizsi rychlost

void init_stepper(){
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void krok1(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok2(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok3(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok4(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok5(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok6(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok7(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}
void krok8(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  vTaskDelay(pdMS_TO_TICKS(rychlost));
}

void rotacePoSmeru() {
  krok1();
  krok2();
  krok3();
  krok4();
  krok5();
  krok6();
  krok7();
  krok8();
}

void rotaceProtiSmeru() {
  krok8();
  krok7();
  krok6();
  krok5();
  krok4();
  krok3();
  krok2();
  krok1();
}

void vypni_civky() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void otoc_motorem(int uhel, bool proti_smeru){
  // Pokud jedeme po směru (proti_smeru == false), odečteme pár stupňů jako korekci proti přetáčení.
  float korekce = -0.3;
  if (!proti_smeru) {
      korekce = -0.2; // O kolik stupňů se to "přetáčí" – lze libovolně doladit
  }
  
  int pocet_kroku = ((uhel - korekce) * 64) / 45;
  
  if(proti_smeru){
    for(int i=0;i<pocet_kroku;i++){
      rotaceProtiSmeru();
    }
  }
  else{
    for(int i=0;i<pocet_kroku;i++){
      rotacePoSmeru();
    }
  }
  
  // Vypnutí cívek na konci může také pomoci s cuknutím/přetáčením na nejbližší magnetický krok
  vypni_civky();
}
