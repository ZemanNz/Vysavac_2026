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

void otoc_motorem(int uhel, bool ruka_po_smeru){
  int pocet_kroku = (uhel * 64) / 45;
  if(ruka_po_smeru){
    for(int i=0;i<pocet_kroku;i++){
      rotaceProtiSmeru();
    }
  }
  else{
    for(int i=0;i<pocet_kroku;i++){
      rotacePoSmeru();
    }
  }
}
