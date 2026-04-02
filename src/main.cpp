#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "robotka.h"
#include "stepper_motor.h"

// Zde se nastaví barva puku, kterou hledáme a sbíráme
char nase_barva = 'R';

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float r, g, b;

void setup() {
    Serial.begin(115200);
    rkConfig cfg; 
    rkSetup(cfg);
    delay(50);
    
    // Inicializace krokového motoru
    init_stepper();

    pinMode(21, PULLUP);
    pinMode(22, PULLUP);
  
    // 1) Spust obě I2C sběrnice
    Wire.begin(21, 22, 400000);
    Wire.setTimeOut(1);
    // 2) Inicializuj senzory:
    // Initialize the color sensor with a unique name and the I2C bus
    rkColorSensorInit("front", Wire, tcs);
}

void loop() {
  // Retrieve RGB values from the sensor named "front"
  if (rkColorSensorGetRGB("front", &r, &g, &b)) {
    Serial.print("R: "); Serial.print(r, 3);
    Serial.print(" G: "); Serial.print(g, 3);
    Serial.print(" B: "); Serial.println(b, 3);
    
    // Zpracování barvy puku a otočení motorem přes předanou referenci hodnot
    roztrid_puk(r, g, b);
  } else {
    Serial.println("Sensor 'front' not found.");
  }

  delay(1000); // Wait for a second before the next reading
}
