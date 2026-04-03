#include <Arduino.h>
#include "lidar.h"

// Ultrazvuky a lasery jsou fyzicky stále ve svých hlavičkových souborech "ultrazvuky.h" a "lasery.h",
// takže se k nim můžeme kdykoliv v budoucnu vrátit. Pro vizualizaci LiDaru na PC ale 
// potřebujeme čistý formát sériové linky, takže je tu teď schválně vypínám.

void setup() {
    // Inicializuje Lidar (a změní rychlost baudrate pro Serial0 na 921600 pro PC!)
    init_lidar();
}

void loop() {
    // Čte sériovou linku z Lidaru a sype čistá binární data do PC pro vizualizátor
    loop_lidar();
}
