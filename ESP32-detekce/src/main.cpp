#include <Arduino.h>

// ============================================================
//  Přepínání režimu:
//    USE_VIZ  1  →  binární výstup pro Python vizualizátor (lidar.h)
//    USE_VIZ  0  →  textový výstup do Serial Monitoru (lidar_no_viz.h)
// ============================================================
#define USE_VIZ  0

#if USE_VIZ
    #include "lidar.h"
#else
    #include "lidar_no_viz.h"
#endif

void setup() {
    #if USE_VIZ
        init_lidar();
    #else
        init_lidar_nv();
    #endif
}

void loop() {
    #if USE_VIZ
        loop_lidar();
    #else
        loop_lidar_nv();
    #endif
}
