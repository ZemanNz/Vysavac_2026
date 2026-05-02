#include <Arduino.h>

// ============================================================
//  Přepínání režimu:
//
//    USE_VIZ    1  →  binární výstup pro Python vizualizátor (lidar.h)
//    USE_VIZ    0  →  textový výstup do Serial Monitoru (lidar_no_viz.h)
//
//    USE_MOZEK  1  →  zapnout rozhodovací logiku + UART k RBCX (mozek.h)
//                     (vyžaduje USE_VIZ 0)
// ============================================================
#define USE_VIZ    0
#define USE_MOZEK  0 // Vypneme mozek pro testovani
#define USE_TEST   1 // Zapneme testovani pohybu

#if USE_VIZ
    #include "lidar.h"
#else
    #include "lidar_no_viz.h"
#endif

#if USE_MOZEK && !USE_VIZ
    #include "mozek.h"
#endif

#if USE_TEST && !USE_VIZ
    #include "test_pohybu.h"
#endif

void setup() {
    #if USE_VIZ
        init_lidar();
    #else
        init_lidar_nv();
    #endif

    #if USE_MOZEK && !USE_VIZ
        mozek_init();
    #endif

    #if USE_TEST && !USE_VIZ
        test_pohybu_init();
    #endif
}

void loop() {
    #if USE_VIZ
        loop_lidar();
    #else
        loop_lidar_nv();
    #endif

    #if USE_MOZEK && !USE_VIZ
        mozek_update();
    #endif

    #if USE_TEST && !USE_VIZ
        test_pohybu_update();
    #endif
}
