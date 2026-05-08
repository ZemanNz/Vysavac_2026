#include <Arduino.h>

// ============================================================
//  Přepínání režimu:
//
//    USE_VIZ    1  →  binární výstup pro Python vizualizátor (lidar.h)
//    USE_VIZ    0  →  textový výstup do Serial Monitoru (lidar_no_viz.h)
//
//    USE_MOZEK  1  →  zapnout rozhodovací logiku + UART k RBCX (mozek.h)
//    USE_SOUPER 1  →  testovací režim vyhýbání soupeři (souper.h)
//
//    Pozn: USE_SOUPER potřebuje mozek.h (UART, příkazy, otáčení...)
//          ale nepoužívá mozek_rozhoduj() — má vlastní logiku.
// ============================================================
#define USE_VIZ    0
#define USE_MOZEK  0
#define USE_TEST   0
#define USE_SOUPER 1  // Test soupeře (automaticky includne mozek.h)

#if USE_VIZ
    #include "lidar.h"
#else
    #include "lidar_no_viz.h"
#endif

// mozek.h se includne vždy, když je zapnutý MOZEK nebo SOUPER
#if (USE_MOZEK || USE_SOUPER) && !USE_VIZ
    #include "mozek.h"
    #if USE_MOZEK
        #include "test_prejezdu.h"
    #endif
    #if USE_SOUPER
        #include "souper.h"
    #endif
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

    #if (USE_MOZEK || USE_SOUPER) && !USE_VIZ
        #if USE_SOUPER
            souper_init();
        #elif USE_MOZEK
            mozek_init();
        #endif
    #endif

    #if USE_TEST && !USE_VIZ
        test_pohybu_init();
        test_pohybu_sekvence();
    #endif
}

void loop() {
    #if USE_VIZ
        loop_lidar();
    #else
        loop_lidar_nv();
    #endif

    #if (USE_MOZEK || USE_SOUPER) && !USE_VIZ
        #if USE_SOUPER
            souper_update();
        #elif USE_MOZEK
            mozek_update();
        #endif
    #endif

    #if USE_TEST && !USE_VIZ
    #endif
}

