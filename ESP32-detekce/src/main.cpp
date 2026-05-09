#include <Arduino.h>

// ============================================================
//  Přepínání režimu:
//
//    USE_VIZ     1  →  binární výstup pro Python vizualizátor (lidar.h)
//    USE_VIZ     0  →  textový výstup do Serial Monitoru (lidar_no_viz.h)
//
//    USE_MOZEK   1  →  rozhodovací logika + UART k RBCX (mozek.h)
//    USE_SOUPER  1  →  test vyhýbání soupeři (souper.h)
//    USE_SENZORY 1  →  test laserů + ultrazvuků (BEZ LiDARu)
//    USE_TEST    1  →  test pohybu
// ============================================================
#define USE_VIZ     0
#define USE_MOZEK   0
#define USE_TEST    0
#define USE_SOUPER  0
#define USE_SENZORY 0  // Test senzorů (lasery + ultrazvuky, bez LiDARu)
#define USE_NAVRAT  1  // Test nouzového návratu

// --- Senzory (samostatný režim, nepotřebuje LiDAR ani RBCX) ---
#if USE_SENZORY
    #include "test_senzory.h"
#endif

// --- LiDAR ---
#if !USE_SENZORY
    #if USE_VIZ
        #include "lidar.h"
    #else
        #include "lidar_no_viz.h"
    #endif
#endif

// --- Mozek / Souper ---
#if (USE_MOZEK || USE_SOUPER) && !USE_VIZ && !USE_SENZORY
    #include "mozek.h"
    #if USE_MOZEK
        #include "test_prejezdu.h"
    #endif
    #if USE_SOUPER
        #include "souper.h"
    #endif
#endif

#if USE_TEST && !USE_VIZ && !USE_SENZORY
    #include "test_pohybu.h"
#endif

#if USE_NAVRAT && !USE_VIZ && !USE_SENZORY
    #include "test_navratu.h"
#endif

void setup() {
    #if USE_SENZORY
        test_senzory_init();
    #else
        // Lasery/UZ se MUSÍ inicializovat PŘED LiDARem (Serial2@230400 ruší I2C)
        #if (USE_MOZEK || USE_SOUPER) && !USE_VIZ
            #if USE_SOUPER
                souper_init();
            #elif USE_MOZEK
                mozek_init();
            #endif
        #endif

        #if USE_VIZ
            init_lidar();
        #else
            init_lidar_nv();
        #endif

        #if USE_TEST && !USE_VIZ
            test_pohybu_init();
            test_pohybu_sekvence();
        #endif

        #if USE_NAVRAT && !USE_VIZ
            test_navratu_init();
            test_navratu_sekvence();
        #endif
    #endif
}

void loop() {
    #if USE_SENZORY
        test_senzory_loop();
    #else
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
    #endif
}
