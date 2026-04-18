#pragma once

#include <Arduino.h>
#include "RBCX.h"
#include "funkce.h"

// =============================================================================
//  ASYNCHRONNÍ POHYB DOPŘEDU SE SBĚREM PUKŮ
// =============================================================================

// === KONFIGURACE — MUSÍ ODPOVÍDAT VAŠEMU ROBOTU (z rkConfig) ===
#define MOTOR_LEFT   rb::MotorId::M4
#define MOTOR_RIGHT  rb::MotorId::M1
#define POLARITY_LEFT  false
#define POLARITY_RIGHT true
#define MAX_TICKS_PER_SEC 5200

// Globální flagy
volatile bool zastav_jizdu = false;
volatile int pocet_nasich_puku = 0;

// Převod procent na ticky/s
static inline int16_t pct_to_speed(float pct) {
    return (int16_t)(pct * MAX_TICKS_PER_SEC / 100.0f);
}

/**
 * @brief Jede dopředu, třídí puky a zastaví se na příkaz nebo náraz.
 * @param speed  Cílová rychlost v % (0–100)
 */
void jed_a_sbirej(float speed) {
    auto& man = rb::Manager::get();
    auto& ml = man.motor(MOTOR_LEFT);
    auto& mr = man.motor(MOTOR_RIGHT);
    auto& btns = man.buttons();

    // === P-regulátor z forward_acc ===
    const float m_kp = 0.12f;
    const float m_min_speed = 18.0f;
    const float m_max_correction = 3.0f;

    // === Akcelerace/Decelerace ===
    const float accel_step = 0.5f;   // plynulý rozjezd
    const float decel_step = 1.5f;   // rychlé zastavení

    // === Rychlosti s polaritou ===
    const float base_speed_left  = POLARITY_LEFT  ? -speed : speed;
    const float base_speed_right = POLARITY_RIGHT ? -speed : speed;

    float current_speed_left  = (base_speed_left  > 0) ? m_min_speed : -m_min_speed;
    float current_speed_right = (base_speed_right > 0) ? m_min_speed : -m_min_speed;

    const float step_accel_left  = (base_speed_left  > 0) ? accel_step : -accel_step;
    const float step_accel_right = (base_speed_right > 0) ? accel_step : -accel_step;
    const float step_decel_left  = (base_speed_left  > 0) ? decel_step : -decel_step;
    const float step_decel_right = (base_speed_right > 0) ? decel_step : -decel_step;

    // Reset enkodérů
    ml.setCurrentPosition(0);
    mr.setCurrentPosition(0);

    int left_pos = 0;
    int right_pos = 0;

    // Fáze
    enum Phase { ACCELERATE, CRUISE, DECELERATE, STOPPED };
    Phase phase = ACCELERATE;

    // Třídění
    int tridici_counter = 0;
    int lokalni_pocitadlo = 0;
    float r = 0, g = 0, b = 0;

    zastav_jizdu = false;

    Serial.println("=== JED_A_SBIREJ START ===");

    // === HLAVNÍ SMYČKA ===
    while (true) {

        // --- 1) ČTENÍ ENKODÉRŮ ---
        ml.requestInfo([&](rb::Motor& info) {
            left_pos = info.position();
        });
        mr.requestInfo([&](rb::Motor& info) {
            right_pos = info.position();
        });

        delay(10);

        // --- 2) STOP PODMÍNKY ---
        // Příkaz z venčí
        if (zastav_jizdu && phase != DECELERATE && phase != STOPPED) {
            Serial.println(">> STOP z UART");
            phase = DECELERATE;
        }

        // Jedno tlačítko stačí → decelerate okamžitě
        if ((btns.up() || btns.down()) && phase != DECELERATE && phase != STOPPED) {
            Serial.printf(">> BTN HIT (up=%d down=%d) -> DECELERATE\n", btns.up(), btns.down());
            phase = DECELERATE;
        }

        // --- 3) ŘÍZENÍ RYCHLOSTI ---
        switch (phase) {
            case ACCELERATE:
                if (abs(current_speed_left) < abs(speed)) {
                    current_speed_left += step_accel_left;
                }
                if (abs(current_speed_right) < abs(speed)) {
                    current_speed_right += step_accel_right;
                }
                if (abs(current_speed_left) >= abs(speed) && abs(current_speed_right) >= abs(speed)) {
                    current_speed_left = base_speed_left;
                    current_speed_right = base_speed_right;
                    phase = CRUISE;
                    Serial.println(">> CRUISE");
                }
                break;

            case CRUISE:
                current_speed_left = base_speed_left;
                current_speed_right = base_speed_right;
                break;

            case DECELERATE:
                if (abs(current_speed_left) > m_min_speed) {
                    current_speed_left -= step_decel_left;
                }
                if (abs(current_speed_right) > m_min_speed) {
                    current_speed_right -= step_decel_right;
                }
                if (abs(current_speed_left) <= m_min_speed && abs(current_speed_right) <= m_min_speed) {
                    phase = STOPPED;
                    Serial.println(">> STOPPED");
                }
                break;

            case STOPPED:
                break;
        }

        if (phase == STOPPED) {
            ml.speed(0);
            mr.speed(0);
            ml.power(0);
            mr.power(0);
            break;
        }

        float speed_left  = current_speed_left;
        float speed_right = current_speed_right;

        // --- 4) P-REGULÁTOR — POUZE V CRUISE FÁZI ---
        if (phase == CRUISE) {
            float progres_left  = (float)abs(left_pos);
            float progres_right = (float)abs(right_pos);
            float sum = progres_left + progres_right + 1.0f;
            float rozdil_progres = (progres_left / sum) - (progres_right / sum);

            float correction = rozdil_progres * m_kp * 1800;
            correction = std::max(-m_max_correction, std::min(correction, m_max_correction));

            // Aplikace korekce — PŘESNĚ JAKO V forward_acc
            if (correction > 0) {
                if (POLARITY_LEFT) {
                    speed_left  += correction;
                    speed_right += correction;
                } else {
                    speed_left  -= correction;
                    speed_right -= correction;
                }
            } else if (correction < 0) {
                if (POLARITY_RIGHT) {
                    speed_right -= correction;
                    speed_left  -= correction;
                } else {
                    speed_right += correction;
                    speed_left  += correction;
                }
            }

            // Minimální rychlost
            if (abs(speed_left) < m_min_speed) {
                speed_left = (speed_left > 0) ? m_min_speed : -m_min_speed;
            }
            if (abs(speed_right) < m_min_speed) {
                speed_right = (speed_right > 0) ? m_min_speed : -m_min_speed;
            }
        }

        // --- 5) NASTAVENÍ MOTORŮ ---
        ml.speed(pct_to_speed(speed_left));
        mr.speed(pct_to_speed(speed_right));

        // --- 6) TŘÍDĚNÍ PUKŮ ---
        tridici_counter++;
        if (tridici_counter >= 10) {
            tridici_counter = 0;

            bool sensor_ok = rkColorSensorGetRGB("front", &r, &g, &b);

            // DEBUG výpis
            static int debug_cnt = 0;
            debug_cnt++;
            if (debug_cnt % 5 == 0) {
                Serial.printf("TRID: sensor=%s  R:%.0f G:%.0f B:%.0f  phase:%d  sL:%.1f sR:%.1f  L:%d R:%d\n",
                    sensor_ok ? "OK" : "FAIL", r, g, b, phase, current_speed_left, current_speed_right, left_pos, right_pos);
            }

            if (sensor_ok) {
                char barva = urci_barvu_puku(r, g, b);
                if (barva != 'N') {
                    Serial.printf(">> PUK: %c\n", barva);
                    roztrid_puk(barva);
                    if (barva == nase_barva) {
                        pocet_nasich_puku++;
                    }
                    lokalni_pocitadlo++;
                    if (lokalni_pocitadlo >= 5) {
                        srovnej_trididlo();
                        lokalni_pocitadlo = 0;
                    }
                }
            }
        }
    }

    Serial.printf("=== KONEC === nasich: %d\n", pocet_nasich_puku);
}
