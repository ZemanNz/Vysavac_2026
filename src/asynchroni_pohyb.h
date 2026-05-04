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
volatile float g_lidar_error = 0.0f;

bool byly_tlacitka = false;

extern rkConfig cfg; 

const float m_wheel_circumference = cfg.motor_wheel_diameter * M_PI; 


// Převod procent na ticky/s
static inline int16_t pct_to_speed(float pct) {
    return (int16_t)(pct * MAX_TICKS_PER_SEC / 100.0f);
}

float ticksToMm(int32_t ticks) {
    // Použijeme průměr kola přímo z konfigurace (default 61mm)
    float circumference = cfg.motor_wheel_diameter * M_PI;
    if (cfg.prevod_motoru < 1.0f) return 0; // Ochrana proti dělení nulou
    return (float(ticks) / cfg.prevod_motoru) * circumference;
}

/**
 * @brief Jede dopředu, třídí puky a zastaví se na příkaz nebo náraz.
 * @param speed  Cílová rychlost v % (0–100)
 */
void jed_a_sbirej(float speed, int mm = 0) {
    auto& man = rb::Manager::get();
    auto& ml = man.motor(MOTOR_LEFT);
    auto& mr = man.motor(MOTOR_RIGHT);
    auto& btns = man.buttons();

    // === Kombinovaný regulátor (Enkodéry + LiDAR) ===
    const float m_kp = 0.12f;          // P-složka pro enkodéry (rychlá odezva)
    const float m_lidar_kp = 0.5f;     // P-složka pro LiDAR (pomalá korekce absolutního úhlu)
    const float m_min_speed = 18.0f;
    const float m_max_correction = 3.0f;

    // === Akcelerace/Decelerace ===
    const float accel_step = 0.5f;   // plynulý rozjezd
    const float decel_step = 2.0f;   // plynulé ale rychlé zastavení (původně 1.5, pak 10, nyní zpět na 10 pro přesnost)

    // === Rychlosti s polaritou ===
    float base_speed_left  = POLARITY_LEFT  ? -speed : speed;
    float base_speed_right = POLARITY_RIGHT ? -speed : speed;

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

    g_lidar_error = 0.0f; // Reset LiDAR odchylky před startem jízdy

    // Fáze
    enum Phase { ACCELERATE, CRUISE, DECELERATE, STOPPED };
    Phase phase = ACCELERATE;

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
            byly_tlacitka = true;
            phase = DECELERATE;
        }

        if (mm != 0) {
            float ujeto = max(ticksToMm(left_pos), ticksToMm(right_pos));
            
            Serial.printf("ujeto: %.1f mm\n", ujeto);
            // 1) Úplný cíl
            if (ujeto >= mm) {
                if (phase != DECELERATE && phase != STOPPED) {
                    Serial.printf(">> CIL DOSAZEN (%.1f mm) -> STOP\n", ujeto);
                    phase = DECELERATE;
                }
            } 
            // 2) Pomalý dojezd (8 cm před cílem zpomalíme na 15 %)
            else if (ujeto >= (mm - 80) && speed > 15.0f) {
                if (phase == CRUISE) {
                    Serial.println(">> DOJEZD: Zpomaluji na 15 %");
                    speed = 15.0f;
                    // base_speed se přepočítá v dalším switchi (pokud ho uděláme ne-const)
                }
            }
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
                    phase = CRUISE;
                    Serial.println(">> CRUISE");
                }
                break;

            case CRUISE:
                // Přepočítáme základní rychlosti (pokud se změnilo 'speed')
                base_speed_left  = POLARITY_LEFT  ? -speed : speed;
                base_speed_right = POLARITY_RIGHT ? -speed : speed;
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

        // --- 4) KOMBINOVANÝ REGULÁTOR — POUZE V CRUISE FÁZI ---
        if (phase == CRUISE && (mm == 0)) {
            // 1. Klasický P-regulátor z enkodérů (drží kola ve stejných otáčkách)
            float progres_left  = (float)abs(left_pos);
            float progres_right = (float)abs(right_pos);
            float sum = progres_left + progres_right + 1.0f;
            float rozdil_progres = (progres_left / sum) - (progres_right / sum);
            float enc_correction = rozdil_progres * m_kp * 1800.0f;

            // 2. Dlouhodobá složka z LiDARu (pomalá korekce absolutního úhlu)
            // Kladný g_lidar_error = cíl je vpravo -> potřebujeme zápornou korekci k zatočení vpravo
            float lid_correction = -(g_lidar_error * m_lidar_kp); 

            // 3. Sečtení a oříznutí
            float correction = enc_correction + lid_correction;
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

    }
    Serial.printf("=== KONEC === nasich: %d\n", pocet_nasich_puku);
}
