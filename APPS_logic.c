// APPS_logic.c  — Application Layer (no hardware code)
// FSAE APPS plausibility checker
//
// S1 (AIN0): normal   — ADC 0=0V,    ADC 4095=3.3V
// S2 (AIN1): inverted — ADC 4095=0V, ADC 0=3.3V
// Rule: |S1_V + S2_V - 3.3| must stay < 0.33V (10%)
//       if it exceeds for >100ms → latch fault, zero torque

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

#define ADC_MAX_VAL            4095.0f
#define SYS_VOLTAGE            3.3f
#define MAX_DEV                0.33f
#define IMPLAUSIBILITY_TIME_MS 100L

static bool fault_active       = false;
static long implaus_start_time = 0;

// Monotonic clock — used by HAL layer too
long get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)(ts.tv_sec * 1000L + ts.tv_nsec / 1000000L);
}

// Returns pedal position [0.0–1.0] or 0.0 on fault
float APPS_pedal_press(uint16_t adc1, uint16_t adc2, long current_time) {
    if (fault_active) return 0.0f;

    float S1_V = (adc1 / ADC_MAX_VAL) * SYS_VOLTAGE;
    float S2_V = (1.0f - adc2 / ADC_MAX_VAL) * SYS_VOLTAGE;

    bool implausible = (fabsf(S1_V + S2_V - SYS_VOLTAGE) > MAX_DEV);

    if (implausible) {
        if (implaus_start_time == 0) {
            implaus_start_time = current_time;          // start 100ms timer
        } else if ((current_time - implaus_start_time) >= IMPLAUSIBILITY_TIME_MS) {
            fault_active       = true;                  // latch fault
            implaus_start_time = 0;
            return 0.0f;
        }
        // still inside 100ms window — fall through, return best estimate
    } else {
        implaus_start_time = 0;                         // signal healthy, clear timer
    }

    float pedal = S1_V / SYS_VOLTAGE;
    if (pedal > 1.0f) pedal = 1.0f;
    if (pedal < 0.0f) pedal = 0.0f;
    return pedal;
}

void APPS_reset(void) {
    fault_active       = false;
    implaus_start_time = 0;
}

bool r2d_apps_fault(void) { return fault_active; }
