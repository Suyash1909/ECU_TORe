// R2D_logic.c  — Application Layer (no hardware code)
// Ready-to-Drive logic — FSAE compliant
//
// R2D sequence:
//   1. SDC must be closed (TSMS + LVMS + all interlocks)
//   2. No APPS fault
//   3. Brake pedal pressed > 25% (adc[0] > BPPS_R2D_THRESHOLD)
//   4. Starter button pressed then RELEASED (falling edge)
//   → On R2D: sound buzzer for 1.5 seconds, then enable torque

#include <stdint.h>
#include <stdbool.h>

#define ADC_MAX                4095
#define BPPS_R2D_THRESHOLD     1025    // ~25% of 4095
#define APPS_DEADBAND          205     // ~5% of 4095 — below this pedal = 0 torque
#define MAX_TORQUE_NM          29     // adjust to your motor spec
#define R2D_SOUND_TIME_MS      1500L   // buzzer duration ms

// FIX BUG1+2: removed 'static long time(void)' — conflicts with stdlib
//             now_time is passed in as a parameter from the HAL layer
// FIX BUG3:   now_time parameter is long, not bool

// Extern declarations — implemented in APPS_logic.c
extern float APPS_pedal_press(uint16_t adc1, uint16_t adc2, long current_time);
extern bool  r2d_apps_fault(void);
extern void  APPS_reset(void);

// Internal state
static bool vehicle_is_r2d        = false;
static bool sound_active           = false;
static bool starter_button_pressed = false;
static long sound_start_time       = 0;

// ── Getters for HAL layer ─────────────────────────────────────────────────────
bool r2d_is_active(void)     { return vehicle_is_r2d; }
bool r2d_sound_active(void)  { return sound_active;   }

// ── Main R2D update — call every cycle from HAL ───────────────────────────────
// adc[0] = BPPS (brake)
// adc[1] = APPS S1
// adc[2] = APPS S2
// SDC_closed       : true when full shutdown circuit is closed
// starter_btn_state: raw button pin level (true = pressed/held)
// now_ms           : monotonic time from get_time_ms()
//
// Returns: torque command 0–MAX_TORQUE_NM (0 when not R2D or fault)

float R2D_update(uint16_t adc[3], bool SDC_closed,
                 bool starter_btn_state, long now_ms) {

    // ── 1. SDC check — any interlock open → drop out of R2D immediately ───────
    if (!SDC_closed) {
        vehicle_is_r2d = false;
        sound_active   = false;
        APPS_reset();
        return 0.0f;
    }

    // ── 2. APPS plausibility check ────────────────────────────────────────────
    float pedal = APPS_pedal_press(adc[1], adc[2], now_ms);

    // FIX BUG4: was 'if(r2d_apps_fault)' — checked pointer, always true
    //           now correctly calls the function with ()
    if (r2d_apps_fault()) {
        vehicle_is_r2d = false;
        return 0.0f;
    }

    // ── 3. Starter button falling edge detection (pressed → released) ─────────
    bool falling_edge = (starter_button_pressed && !starter_btn_state);
    starter_button_pressed = starter_btn_state;   // update state for next cycle

    // ── 4. R2D entry condition ────────────────────────────────────────────────
    // Trigger on button release AND brake > 25% AND not already R2D
    if (falling_edge && (adc[0] > BPPS_R2D_THRESHOLD) && !vehicle_is_r2d) {
        vehicle_is_r2d  = true;
        sound_active    = true;
        sound_start_time = now_ms;
    }

    // ── 5. Buzzer timeout ─────────────────────────────────────────────────────
    if (sound_active && (now_ms - sound_start_time) >= R2D_SOUND_TIME_MS) {
        sound_active = false;
    }

    // ── 6. Torque output ──────────────────────────────────────────────────────
    // The R2D sound must complete before torque is enabled.
    if (!vehicle_is_r2d || sound_active) return 0.0f;

    // Apply APPS deadband — small pedal noise = zero torque
    if (adc[1] < APPS_DEADBAND) return 0.0f;

    // Scale pedal position to torque
    float torque = pedal * (float)MAX_TORQUE_NM;
    return torque;
}

// ── Full system reset (e.g. on power cycle or driver reset button) ────────────
void r2d_reset(void) {
    vehicle_is_r2d         = false;
    sound_active           = false;
    sound_start_time       = 0;
    starter_button_pressed = false;
    // FIX BUG5: was 'APPS_reset;' — missing (), function was never called
    APPS_reset();
}
