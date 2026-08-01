#ifndef R2D_H
#define R2D_H

#include <stdbool.h>
#include <stdint.h>

float R2D_update(uint16_t adc[3], bool SDC_closed,
                 bool starter_btn_state, long now_ms);
void r2d_reset(void);
bool r2d_is_active(void);
bool r2d_sound_active(void);

long get_time_ms(void);
float APPS_pedal_press(uint16_t adc1, uint16_t adc2, long current_time);
void APPS_reset(void);
bool r2d_apps_fault(void);

#endif
