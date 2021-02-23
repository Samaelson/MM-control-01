#ifndef _MAIN_H
#define _MAIN_H


#include <inttypes.h>
#include <stdio.h>

void manual_extruder_selector();
void unrecoverable_error();
void drive_error();
void check_filament_not_present();
void signal_load_failure();
void signal_ok_after_load_failure();

void dbg(const char *fmt);
void dbg_val(const char *fmt, uint32_t val);
extern uint8_t tmc2130_mode;

extern FILE* uart_com;

extern bool IR_SENSOR;

#endif //_MAIN_H
