// @file
// @brief Low level stepper routines

#ifndef STEPPER_H
#define STEPPER_H

#include "config.h"
#include <inttypes.h>


enum State {
  Accelerate = 0,
  ConstVelocity = 1,
  Decelerate = 2,
};

extern int8_t filament_type[EXTRUDERS];
extern uint32_t GLOBAL_ACC;

extern uint16_t MAX_SPEED_SELECTOR; 
extern uint16_t MAX_SPEED_IDLER;

void home();
bool home_idler();
bool home_idler_smooth();

int get_idler_steps(int current_filament, int next_filament);
int get_selector_steps(int current_filament, int next_filament);

void park_idler(bool _unpark);

void do_pulley_step();
void set_pulley_dir_pull();
void set_pulley_dir_push();
void move_proportional(int _idler, int _selector);

bool move_smooth(uint8_t axis, int mm, int speed, bool rehomeOnFail = false,
                     bool withStallDetection = true, float ACC = GLOBAL_ACC,
                     bool withFindaDetection = false, bool withIR_SENSORDetection = false);

#endif //STEPPER_H
