// @author Marek Bel

#ifndef PINS_H
#define PINS_H
#include <Arduino.h>
#include <inttypes.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif //defined(__cplusplus)

extern void init_fs_butler();

extern bool get_fs_butler_status();

//const uint8_t selector_step_pin = 0x10;

extern void selector_step_pin_init(void);

extern void selector_step_pin_set(void);

extern void selector_step_pin_reset(void);


//const uint8_t idler_step_pin = 0x40;

extern void idler_step_pin_init(void);

extern void idler_step_pin_set(void);

extern void idler_step_pin_reset(void);


//const uint8_t pulley_step_pin = 0x10;

extern void pulley_step_pin_init(void);

extern void pulley_step_pin_set(void);

extern void pulley_step_pin_reset(void);

#if defined(__cplusplus)
}
#endif //defined(__cplusplus)
#endif /* PINS_H */
