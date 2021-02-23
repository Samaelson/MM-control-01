

#include <Arduino.h>
#include <stdint.h>
#include <avr/io.h>
#include "pins.h"

//#define PIN_D7 1 // D7 -> PE6

//const uint8_t D7_PIN = 0xBF;

#define PINE6 6
#define PIND0 0

void init_fs_guard()
{
//  DDRE &= ~(1 << PINE6); // configure D7 as input
  DDRD &= ~(1 << PIND0); // configure SCL as input
}

bool get_fs_guard_status()
{
//  if((PINE & (1 << PINE6))) return false;
  if((PIND & (1 << PIND0))) return false;
  else return true;
}

const uint8_t selector_step_pin = 0x10;

void selector_step_pin_init(void)
{
    DDRD |= selector_step_pin;
}

void selector_step_pin_set(void)
{
    PORTD |= selector_step_pin;
}
void selector_step_pin_reset(void)
{
    PORTD &= ~selector_step_pin;
}

const uint8_t idler_step_pin = 0x40;

void idler_step_pin_init(void)
{
    DDRD |= idler_step_pin;
}

void idler_step_pin_set(void)
{
    PORTD |= idler_step_pin;
}

void idler_step_pin_reset(void)
{
    PORTD &= ~idler_step_pin;
}

const uint8_t pulley_step_pin = 0x10;

void pulley_step_pin_init(void)
{
    DDRB |= pulley_step_pin;
}
void pulley_step_pin_set(void)
{
    PORTB |= pulley_step_pin;
}
void pulley_step_pin_reset(void)
{
    PORTB &= ~pulley_step_pin;
}
