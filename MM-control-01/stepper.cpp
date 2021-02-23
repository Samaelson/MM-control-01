#include "stepper.h"
#include "shr16.h"
#include "tmc2130.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <Arduino.h>
#include "main.h"
#include "mmctl.h"
#include "Buttons.h"
#include "permanent_storage.h"
#include "pins.h"
#include "tmc2130.h"
#include "motion.h"

// public variables:
//BowdenLength bowdenLength;
//uint16_t BOWDEN_LENGTH = bowdenLength.get();
uint16_t MAX_SPEED_SELECTOR =  MAX_SPEED_SEL_DEF_STEALTH; // micro steps
uint16_t MAX_SPEED_IDLER    =  MAX_SPEED_IDL_DEF_STEALTH; // micro steps
uint32_t GLOBAL_ACC         =  GLOBAL_ACC_DEF_STEALTH; // micro steps / s²

int MIN_SPEED = 200;

uint8_t selSGFailCount = 0;
uint8_t idlSGFailCount = 0;

int8_t filament_type[EXTRUDERS] = {-1, -1, -1, -1, -1};
static bool isIdlerParked = false;

static const int selector_steps_after_homing = -3700;
static const int idler_steps_after_homing = -130;

static const int selector_steps = 2790/4;
static const int idler_steps = 1420 / 4;    // 2 msteps = 180 / 4
static const int idler_parking_steps = (idler_steps / 2) + 40;  // 40

static bool home_selector_smooth();

static int set_idler_direction(int _steps);
static int set_selector_direction(int _steps);
static int set_pulley_direction(int _steps);
static void set_idler_dir_down();
static void set_idler_dir_up();
static void move(int _idler, int _selector, int _pulley);

static void move_idl(int steps_idl,int speed=MIN_SPEED, bool sgdetect=false, float acc=GLOBAL_ACC);
static void move_sm(int steps_idl, int steps_sel, int steps_pul, int speed=MIN_SPEED, bool sgdetect=false, float acc=GLOBAL_ACC);

//! @brief Compute steps for selector needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return selector steps
int get_selector_steps(int current_filament, int next_filament)
{
    return (((current_filament - next_filament) * selector_steps) * -1);
}

//! @brief Compute steps for idler needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return idler steps
int get_idler_steps(int current_filament, int next_filament)
{
    return ((current_filament - next_filament) * idler_steps);
}

void do_pulley_step()
{
  pulley_step_pin_set();
	asm("nop");
	pulley_step_pin_reset();
	asm("nop");
}



//! @brief home idler
//!
//! @param toLastFilament
//!   - true
//! Move idler to previously loaded filament and disengage. Returns true.
//! Does nothing if last filament used is not known and returns false.
//!   - false (default)
//! Move idler to filament 0 and disengage. Returns true.
//!
//! @retval true Succeeded
//! @retval false Failed
bool home_idler()
{
	int _c = 0;
	int _l = 0;
  
  // Override common homing procedure
  home_idler_smooth();
  return true;

	tmc2130_init(HOMING_MODE);

	move(-10, 0, 0); // move a bit in opposite direction

	for (int c = 1; c > 0; c--)  // not really functional, let's do it rather more times to be sure
	{
		delay(50);
		for (int i = 0; i < 2000; i++)
		{
  	  move(1, 0,0);
			delayMicroseconds(100);

			tmc2130_read_sg(0); 

			_c++;
			if (i == 1000) { _l++; }
			if (_c > 100) { shr16_set_led(1 << 2 * _l); };
			if (_c > 200) { shr16_set_led(0x000); _c = 0; };
		}
	}

	move(idler_steps_after_homing, 0, 0); // move to initial position

	tmc2130_init(tmc2130_mode);

	delay(500);

  isIdlerParked = false;

	park_idler(false);

	return true;
}


bool home_idler_smooth(void)
{
    uint32_t acc_backup = GLOBAL_ACC;
  
    tmc2130_init(tmc2130_mode);  // trinamic, normal
    
    //move_idl(-250, MAX_SPEED_IDLER, false);
    move_sm(-250, 0, 0, MAX_SPEED_IDLER, false);
      
    for (uint8_t c = 2; c > 0; c--) { // touch end 2 times

        tmc2130_init(HOMING_MODE);  // trinamic, homing

        GLOBAL_ACC = GLOBAL_ACC_DEF_NORMAL;

        //move_idl(2600,6350,true);
        move_sm(2600, 0, 0, 6350, true);
 
        tmc2130_init(tmc2130_mode);  // trinamic, homing

        GLOBAL_ACC = acc_backup;

        if (c > 1)  move_sm( -600, 0, 0, MAX_SPEED_IDLER, false); //move_idl( -600, MAX_SPEED_IDLER, false);

        delay(50);
    }

//  move(idler_steps_after_homing, 0, 0); // move to initial position
//  move_idl(idler_steps_after_homing, MAX_SPEED_IDLER, false); // move to initial position
    move_sm(idler_steps_after_homing, 0, 0, MAX_SPEED_IDLER, false);

  tmc2130_init(tmc2130_mode);

  //delay(500);

  delay(50);


  isIdlerParked = false;

  park_idler(false);
 /*   
    activeIdlPos = EXTRUDERS;
    if (toLastFilament) {
        uint8_t filament = 0;
        FilamentLoaded::get(filament);
        active_extruder = filament;
        setIDL2pos(active_extruder);
        engage_filament_pulley(false);
    }
*/
  return true;
}

bool home_selector()
{

  // Override common homing procedure 
  home_selector_smooth();
  return true;    

    // if FINDA is sensing filament do not home
    check_filament_not_present();
    
    tmc2130_init(HOMING_MODE);

	int _c = 0;
	int _l = 2;

	for (int c = 7; c > 0; c--)   // not really functional, let's do it rather more times to be sure
	{
		move(0, c * -18, 0);

		delay(50);
		for (int i = 0; i < 4000; i++)
		{
			move(0, 1,0);
			uint16_t sg = tmc2130_read_sg(AX_SEL);
			if ((i > 16) && (sg < 5))	break;

			_c++;
			if (i == 3000) { _l++; }
			if (_c > 100) { shr16_set_led(1 << 2 * _l); };
			if (_c > 200) { shr16_set_led(0x000); _c = 0; };
		}
	}

	move(0, selector_steps_after_homing,0); // move to initial position

  tmc2130_init(tmc2130_mode);

//	delay(500);
  delay(50);

	return true;
}

bool home_selector_smooth()
{
    uint32_t acc_backup = GLOBAL_ACC;

    //if(digitalRead(A1) == 1)
    check_filament_not_present(); // Blocks as long as filament is reported present 

    for (int c = 2; c > 0; c--) { // touch end 2 times

        tmc2130_init(HOMING_MODE);  // trinamic, homing

        GLOBAL_ACC = GLOBAL_ACC_DEF_NORMAL;

        //move_sm(int steps_idl=0, int steps_sel=0, int steps_pul=0, int speed, bool sgdetect=false, float acc=GLOBAL_ACC)
        move_sm(0, 4000, 0, 2000, true);

        GLOBAL_ACC = acc_backup;
        
        tmc2130_init(tmc2130_mode);  // trinamic, normal

        if (c > 1) move_sm(0, -300, 0, 2000 ,false);

        //ToDo: Add fancy led blinking
    }
    
    //move(0, selector_steps_after_homing,0); // move to initial position
    move_sm(0, selector_steps_after_homing, 0, 2000, false);

    tmc2130_init(tmc2130_mode);

    delay(500);

    return true;
}




//! @brief Home both idler and selector if already not done
void home()
{
    home_idler();

    home_selector();

    shr16_set_led(0x155);

    shr16_set_led(0x000);

    shr16_set_led(1 << 2 * (4-active_extruder));
}
 

void move_proportional(int _idler, int _selector)
{
	// gets steps to be done and set direction
	_idler = set_idler_direction(_idler);
	_selector = set_selector_direction(_selector);

	float _idler_step = _selector ? (float)_idler/(float)_selector : 1.0;
	float _idler_pos = 0;
	int delay = 2500; //microstep period in microseconds
	const int _start = _selector - 250;
	const int _end = 250;

	while (_selector != 0 || _idler != 0 )
	{
		if (_idler_pos >= 1)
		{
			if (_idler > 0) { idler_step_pin_set(); }
		}
		if (_selector > 0) { selector_step_pin_set(); }
		
		asm("nop");
		
		if (_idler_pos >= 1)
		{
			if (_idler > 0) { idler_step_pin_reset(); _idler--;  }
		}

		if (_selector > 0) { selector_step_pin_reset(); _selector--; }
		asm("nop");

		if (_idler_pos >= 1)
		{
			_idler_pos = _idler_pos - 1;
		}

		_idler_pos = _idler_pos + _idler_step;

		delayMicroseconds(delay);
		if (delay > 900 && _selector > _start) { delay -= 10; }
		if (delay < 2500 && _selector < _end) { delay += 10; }

	}
}

void move(int _idler, int _selector, int _pulley)
{
	int _acc = 50;

	// gets steps to be done and set direction
	_idler = set_idler_direction(_idler); 
	_selector = set_selector_direction(_selector);
	_pulley = set_pulley_direction(_pulley);
	

	do
	{
    if(_idler > 0) {idler_step_pin_set();}
		if(_selector > 0) { selector_step_pin_set();}
		if(_pulley > 0) { pulley_step_pin_set(); }
		asm("nop");
		if(_idler > 0) { idler_step_pin_reset(); _idler--; delayMicroseconds(1000); }
		if(_selector > 0) { selector_step_pin_reset(); _selector--;  delayMicroseconds(800); }
		if(_pulley > 0) { pulley_step_pin_reset(); _pulley--;  delayMicroseconds(700); }
		asm("nop");

		if (_acc > 0) { delayMicroseconds(_acc*10); _acc = _acc - 1; }; // super pseudo acceleration control

	} while (_selector != 0 || _idler != 0 || _pulley != 0);
}



void set_idler_dir_down()
{
	shr16_set_dir(shr16_get_dir() & ~4);
	//shr16_set_dir(shr16_get_dir() | 4);
}
void set_idler_dir_up()
{
	shr16_set_dir(shr16_get_dir() | 4);
	//shr16_set_dir(shr16_get_dir() & ~4);
}


int set_idler_direction(int _steps)
{
	if (_steps < 0)
	{
		_steps = _steps * -1;
		set_idler_dir_down();
	}
	else 
	{
		set_idler_dir_up();
	}
	return _steps;
}

int set_selector_direction(int _steps)
{
	if (_steps < 0)
	{
		_steps = _steps * -1;
		shr16_set_dir(shr16_get_dir() & ~2);
	}
	else
	{
		shr16_set_dir(shr16_get_dir() | 2);
	}
	return _steps;
}


int set_pulley_direction(int _steps)
{
	if (_steps < 0)
	{
		_steps = _steps * -1;
		set_pulley_dir_pull();
	}
	else
	{
		set_pulley_dir_push();
	}
	return _steps;
}

void set_pulley_dir_push()
{
	shr16_set_dir(shr16_get_dir() & ~1);
}
void set_pulley_dir_pull()
{
	shr16_set_dir(shr16_get_dir() | 1);
}

//! @brief Park idler
//! each filament selected has its park position, there is no park position for all filaments.
//! @param _unpark
//!  * false park
//!  * true engage
void park_idler(bool _unpark)
{
    if (_unpark && isIdlerParked) // get idler in contact with filament
    {
        move_proportional(idler_parking_steps, 0);
        isIdlerParked = false;
    }
    else if (!_unpark && !isIdlerParked) // park idler so filament can move freely
    {
        move_proportional(idler_parking_steps*-1, 0);
        isIdlerParked = true;
    }
}

bool move_selector_smooth(int steps, uint16_t speed)
{
    if (speed > MAX_SPEED_SELECTOR) speed = MAX_SPEED_SELECTOR;
    if (!isFilamentLoaded) {
        if (move_smooth(AX_SEL, steps, speed) == false) return false;
    } else return false;
    return true;
}

//move_smooth(uint8_t axis, int steps, int speed, bool rehomeOnFail,bool withStallDetection, float acc, bool withFindaDetection, bool withIR_SENSORDetection)
//void move_sm(int steps_idl, int steps_sel, int steps_pul, int speed, bool sgdetect, float acc)


void move_pulley_smooth(int steps, uint16_t speed)
{
    //move_smooth(AX_PUL, steps, speed, false, true);
    move_sm(0,0,steps,speed,true); // run with stall detection
}

/**
 * @brief set_idler_direction
 * @param steps: positive = towards engaging filament nr 1,
 * negative = towards engaging filament nr 5.
 * @return abs(steps)
 */
uint16_t set_idler_direction_smooth(int steps)
{
    if (steps < 0) {
        steps = steps * -1;
        shr16_write(shr16_v & ~SHR16_DIR_2);
    } else {
        shr16_write(shr16_v | SHR16_DIR_2);
    }
    return steps;
}

/**
 * @brief set_selector_direction
 * Sets the direction bit on the motor driver and returns positive number of steps
 * @param steps: positive = to the right (towards filament 5),
 * negative = to the left (towards filament 1)
 * @return abs(steps)
 */
uint16_t set_selector_direction_smooth(int steps)
{
    if (steps < 0) {
        steps = steps * -1;
        shr16_write(shr16_v & ~SHR16_DIR_1);
    } else {
        shr16_write(shr16_v | SHR16_DIR_1);
    }
    return steps;
}

/**
 * @brief set_pulley_direction
 * @param steps, positive (push) or negative (pull)
 * @return abs(steps)
 */
uint16_t set_pulley_direction_smooth(int steps)
{
    if (steps < 0) {
        steps = steps * -1;
        shr16_write(shr16_v | SHR16_DIR_0);
    } else {
        shr16_write(shr16_v & ~SHR16_DIR_0);
    }
    return steps;
}

/*
enum State {
      Accelerate = 0,
      ConstVelocity = 1,
      Decelerate = 2,
};
*/

void move_sm(int steps_idl, int steps_sel, int steps_pul, int speed, bool sgdetect, float acc)
{


 // int _acc = 50;
    int steps = 0;
    
    float vMax = speed;
    float v0 = 200; // steps/s, minimum speed
    float v = v0; // current speed
    int accSteps = 0; // number of steps for acceleration
    int stepsDone = 0;
    int stepsLeft = 0;
    int stepsOverall = 0;
    int steps_idl_remain = 0;
    int steps_sel_remain = 0;
    int steps_pul_remain = 0;
   
    //uint32_t result;
    //uint8_t sg;

    State st = Accelerate;

    // gets steps to be done and set direction
    if(steps_idl !=0) steps_idl_remain = set_idler_direction_smooth(steps_idl);
    if(steps_sel !=0) steps_sel_remain = set_selector_direction_smooth(steps_sel);
    if(steps_pul !=0) steps_pul_remain = set_pulley_direction_smooth(steps_pul);

    stepsOverall = steps_idl_remain + steps_sel_remain + steps_pul_remain;
    stepsLeft = stepsOverall;
   
    while(stepsLeft > 0)
    {
      if(steps_idl_remain > 0){
        idler_step_pin_set();
        asm("nop");
        idler_step_pin_reset();
        asm("nop");

      //c_idl++;
      //if (i == 1000) { _l++; }
      //if (_c > 100) { shr16_set_led(1 << 2 * _l); };
      //if (_c > 200) { shr16_set_led(0x000); _c = 0; };

        if (sgdetect && !digitalRead(A5)) { // stall detected
          delay(50);
          steps_idl_remain=1;
        }
        steps_idl_remain--;
        
      }
      if(steps_sel_remain > 0){
        selector_step_pin_set();
        asm("nop");
        selector_step_pin_reset();
        asm("nop");

        if (sgdetect && !digitalRead(A4)) { // stall detected
          delay(50);
          steps_sel_remain=1;
        }
        steps_sel_remain--;        
      }
      if(steps_pul_remain > 0){
        pulley_step_pin_set();
        asm("nop");
        pulley_step_pin_reset();
        asm("nop");

        if (sgdetect && !digitalRead(A3)) { // stall detected
          delay(50);
          steps_pul_remain=1;
        }
        steps_pul_remain--;
        
      }

      stepsLeft = steps_idl_remain + steps_sel_remain + steps_pul_remain;
      stepsDone = stepsOverall-stepsLeft;
       
      //stepsDone++;
      //stepsLeft--;

        float dt = 1 / v;
        delayMicroseconds(1e6 * dt);

        switch (st) {
        case Accelerate:
            v += acc * dt;
            if (v >= vMax) {
                accSteps = stepsDone;
                st = ConstVelocity;
                v = vMax;
            } else if (stepsDone > stepsLeft) {
                accSteps = stepsDone;
                st = Decelerate;
            }
            break;
        case ConstVelocity: {
            if (stepsLeft <= accSteps) {
                st = Decelerate;
            }
        }
        break;
        case Decelerate: {
            v -= acc * dt;
            if (v < v0) {
                v = v0;
            }
        }
        break;
        }
   
   }
}




/**
 * @brief moveTest
 * @param axis, index of axis, use AX_PUL, AX_SEL or AX_IDL
 * @param steps, number of micro steps to move
 * @param speed, max. speed
 * @param rehomeOnFail: flag, by default true, set to false
 *   in homing commands, to prevent endless loops and stack overflow.
 * @return
 */
// TODO 3: compensate delay for computation time, to get accurate speeds
bool move_smooth(uint8_t axis, int steps, int speed, bool rehomeOnFail,
                     bool withStallDetection, float acc,
                     bool withFindaDetection, bool withIR_SENSORDetection)
{
    // if in stealth mode don't look for stallguard
    if (tmc2130_mode == STEALTH_MODE) rehomeOnFail = false;
    shr16_set_ena(axis);
//ToDo    startWakeTime = millis();
    bool ret = true;
    if (withFindaDetection or withIR_SENSORDetection) ret = false;
    float vMax = speed;
    float v0 = 200; // steps/s, minimum speed
    float v = v0; // current speed
    int accSteps = 0; // number of steps for acceleration
    int stepsDone = 0;
    int stepsLeft = 0;

    switch (axis) {
    case AX_PUL:
        stepsLeft = set_pulley_direction_smooth(steps);
        break;
    case AX_IDL:
        stepsLeft = set_idler_direction_smooth(steps);
        break;
    case AX_SEL:
        stepsLeft = set_selector_direction_smooth(steps);
        break;
    }

    enum State {
        Accelerate = 0,
        ConstVelocity = 1,
        Decelerate = 2,
    };

    State st = Accelerate;

    while (stepsLeft) {
        switch (axis) {
        case AX_PUL:
            pulley_step_pin_set();//PIN_STP_PUL_HIGH;
            pulley_step_pin_reset();//PIN_STP_PUL_LOW;
            if (withStallDetection && !digitalRead(A3)) { // stall detected
                delay(50); // delay to release the stall detection
                return false;
            }
            if (withFindaDetection && ( steps > 0 ) && isFilamentLoaded) {
              return true;
            }
            if (withFindaDetection && ( steps < 0 ) && (isFilamentLoaded == false)) {
              return true;
            }
            if (withIR_SENSORDetection && IR_SENSOR) {
                IR_SENSOR = false;
                return true;
            }
            break;
        case AX_IDL:
            idler_step_pin_set();//PIN_STP_IDL_HIGH;
            idler_step_pin_reset();//PIN_STP_IDL_LOW;
            if (withStallDetection && !digitalRead(A5)) { // stall detected
                delay(50); // delay to release the stall detection
                if (rehomeOnFail) {
                    if (idlSGFailCount < 3) {
                        idlSGFailCount++;
                        //set_idler_toLast_positions(active_extruder);
                        motion_set_idler_selector(active_extruder, 0);
                        return false;
                    } else {
//ToDo                      fixIdlCrash();
/*DBG*/dbg(PSTR("AX_IDL: Unrecoverable_error\n"));
                            unrecoverable_error();
                      return false;
                    }
                } else {
                  return false;
                }
            }
            break;
        case AX_SEL:
            selector_step_pin_set();//PIN_STP_SEL_HIGH;
            selector_step_pin_reset();//PIN_STP_SEL_LOW;
            if (withStallDetection && !digitalRead(A4)) { // stall detected
                delay(50); // delay to release the stall detection
                if (rehomeOnFail) {
                    if (selSGFailCount < 3) {
                      selSGFailCount++;
                      //set_sel_toLast_positions(active_extruder);
                      motion_set_idler_selector(0,active_extruder);
                      return false;
                    } else {
//ToDo                      fixSelCrash();
/*DBG*/dbg(PSTR("AX_SEL: Unrecoverable_error\n"));
                            unrecoverable_error();
                      return false;
                    }
                } else {
                  return false;
                }
            }
            break;
        }

        stepsDone++;
        stepsLeft--;

        float dt = 1 / v;
        delayMicroseconds(1e6 * dt);

        switch (st) {
        case Accelerate:
            v += acc * dt;
            if (v >= vMax) {
                accSteps = stepsDone;
                st = ConstVelocity;

                v = vMax;
            } else if (stepsDone > stepsLeft) {
                accSteps = stepsDone;
                st = Decelerate;

            }
            break;
        case ConstVelocity: {
            if (stepsLeft <= accSteps) {
                st = Decelerate;
            }
        }
        break;
        case Decelerate: {
            v -= acc * dt;
            if (v < v0) {
                v = v0;
            }
        }
        break;
        }
    }
    switch (axis) {
    case AX_IDL:
//ToDo        idlSGFailCount = 0;
        break;
    case AX_SEL:
//ToDo        selSGFailCount = 0;
        break;
    }
    return ret;
}
