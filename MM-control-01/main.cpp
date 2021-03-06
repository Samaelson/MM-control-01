//! @file


#include "main.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include "shr16.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "tmc2130.h"
#include "abtn3.h"
#include "mmctl.h"
#include "stepper.h"
#include "Buttons.h"
#include <avr/wdt.h>
#include "permanent_storage.h"
#include "version.h"
#include "config.h"
#include "motion.h"
#include "Detect12V24V.h"
#include "i2c.h"
#include "pins.h"
uint8_t tmc2130_mode = STEALTH_MODE;

bool IR_SENSOR = true;

#if (UART_COM == 0)
FILE* uart_com = uart0io;
#elif (UART_COM == 1)
FILE* uart_com = uart1io;
#endif //(UART_COM == 0)


/*DBG*/#if (UART_DBG == 1)
/*DBG*/FILE* uart_dbg = uart0io;
/*DBG*/#endif


void fs_butler_set_ovr_status(void);
void fs_butler_reset_ovr_status(void);
bool fs_butler_get_ovr_status(void);
void process_fs_butler(FILE* inout);



namespace
{
//! @brief State
enum class S
{
    Idle,
    Setup,
    Printing,
    SignalFilament,
    Wait,
    WaitOk,
};
}

//! @brief Main MMU state
//!
//! @startuml
//!
//! title MMU Main State Diagram
//!
//! state Any {
//!   state Idle : Manual extruder selector
//!   state Setup
//!   state Printing
//!   state SignalFilament
//!
//!   [*] --> Idle : !MiddleButton
//!   [*] --> Setup : MiddleButton
//!   Any --> Printing : T<nr> || Eject
//!   Any --> Idle : Unload || RecoverEject
//!   Any --> SignalFilament : Load && filamentLoaded
//!   Any --> Wait : W0
//!   Setup --> Idle
//!   Wait --> Idle : RightButton
//!   WaitOk --> Idle : RightButton
//!   Wait --> WaitOk : MiddleButton && mmctl_IsOk
//!   WaitOk --> Wait : MiddleButton && !mmctl_IsOk
//! }
//! @enduml
static S state;

static void process_commands(FILE* inout);
/*DBG*/static void process_dbg_commands(FILE* inout);
static void process_fs_butler(void);

static void led_blink(int _no)
{
    shr16_set_led(1 << 2 * _no);
    delay(40);
    shr16_set_led(0x000);
    delay(20);
    shr16_set_led(1 << 2 * _no);
    delay(40);

    shr16_set_led(0x000);
    delay(10);
}

//! @brief signal filament presence
//!
//! non-blocking
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
static void signal_filament_present()
{
    shr16_set_led(0x2aa);
    delay(300);
    shr16_set_led(0x000);
    delay(300);
}

void signal_load_failure()
{
    shr16_set_led(0x000);
    delay(800);
    shr16_set_led(2 << 2 * (4 - active_extruder));
    delay(800);
}

void signal_ok_after_load_failure()
{
    shr16_set_led(0x000);
    delay(800);
    shr16_set_led(1 << 2 * (4 - active_extruder));
    delay(100);
    shr16_set_led(2 << 2 * (4 - active_extruder));
    delay(100);
    delay(800);
}

//! @brief Signal filament presence
//!
//! @retval true still present
//! @retval false not present any more
bool filament_presence_signaler()
{
    if (digitalRead(A1) == 1)
    {
        signal_filament_present();
        return true;
    }
    else
    {
        isFilamentLoaded = false;
        return false;
    }
}



//! @brief Check, if filament is not present in FINDA
//!
//! blocks, until filament is not removed and button pushed
//!
//! button | action
//! ------ | ------
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! b0 | b0 | b0 | b0 | b0 | Error, filament detected, still present
//! 0b | 0b | 0b | 0b | 0b | Error, filament detected, no longer present, continue by right button click
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking

void check_filament_not_present()
{
    while (digitalRead(A1) == 1)
    {
        while (Btn::right != buttonPressed())
        {
            if (digitalRead(A1) == 1)
            {
                signal_filament_present();
            }
            else
            {
                shr16_set_led(0x155);
                delay(300);
                shr16_set_led(0x000);
                delay(300);
            }
        }
    }
}

static void signal_drive_error()
{
    shr16_set_led(0x3ff);
    delay(300);
    shr16_set_led(0x000);
    delay(300);
}

void drive_error()
{
    for(uint8_t i = 0; i < 3; ++i)
    {
        signal_drive_error();
    }
    DriveError::increment();
}

//! @brief Unrecoverable hardware fault
//!
//! Stay in infinite loop and blink.
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG
//! -- | -- | -- | -- | --
//! bb | bb | bb | bb | bb
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void unrecoverable_error()
{
    while (1)
    {
        signal_drive_error();
    }
}

//! @brief Initialization after reset
//!
//! button | action
//! ------ | ------
//! middle | enter setup
//! right  | continue after error
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 00 | 00 | 00 | 00 | 0b | Shift register initialized
//! 00 | 00 | 00 | 0b | 00 | uart initialized
//! 00 | 00 | 0b | 00 | 00 | spi initialized
//! 00 | 0b | 00 | 00 | 00 | tmc2130 initialized
//! 0b | 00 | 00 | 00 | 00 | A/D converter initialized
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void setup()
{
  permanentStorageInit();

	shr16_init(); // shift register
	led_blink(0);

	uart0_init(); //uart0
	uart1_init(); //uart1
	led_blink(1);

#if (UART_STD == 0)
	stdin = uart0io; // stdin = uart0
	stdout = uart0io; // stdout = uart0
#elif (UART_STD == 1)
	stdin = uart1io; // stdin = uart1
	stdout = uart1io; // stdout = uart1
#endif //(UART_STD == 1)

	fprintf_P(uart_com, PSTR("start\n")); //startup message

	spi_init();
	led_blink(2);
	led_blink(3);

	adc_init(); // ADC
	led_blink(4);

	shr16_set_ena(7);
	shr16_set_led(0x000);

  //init_i2c();
  init_fs_butler();

    // check if to goto the settings menu
    if (buttonPressed() == Btn::middle)
    {
        state = S::Setup;
    }

    tmc2130_init(HOMING_MODE);
    tmc2130_read_gstat(); //consume reset after power up
    uint8_t filament;

/*DBG*/delay(5000);
/*DBG*/dbg(PSTR("StartUp MMU\n"));
    
    
    if(FilamentLoaded::get(filament))
    {
        motion_set_idler(filament);
    }

	if (digitalRead(A1) == 1) isFilamentLoaded = true;

}



//! @brief Select filament menu
//!
//! Select filament by pushing left and right button, park position can be also selected.
//!
//! button | action
//! ------ | ------
//! left   | select previous filament
//! right  | select next filament
//!
//! LED indication of states
//!
//! RG | RG | RG | RG | RG | meaning
//! -- | -- | -- | -- | -- | ------------------------
//! 01 | 00 | 00 | 00 | 00 | filament 1
//! 00 | 01 | 00 | 00 | 00 | filament 2
//! 00 | 00 | 01 | 00 | 00 | filament 3
//! 00 | 00 | 00 | 01 | 00 | filament 4
//! 00 | 00 | 00 | 00 | 01 | filament 5
//! 00 | 00 | 00 | 00 | bb | park position
//!
//! @n R - Red LED
//! @n G - Green LED
//! @n 1 - active
//! @n 0 - inactive
//! @n b - blinking
void manual_extruder_selector()
{
	shr16_set_led(1 << 2 * (4 - active_extruder));

	if ((Btn::left|Btn::right) & buttonPressed())
	{
		delay(500);

		switch (buttonPressed())
		{
		case Btn::right:
			if (active_extruder < 5)
			{
				select_extruder(active_extruder + 1);
			}
			break;
		case Btn::left:
			if (active_extruder > 0) select_extruder(active_extruder - 1);
			break;

		default:
			break;
		}
		delay(500);
	}

	if (active_extruder == 5)
	{
		shr16_set_led(2 << 2 * 0);
		delay(50);
		shr16_set_led(1 << 2 * 0);
		delay(50);
	}
}

/*DBG*/
void dbg(const char *fmt){
  fprintf_P(uart_dbg, fmt);
  delay(5);

}
/*DBG*/
void dbg_val(const char *fmt, uint32_t val){

  fprintf_P(uart_dbg, PSTR("%s: %lu\n"), fmt, val);
  delay(5);

}


//! @brief main loop
//!
//! It is possible to manually select filament and feed it when S::Idle.
//!
//! button | action
//! ------ | ------
//! middle | feed filament
//!
//! @copydoc manual_extruder_selector()
void loop()
{
    process_commands(uart_com);

    //ToDo: Check if thsi can be moved to state Printing 
    process_fs_butler();
        
    switch (state)
    {
    case S::Setup:
        if (!setupMenu()) state = S::Idle;
        break;
    case S::Printing:
        //ToDo: Check if this is working properly here 
        //process_fs_butler();
        break;
    case S::SignalFilament:
        if (!filament_presence_signaler()) state = S::Idle;
        break;
    case S::Idle:
        manual_extruder_selector();
        if(Btn::middle == buttonPressed() && active_extruder < 5)
        {
            shr16_set_led(2 << 2 * (4 - active_extruder));
            delay(500);
            if (Btn::middle == buttonPressed())
            {
                motion_set_idler_selector(active_extruder);
                feed_filament();
            }
        }
        break;
    case S::Wait:
        signal_load_failure();
        switch(buttonClicked())
        {
        case Btn::middle:
            if (mmctl_IsOk()) state = S::WaitOk;
            break;
        case Btn::right:
            state = S::Idle;
            fprintf_P(uart_com, PSTR("ok\n"));
            break;
        default:
            break;
        }
        break;
    case S::WaitOk:
        signal_ok_after_load_failure();
        switch(buttonClicked())
        {
        case Btn::middle:
            if (!mmctl_IsOk()) state = S::Wait;
            break;
        case Btn::right:
            state = S::Idle;
            fprintf_P(uart_com, PSTR("ok\n"));
            break;
        default:
            break;
        }
        break;
    }
}

/*DBG*/
void process_dbg_commands(FILE* inout)
{
  static char line[32];
  static int count = 0;
  int c = -1;
  if (count < 32)
  {
    if ((c = getc(inout)) >= 0)
    {
      if (c == '\r') c = 0;
      if (c == '\n') c = 0;
      line[count++] = c;
    }
  }
  else
  {
    count = 0;
    //overflow
  }
  int value = 0;
  int value0 = 0;

  if ((count > 0) && (c == 0))
  {
    //line received
    //printf_P(PSTR("line received: '%s' %d\n"), line, count);
    count = 0;
        //! T<nr.> change to filament <nr.>
    if (sscanf_P(line, PSTR("T%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS))
      {
          state = S::Printing;
        switch_extruder_withSensor(value);
        fprintf_P(inout, PSTR("ok\n"));
      }
    }
    else if (sscanf_P(line, PSTR("X%d"), &value) > 0)
    {
      if (value == 0) //! X0 MMU reset
        wdt_enable(WDTO_15MS);
    }
    else if (sscanf_P(line, PSTR("H%d"), &value) > 0)
    {
      if (!isFilamentLoaded){
        delay(2000);
        if (value == 0){ //! H0 Home idler
              home_idler();
        }
        else if (value==1){
              home_idler_smooth();
        }
      }
      fprintf_P(inout, PSTR("ok\n"));
    }
    else if (sscanf_P(line, PSTR("MIF%d"), &value) > 0)
    {
      if (!isFilamentLoaded){
              move_proportional(value,0);
      }
      fprintf_P(inout, PSTR("ok\n"));
    }
    else if (sscanf_P(line, PSTR("MIR%d"), &value) > 0)
    {
      if (!isFilamentLoaded){
              move_proportional((-1)*value,0);
      }
      fprintf_P(inout, PSTR("ok\n"));
    }
    else if (sscanf_P(line, PSTR("F%d"), &value) > 0)
    {
      if ((value >= 0) && (value < EXTRUDERS)) { //! Fx: Selector/Idler set to filament x wiht x=[0,EXTRUDERS-1]
          if (!isFilamentLoaded){
            select_extruder(value);
          }
          fprintf_P(inout, PSTR("ok\n"));
      }
      else{
        //invalid value      
      }
    }
    else{
      // unknown commans
    }
  }
  else
  { //nothing received
  }  
 }

//! @brief receive and process commands from serial line
//! @param[in,out] inout struct connected to serial line to be used
//!
//! All commands have syntax in form of one letter integer number.
void process_commands(FILE* inout)
{
	static char line[32];
	static int count = 0;
	int c = -1;
	if (count < 32)
	{
		if ((c = getc(inout)) >= 0)
		{
			if (c == '\r') c = 0;
			if (c == '\n') c = 0;
			line[count++] = c;
		}
	}
	else
	{
		count = 0;
		//overflow
	}
	int value = 0;
	int value0 = 0;

	if ((count > 0) && (c == 0))
	{
		//line received
		//printf_P(PSTR("line received: '%s' %d\n"), line, count);
		count = 0;
        //! T<nr.> change to filament <nr.>
		if (sscanf_P(line, PSTR("T%d"), &value) > 0)
		{
			if ((value >= 0) && (value < EXTRUDERS))
			{
        //fs_butler_set_ovr_status();
			  state = S::Printing;
				switch_extruder_withSensor(value);
				fprintf_P(inout, PSTR("ok\n"));
			}
		}
        //! L<nr.> Load filament <nr.>
		else if (sscanf_P(line, PSTR("L%d"), &value) > 0)
		{
			//fs_butler_set_ovr_status();
			if ((value >= 0) && (value < EXTRUDERS))
			{
			    if (isFilamentLoaded) state = S::SignalFilament;
			    else
			    {
                    select_extruder(value);
                    feed_filament();
			    }
                fprintf_P(inout, PSTR("ok\n"));
			}
		}
		else if (sscanf_P(line, PSTR("M%d"), &value) > 0)
		{
			//! M0 set to normal mode
			//!@n M1 set to stealth mode
      uint8_t sysV = getMMU2S_System_Voltage();
      switch (value) 
      {
        case 0:
          if (sysV < 254 && sysV > 180) 
          {
              //filament_lookup_table[0][0] = TYPE_0_MAX_SPPED_PUL;
              //filament_lookup_table[0][1] = TYPE_1_MAX_SPPED_PUL;
              //filament_lookup_table[0][2] = TYPE_2_MAX_SPPED_PUL;
              MAX_SPEED_IDLER = MAX_SPEED_IDL_DEF_NORMAL;
              MAX_SPEED_SELECTOR = MAX_SPEED_SEL_DEF_NORMAL;
              GLOBAL_ACC = GLOBAL_ACC_DEF_NORMAL;
              tmc2130_mode =  NORMAL_MODE;
          }
          else
          {
            tmc2130_mode = STEALTH_MODE; 
          }
          break;
        case 1: 
            //for (uint8_t i = 0; i < 3; i++)
            //  if (filament_lookup_table[0][i] > 1500) filament_lookup_table[0][i] = 1400;
            MAX_SPEED_IDLER = MAX_SPEED_IDL_DEF_STEALTH;
            MAX_SPEED_SELECTOR = MAX_SPEED_SEL_DEF_STEALTH;
            GLOBAL_ACC = GLOBAL_ACC_DEF_STEALTH;

            tmc2130_mode = STEALTH_MODE;
          break;
        default: return;
      }
  		//init all axes
			tmc2130_init(tmc2130_mode);
			fprintf_P(inout, PSTR("ok\n"));
		}
		//! U<nr.> Unload filament. <nr.> is ignored but mandatory.
		else if (sscanf_P(line, PSTR("U%d"), &value) > 0)
		{
			//fs_butler_set_ovr_status();
			unload_filament_withSensor();
			fprintf_P(inout, PSTR("ok\n"));

			state = S::Idle;
		}
    //! V<nr.> dis-/engage filament.
    else if (sscanf_P(line, PSTR("V%d"), &value) > 0)
    {
      //!V0 set fs butler override
      //!V1 reset fs butler override
      //!V3 return butler 'status' to printer 
      if (value == 0)
      {
        // Disengage filament
        fs_butler_set_ovr_status();
        fprintf_P(inout, PSTR("ok\n"));
      }
      else if (value == 1)
      {
        // Engage filament
        fs_butler_reset_ovr_status();
        fprintf_P(inout, PSTR("ok\n"));
      }
      else if (value == 3)
      {
        // Status request received
        // ToDo: Add sensor check here
        fprintf_P(inout, PSTR("ok\n"));
      }
    }   
		else if (sscanf_P(line, PSTR("X%d"), &value) > 0)
		{
			if (value == 0) //! X0 MMU reset
				wdt_enable(WDTO_15MS);
		}
		else if (sscanf_P(line, PSTR("P%d"), &value) > 0)
		{
			if (value == 0) //! P0 Read finda
				fprintf_P(inout, PSTR("%dok\n"), digitalRead(A1));
		}
		else if (sscanf_P(line, PSTR("S%d"), &value) > 0)
		{
			if (value == 0) //! S0 return ok
				fprintf_P(inout, PSTR("ok\n"));
			else if (value == 1) //! S1 Read version
				fprintf_P(inout, PSTR("%dok\n"), fw_version);
			else if (value == 2) //! S2 Read build nr.
				fprintf_P(inout, PSTR("%dok\n"), fw_buildnr);
			else if (value == 3) //! S3 Read drive errors
			    fprintf_P(inout, PSTR("%dok\n"), DriveError::get());
		}
		//! F<nr.> \<type\> filament type. <nr.> filament number, \<type\> 0, 1 or 2. Does nothing.
		else if (sscanf_P(line, PSTR("F%d %d"), &value, &value0) > 0)
		{
			if (((value >= 0) && (value < EXTRUDERS)) &&
				((value0 >= 0) && (value0 <= 2)))
			{
				filament_type[value] = value0;
				fprintf_P(inout, PSTR("ok\n"));
			}
		}
		else if (sscanf_P(line, PSTR("C%d"), &value) > 0)
		{
			if (value == 0) //! C0 continue loading current filament (used after T-code).
			{
				load_filament_inPrinter();
				fprintf_P(inout, PSTR("ok\n"));
			}
		}
		else if (sscanf_P(line, PSTR("E%d"), &value) > 0)
		{
			if ((value >= 0) && (value < EXTRUDERS)) //! E<nr.> eject filament
			{
				//fs_butler_set_ovr_status();
				eject_filament(value);
				fprintf_P(inout, PSTR("ok\n"));
				state = S::Printing;
			}
		}
		else if (sscanf_P(line, PSTR("R%d"), &value) > 0)
		{
			if (value == 0) //! R0 recover after eject filament
			{
				//fs_butler_set_ovr_status();
				recover_after_eject();
				fprintf_P(inout, PSTR("ok\n"));
				state = S::Idle;
			}
		}
        else if (sscanf_P(line, PSTR("W%d"), &value) > 0)
        {
            if (value == 0) //! W0 Wait for user click
            {
                //fs_butler_set_ovr_status();
                state = S::Wait;
            }
        }
        else if (sscanf_P(line, PSTR("K%d"), &value) > 0)
        {
            if ((value >= 0) && (value < EXTRUDERS)) //! K<nr.> cut filament
            {
                //fs_butler_set_ovr_status();
                mmctl_cut_filament(value);
                fprintf_P(inout, PSTR("ok\n"));
            }
        }
       
	}
	else
	{ //nothing received
	}
}

bool last_state = false;
bool last_isFilamentLoaded = false;
bool msg_sent = false;
bool last_door_sensor_status = false;
bool new_door_sensor_status = false;
bool last_finda_status = false;
bool new_finda_status = false;
bool filamentIsEngaged = false;
bool fs_butler_override = true;

void fs_butler_set_ovr_status(void)
{
  fs_butler_override = true;

  if(digitalRead(A1))
  {
    // Override was triggered and filament still triggers finda 
    if(filamentIsEngaged)
    {
      motion_disengage_idler();
      disengage_pulley();
      filamentIsEngaged = false;
    }
  }
}

void fs_butler_reset_ovr_status(void)
{
  fs_butler_override = false;
      
  if(digitalRead(A1))
  {
    
    if(!filamentIsEngaged)
    {
      motion_engage_idler();
      engage_pulley();
      filamentIsEngaged = true;
    }
  }
}

bool fs_butler_get_ovr_status(void)
{
  return fs_butler_override;
}



void process_fs_butler(void)
{
  bool new_state;
  
  if(fs_butler_get_ovr_status()) return;

  // Check finda status
  new_finda_status = digitalRead(A1);
  if(new_finda_status != last_finda_status)
  {
    last_finda_status = new_finda_status;

    if(!new_finda_status)
    {
      // Filament no longer triggering finda
      fs_butler_set_ovr_status();
      return;
    }
  }

  // Filement is loaded, so get the state of the fs butler sensor
  new_state = get_fs_butler_status();
  
  // check for state change
  if(last_state == new_state) return;
  
  last_state = new_state;

  if(new_state == true)
  {
    feed_filament_withSensor(70); //1 pulley ustep = (d*pi)/(mres*FSPR) = 49.48 um --> 110 = ~5.5mm / 70 =~3.5mm
  }
  // else do nothing

}

/*
void process_fs_butler(void)
{

  bool new_state;

  if(fs_butler_get_ovr_status() && digitalRead(A1))
  {
    // Override was triggered and filament still triggers finda 
    if(filamentIsEngaged)
    {
      motion_disengage_idler();
      disengage_pulley();
      filamentIsEngaged = false;
    }
    return;
  }
  // Check finda status
  new_finda_status = digitalRead(A1);

  if(new_finda_status != last_finda_status)
  {
    last_finda_status = new_finda_status;

    if(!new_finda_status)
    {
      motion_reset_door_sensor_status();
      fs_butler_reset_ovr_status();
    }
  }

  // check printer door gate sensor status
  new_door_sensor_status = motion_get_door_sensor_status();

  if(new_door_sensor_status != last_door_sensor_status)
  {
    last_door_sensor_status = new_door_sensor_status;
    if(new_door_sensor_status)
    {
      motion_engage_idler();
      engage_pulley();
      filamentIsEngaged = true;
    }
    else
    {
      motion_disengage_idler();
      disengage_pulley();
      filamentIsEngaged = false;
    }
  }

  if(!motion_get_door_sensor_status()) led_blink(1);
  if(!!filamentIsEngaged) led_blink(2);
  

  // check if filament is still loaded into printer and triggered door gate sensor 
  if(!motion_get_door_sensor_status() || !filamentIsEngaged) return;

  // Filement is loaded, so get the state of the fs butler sensor
  new_state = get_fs_butler_status();
  
  // check for state change
  if(last_state == new_state) return;
  
  last_state = new_state;

  if(new_state == true)
  {
    feed_filament_withSensor(70); //1 pulley ustep = (d*pi)/(mres*FSPR) = 49.48 um --> 110 = ~5.5mm / 70 =~3.5mm
  }
  // else do nothing
}

*/
