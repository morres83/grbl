/*
  defaults_hackswood.h - defaults settings configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The defaults.h file serves as a central default settings file for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   here are supplied by users, so your results may vary. However, this should give you
   a good starting point as you get to know your machine and tweak the settings for your
   nefarious needs. */

#ifndef defaults_h
#define defaults_h

// Grbl default settings for C'T HACKS WOODMILL.
#define DEFAULT_X_STEPS_PER_MM 80.0 //tbd
#define DEFAULT_Y_STEPS_PER_MM 80.0 //tbd
#define DEFAULT_Z_STEPS_PER_MM 100.0 //tbd
#define DEFAULT_X_MAX_RATE 400.0 // mm/min //tbd
#define DEFAULT_Y_MAX_RATE 400.0 // mm/min //tbd
#define DEFAULT_Z_MAX_RATE 100.0 // mm/min //tbd
#define DEFAULT_X_ACCELERATION (200.0*60*60) // 200*60*60 mm/min^2 = 200 mm/sec^2 //tbd
#define DEFAULT_Y_ACCELERATION (200.0*60*60) // 200*60*60 mm/min^2 = 200 mm/sec^2 //tbd
#define DEFAULT_Z_ACCELERATION (200.0*60*60) // 200*60*60 mm/min^2 = 200 mm/sec^2 //tbd
#define DEFAULT_X_MAX_TRAVEL 800.0 // mm //tbd
#define DEFAULT_Y_MAX_TRAVEL 500.0 // mm //tbd
#define DEFAULT_Z_MAX_TRAVEL 120.0 // mm //tbd
	#define DEFAULT_STEP_PULSE_MICROSECONDS 10
	#define DEFAULT_STEPPING_INVERT_MASK 0
	#define DEFAULT_DIRECTION_INVERT_MASK 0
	#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
	#define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION))
	#define DEFAULT_JUNCTION_DEVIATION 0.1 // mm
	#define DEFAULT_ARC_TOLERANCE 0.02 // mm //tbd
	#define DEFAULT_REPORT_INCHES 0 // false
	#define DEFAULT_AUTO_START 1 // true
	#define DEFAULT_INVERT_ST_ENABLE 0 // false
	#define DEFAULT_INVERT_LIMIT_PINS 0 // false
	#define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
	#define DEFAULT_HARD_LIMIT_ENABLE 1  // false
	#define DEFAULT_HOMING_ENABLE 1  // false
	#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
	#define DEFAULT_HOMING_FEED_RATE 200.0 // mm/min
	#define DEFAULT_HOMING_SEEK_RATE 1500.0 // mm/min
	#define DEFAULT_HOMING_DEBOUNCE_DELAY 100 // msec (0-65k)
	#define DEFAULT_HOMING_PULLOFF 5.0 // mm
	#define DEFAULT_Z_GAUGE 10.0 // mm, Z Gauge Block Dicke
	#define DEFAULT_Z_PULLOFF 0.0 // mm, auto pull back Z after pressing ZERO center btn

#define JOG_MIN_SPEED   25    // Hz, kleinste Geschwindigkeit, > 10!
#define JOG_MAX_SPEED   8000  // Hz, größte Schrittgeschwindigkeit
#define JOG_RAMP        5     // Ramp speed inc/dec in µs per loop (higher=faster)

#endif