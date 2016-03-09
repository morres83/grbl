/*
  jogging.c - code pertaining to  jog switches
  Copyright (c) 2013 Carsten Meyer, cm@ct.de
  Adjustments for GRBL 0.9, code enhancement and bug fixes by Matthias N��lein, mail@morres83.eu

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

#include "grbl.h"
#define ADCSRA_init 0x83  // AD enable, IRQ off, Prescaler 8
#define ADMUX_init  0x20  // ADLAR =1 (left adjusted, 8-Bit-Result on ADCH)
uint32_t speed_span[3];

void jog_init() {
	// Initialize jog switch port bits and DDR
	#ifdef LED_PRESENT
	LED_DDR |= ((1<<LED_RUN_BIT) | (1<<LED_ERROR_BIT));
	LED_PORT |= ((1<<LED_RUN_BIT) | (1<<LED_ERROR_BIT)); // active low, so set high
	#endif
	JOGSW_DDR &= ~(JOGSW_MASK); // Set as input pins
	JOGSW_PORT |= (JOGSW_MASK); // Enable internal pull-up resistors. Active low operation.

	CONTROL_DDR &= ~(1<<SPIN_TOGGLE);  // Set as input pin
	CONTROL_PORT|= (1<<SPIN_TOGGLE);   // Pullup
	
	ADCSRA = ADCSRA_init;
	ADMUX = ADMUX_init | JOG_POT;     // Kanal, ADLAR =1 (left adjustet, 8-Bit-Result on ADCH)
	
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {}         // wait until initial conversion is done
	(void) ADCH; // ADCH needs to be read one time

	for (uint8_t i=0; i<N_AXIS; i++){
		//speed span = 4/5 of max rate; divided by 255 for ADC multiplication; 4/(5 * 60 * 255) = 19125
		speed_span[i] = settings.max_rate[i]*settings.steps_per_mm[i] / 19125;
	}

}

void jog_btn_release() {
	uint8_t jog_bits;
	do {
		jog_bits = (~JOGSW_PIN) & JOGSW_MASK; // active low
		protocol_process(); // process the serial protocol while waiting
		protocol_execute_realtime();
	}
	while (jog_bits); // until released
}

void spindle_btn_release() {
	uint8_t spindle_bits;
	do {
		spindle_bits = (~CONTROL_PIN) & (1<<SPIN_TOGGLE); // active low
		protocol_process(); // process the serial protocol while waiting
		protocol_execute_realtime();
	}
	while (spindle_bits); // until released
}

void jogging()
// Tests jog port pins and moves steppers
{
	uint8_t jog_bits, jog_bits_old, out_bits0, jog_exit, last_sys_state;
	uint8_t i, spindle_bits;
	
	uint32_t dest_step_rate, step_rate, step_delay, init_step_rate; // Step delay after pulse
	float work_position, mm_per_step;
	char line[LINE_BUFFER_SIZE];

	switch (sys.state) {
		case STATE_CYCLE: case STATE_HOMING:
		//LED_PORT |= (1<<LED_ERROR_BIT);
			return;
		case STATE_ALARM: case STATE_SAFETY_DOOR:
			break;
		//default:
		//LED_PORT |= (1<<LED_ERROR_BIT);
	}
	last_sys_state = sys.state;


	spindle_bits = (~CONTROL_PIN) & (1<<SPIN_TOGGLE); // active low
	if (spindle_bits) {
		if (bit_istrue(SPINDLE_ENABLE_PORT,bit(SPINDLE_ENABLE_BIT))) {
			//      gc.spindle_direction = 0;
			spindle_run(0, 0.0);
		}
		else {
			//      gc.spindle_direction = 1;   // also update gcode spindle status
			spindle_run(1, 0.0);
		}
		spindle_btn_release();
		delay_ms(20);
	}

	jog_bits = (~JOGSW_PIN) & JOGSW_MASK; // active low
	if (!jog_bits) { return; }  // nothing pressed
	
	// At least one jog/joystick switch is active
	if (jog_bits & (1<<JOG_ZERO)) {     // Zero-Button gedr�ckt
		jog_btn_release();
		sys.state = last_sys_state;
		if (bit_isfalse(CONTROL_PIN,bit(RESET_BIT))) { // RESET und zus�tzlich ZERO gedr�ckt: Homing
			if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
				// Only perform homing if Grbl is idle or lost.
				if ( sys.state==STATE_IDLE || sys.state==STATE_ALARM ) {
					sys.state = STATE_HOMING; // Set system state variable
					mc_homing_cycle();
					if (!sys.abort) { 
						sys.state = STATE_IDLE; // Set to IDLE when complete.
						st_go_idle(); // Set steppers to the settings idle state before returning.
						 system_execute_startup(line); // Execute startup scripts after successful homing.
					} 
				}
			}
		} 
		else { // Zero current work position

			plan_sync_position();
			gc_sync_position();

			//      gc.coord_system[i]    Current work coordinate system (G54+). Stores offset from absolute machine
			//                            position in mm. Loaded from EEPROM when called.
			//      gc.coord_offset[i]    Retains the G92 coordinate offset (work coordinates) relative to
			//                            machine zero in mm. Non-persistent. Cleared upon reset and boot.

			for (i=0; i<N_AXIS; i++) { // Axes indices are consistent, so loop may be used.
				gc_state.coord_offset[i] = gc_state.position[i] - gc_state.coord_system[i];
			}

			// Z-Achse um bestimmten Betrag zur�ckziehen
			float xyz[3] = {gc_state.position[X_AXIS], gc_state.position[Y_AXIS], gc_state.position[Z_AXIS] + settings.z_zero_pulloff};
			mc_line(xyz, settings.homing_seek_rate, false);
			
			protocol_buffer_synchronize(); // Make sure the motion completes
			
			gc_state.position[Z_AXIS] = gc_state.position[Z_AXIS] - (settings.z_zero_gauge);
			gc_state.coord_offset[Z_AXIS] = gc_state.position[Z_AXIS] - gc_state.coord_system[Z_AXIS];
			
			// The gcode parser position circumvented by the pull-off maneuver, so sync position vectors.
			// Sets the planner position vector to current steps. Called by the system abort routine.
			// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
			plan_sync_position();
			gc_sync_position(); // Syncs all internal position vectors to the current system position.

		}
		return;
	}

	uint8_t reverse_flag = 0;
	uint8_t out_bits = 0;
	uint8_t jog_select = 0;
	out_bits0 = 0; // no invert anymore!
	
	ADCSRA |= (1<<ADSC); //start conversion
	
	sys.state = STATE_JOG;
	
	// check for reverse switches
	if (jog_bits & (1<<JOGREV_X_BIT)) { // X reverse switch on
		if (bit_isfalse(settings.dir_invert_mask,bit(0))) out_bits0 ^= (1<<X_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<X_STEP_BIT);
		reverse_flag = 1;
	}
	else if (jog_bits & (1<<JOGREV_Y_BIT)) { // Y reverse switch on
		if (bit_isfalse(settings.dir_invert_mask,bit(1))) out_bits0 ^= (1<<Y_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<Y_STEP_BIT);
		reverse_flag = 1;
		jog_select = 1;
	}
	else if (jog_bits & (1<<JOGREV_Z_BIT)) { // Z reverse switch on
		if (bit_isfalse(settings.dir_invert_mask,bit(2))) out_bits0 ^= (1<<Z_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<Z_STEP_BIT);
		reverse_flag = 1; // positive Z dir!
		jog_select = 2;
	}
	// check for forward switches
	else if (jog_bits & (1<<JOGFWD_X_BIT)) { // X forward switch on
		if (bit_istrue(settings.dir_invert_mask,bit(0))) out_bits0 ^= (1<<X_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<X_STEP_BIT);
	}
	else if (jog_bits & (1<<JOGFWD_Y_BIT)) { // Y forward switch on
		if (bit_istrue(settings.dir_invert_mask,bit(1))) out_bits0 ^= (1<<Y_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<Y_STEP_BIT);
		jog_select = 1;
	}
	else if (jog_bits & (1<<JOGFWD_Z_BIT)) { // Z forward switch on
		if (bit_istrue(settings.dir_invert_mask,bit(2))) out_bits0 ^= (1<<Z_DIRECTION_BIT);
		out_bits = out_bits0 ^ (1<<Z_STEP_BIT);
		//reverse_flag = 1; // positive Z dir!
		jog_select = 2;
	}

	while (ADCSRA & (1<<ADSC)) {} // wait until conversion is finished
	dest_step_rate = ADCH;    // set initial dest_step_rate according to analog input
	//Formula as follows:
	//min_rate is 1/5 of max_rate; ADC at 255 => max speed; ADC at 0 => 1/5 max speed. dest_step_rate in Hz
	dest_step_rate = speed_span[jog_select] * (dest_step_rate + 255/4);
	init_step_rate = settings.max_rate[jog_select] * settings.steps_per_mm[jog_select] * 1 / 16 / 60;   // set initial step rate
	step_rate = init_step_rate;
	jog_exit = 0;
	
	uint8_t bits = LIMIT_PIN;
	if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { bits ^= LIMIT_MASK; }
	if (bit_istrue(bits,LIMIT_MASK) && reverse_flag) {
		sys.state = last_sys_state;
		return;
	} // do not move towards a limit switch if either one is hit already
	st_wake_up();
	
	
	// prepare direction with small delay, direction settle time
	DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (out_bits0 & DIRECTION_MASK);
	//STEP_PORT = (STEP_PORT & ~STEP_MASK) | (out_bits0 & STEP_MASK);
	delay_us(10);
	jog_bits_old = jog_bits;
	i = 0;  // now index for sending position data
	
	// Report machine position;
	if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
		mm_per_step = 1/(settings.steps_per_mm[jog_select] * INCH_PER_MM);
		} else {
		mm_per_step = 1/settings.steps_per_mm[jog_select];
	}
	
	//work_position = sys.position[jog_select] / mm_per_step;
	work_position = sys.position[jog_select]; //NEU
	//work_position = print_position[jog_select];
	
	for(;;) { // repeat until button/joystick released
		//    report_realtime_status(); // ben�tigt viel Zeit!

		// Get limit pin state
		bits = LIMIT_PIN;
		if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { bits ^= LIMIT_MASK; }
		if (bit_istrue(bits,LIMIT_MASK) && reverse_flag) { jog_exit = 1; } // immediate stop on any switch
		
		jog_bits = (~JOGSW_PIN) & JOGSW_MASK; // active low
		if (jog_bits == jog_bits_old) { // nothing changed
			if (step_rate < (dest_step_rate - 5)) { // Hysteresis for A/D-conversion
				step_rate += JOG_RAMP; // accelerate
			}
			if (step_rate > (dest_step_rate + 5)) { // Hysteresis for A/D-conversion
				step_rate -= JOG_RAMP; // brake
			}
		}
		else {
			if (step_rate > (init_step_rate)) {  // switch change happened, fast brake to complete stop
				step_rate = ((step_rate * 99) / 100) - 20;
			}
			else { jog_exit = 1; } // finished to stop and exit
		}
		
		// stop and exit if done
		if (jog_exit || (sys_rt_exec_state & EXEC_RESET)) {
			st_go_idle();
			sys.state = last_sys_state;
			sys.position[jog_select] = work_position;
			plan_sync_position();
			gc_sync_position(); // Syncs all internal position vectors to the current system position.
			return;
		}
		

		ADCSRA |= (1<<ADSC); //start ADC conversion
		// Both direction and step pins appropriately inverted and set. Perform one step
		STEP_PORT = (STEP_PORT & ~STEP_MASK) | (out_bits & STEP_MASK); //Step pulse on
		delay_us(settings.pulse_microseconds>>1); //seems okay for little step  pulses
		STEP_PORT = (STEP_PORT & ~STEP_MASK);// | (out_bits0 & STEP_MASK); //Step pulse off
		
		step_delay = (1000000/step_rate) - settings.pulse_microseconds - 100; // 100 = fixed value for loop time; this might have to be checked by oscilloscope...
	
		if (reverse_flag) {
			//sys.position[jog_select]--;       // sys.position ist in Steps!
			//work_position -= mm_per_step;
			work_position--;
		}
		else {
			//sys.position[jog_select]++;
			//work_position += mm_per_step;    // relative print_position in mm since last report
			work_position++;
		}
		
		if (sys_rt_exec_state & EXEC_STATUS_REPORT) {
			if (step_delay > 250) { //this might have to be checked by oscilloscope...
				// status report requested, print short msg only
				printPgmString(PSTR("<Jog"));
				serial_write(88 + jog_select); // 88 = X + 1 = Y etc.
				serial_write(44);
				printFloat_CoordValue(work_position * mm_per_step - gc_state.coord_offset[jog_select]);
				serial_write(62); // ">" char
				serial_write(13);
				serial_write(10);
				
				step_delay -= 250;
			}
			else
			{
				printPgmString(PSTR("<Jog>\r\n"));
			}
			sys_rt_exec_state = 0;
		}
		
		delay_us(step_delay);
		
		while (ADCSRA & (1<<ADSC)) {} // wait until conversion is finished
		dest_step_rate = ADCH;    // set next dest_step_rate according to analog input
		dest_step_rate = speed_span[jog_select] * (dest_step_rate + 255/4);

	}
}