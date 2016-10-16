/*  
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: cs.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive\error.h>


#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>
//#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "beacon.h"

/* called every 5 ms */
static void do_cs(void *dummy) 
{
	/* read encoders */
	if (sensorboard.flags & DO_ENCODERS) {
		//encoders_spi_manage(NULL);
		encoders_dspic_manage(NULL);
	}
	/* control system */
	if (sensorboard.flags & DO_CS) {
		if (sensorboard.beacon.on)
			cs_manage(&sensorboard.beacon.cs);
	}
	if (sensorboard.flags & DO_BD) {
		bd_manage_from_cs(&sensorboard.beacon.bd, &sensorboard.beacon.cs);
	}
	if (sensorboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"), 
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void microb_cs_init(void)
{
	/* ---- CS beacon */
	/* PID */
	pid_init(&sensorboard.beacon.pid);
	pid_set_gains(&sensorboard.beacon.pid, 80, 80, 250);
	pid_set_maximums(&sensorboard.beacon.pid, 0, 10000, 2600);
	pid_set_out_shift(&sensorboard.beacon.pid, 6);
	pid_set_derivate_filter(&sensorboard.beacon.pid, 6);

	/* CS */
	cs_init(&sensorboard.beacon.cs);
	cs_set_correct_filter(&sensorboard.beacon.cs, pid_do_filter, &sensorboard.beacon.pid);
	cs_set_process_in(&sensorboard.beacon.cs, pwm_mc_set, BEACON_PWM);
	cs_set_process_out(&sensorboard.beacon.cs, encoders_spi_update_beacon_speed, BEACON_ENCODER);
	cs_set_consign(&sensorboard.beacon.cs, 0);

	/* Blocking detection */

	/* set them on !! */
	sensorboard.beacon.on = 0;

	scheduler_add_periodical_event_priority(do_cs, NULL, 
						5000L / SCHEDULER_UNIT, CS_PRIO);
}
