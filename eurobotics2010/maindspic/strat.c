/*  
 *  Copyright Droids, Microb Technology (2009)
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
 *  Revision : $Id: strat.c,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

//#include <diagnostic.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"

struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},
	.corn = {
		[0] = {X(0), Y(9)},
		[1] = {X(0), Y(5)},
		[2] = {X(0), Y(1)},
		[3] = {X(2), Y(11)},
		[4] = {X(2), Y(7)},
		[5] = {X(2), Y(3)},
		[6] = {X(4), Y(9)},
		[7] = {X(4), Y(5)},
		[8] = {X(6), Y(11)},
		[9] = {X(6), Y(7)},
		[10] = {X(8), Y(9)},
		[11] = {X(8), Y(5)},
		[12] = {X(10), Y(11)},
		[13] = {X(10), Y(7)},
		[14] = {X(10), Y(3)},
		[15] = {X(12), Y(9)},
		[16] = {X(12), Y(5)},
		[17] = {X(12), Y(1)},
	}
};

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/

void strat_set_bounding_box(void)
{
	if (get_color() == I2C_COLOR_YELLOW) {
		strat_infos.area_bbox.x1 = 350;
		strat_infos.area_bbox.y1 = 300;
		strat_infos.area_bbox.x2 = 2650; /* needed for c1 */
		strat_infos.area_bbox.y2 = 1800;
	}
	else {
		strat_infos.area_bbox.x1 = 350;
		strat_infos.area_bbox.y1 = 300;
		strat_infos.area_bbox.x2 = 2650; /* needed for c1 */
		strat_infos.area_bbox.y2 = 1800;
	}

	polygon_set_boundingbox(strat_infos.area_bbox.x1,
				strat_infos.area_bbox.y1,
				strat_infos.area_bbox.x2,
				strat_infos.area_bbox.y2);
}

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
										DO_POS | DO_BD | DO_POWER | DO_OPP;

	i2c_slavedspic_mode_hide_arm();

	strat_dump_conf();
	strat_dump_infos(__FUNCTION__);
}

void strat_dump_conf(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));
	
	printf_P(PSTR("  harvest_tomatoes: "));
	if (strat_infos.conf.flags & STRAT_CONF_HARVEST_TOMATOES)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  harvest_static_corns: "));
	if (strat_infos.conf.flags & STRAT_CONF_HARVEST_STATIC_CORNS)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));

	printf_P(PSTR("  harvest_fall_corns: "));
	if (strat_infos.conf.flags & STRAT_CONF_HARVEST_FALL_CORNS)
		printf_P(PSTR("on\r\n"));
	else
		printf_P(PSTR("off\r\n"));
}

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("%s() dump strat infos:\r\n"), caller);

	printf("balls_count = %d\n\r", strat_infos.balls_count);
	printf("corns_count = %d\n\r", strat_infos.corns_count);
	printf("opp_was_on_rampe = %d\n\r", strat_infos.opp_was_on_rampe);
  printf("tomato_event = %d\n\r", strat_infos.tomato_event);
  printf("static_corn_event = %d\n\r", strat_infos.static_corn_event);
  printf("fall_corn_event = %d\n\r", strat_infos.fall_corn_event);
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	strat_set_bounding_box();
	
	strat_infos.opp_was_on_rampe = FALSE;
	strat_infos.tomato_event = OFF;
	strat_infos.static_corn_event = OFF;
	strat_infos.fall_corn_event = OFF;
	strat_infos.event_elements_enable = 0;
}

/* call it just before launching the strat */
void strat_init(void)
{
	strat_reset_infos();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_OPP;
}


/* call it after each strat */
void strat_exit(void)
{
	uint8_t flags;

	mainboard.flags &= ~(DO_TIMER);
	strat_hardstop();
	time_reset();
	wait_ms(1000);
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	dac_mc_set(LEFT_DAC, 0);
	dac_mc_set(RIGHT_DAC, 0);
	IRQ_UNLOCK(flags);
	beacon_cmd_beacon_off();
}

/* called periodically */
void strat_event(void *dummy)
{
	/* limit speed when opponent is close */
	strat_limit_speed();
	
	//if(strat_infos.event_elements_enable){
		/* dectect tomatos */
		strat_event_tomato();
		
		/* detect corns */
		//strat_event_static_corn();
		
		/* detect fall corns */
		strat_event_fall_corn();
	//}	

//	/* dectct opponent with sensor */
//	if(sensor_get(S_OBSTACLE_FRONT))
//		strat_hardstop();
}

/* dump state (every 5 s max) */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


#define ERROUT(e) do{\
	err = e;\
}while(0)	


	
uint8_t strat_homologation(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;

	/* set new speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* show arm */ 
	i2c_slavedspic_mode_show_arm();
	//i2c_slavedspic_mode_prepare_harvest_ball();
	
	/* close balls lids */
	ball_lids_close();

	/* go in area */
//	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(350), 350);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			
	/* main diagonal */
	err = strat_goto_and_avoid_harvesting(COLOR_X(X(10)), Y(9),
		 STRAT_CONF_HARVEST_TOMATOES);	
 	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
		
	/* second diagonal inverse from tomato 9 to 13*/
	err = strat_goto_and_avoid_harvesting(COLOR_X(X(8)), Y(7),
		 STRAT_CONF_HARVEST_TOMATOES);	
 	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	err = strat_goto_and_avoid_harvesting(COLOR_X(X(11)), Y(4),
		 STRAT_CONF_HARVEST_TOMATOES);	
 	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go to basket */
	err = goto_and_avoid(COLOR_X(X(10)), Y(5), 
		TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
 	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	err = goto_and_avoid(COLOR_X(X(11)), Y(9), 
		TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
 	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
		
	/* out balls */
	ball_lids_open();
	
 	strat_set_speed(old_spdd, old_spda);
 	beacon_cmd_beacon_off();
		return err;	
		
}

static uint8_t strat_beginning(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;

	/* set new speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* show arm */ 
	i2c_slavedspic_mode_show_arm();

	/* goto near corn 5 */
	trajectory_goto_xy_abs(&mainboard.traj, 478, 653);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* go to tomato 1 */
	trajectory_goto_xy_abs(&mainboard.traj, 342, 789);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* when tomato cached go fordward a bit */
	trajectory_d_rel(&mainboard.traj, 10);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		
	/* go to read corn 1 */
	trajectory_goto_xy_abs(&mainboard.traj, 353, 1010);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* go in angle grid */
	trajectory_a_abs(&mainboard.traj, COLOR_A(31));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* go backward to read corn 5 */
	trajectory_d_rel(&mainboard.traj, -100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* go fordward to get tomato 3 */
	trajectory_d_rel(&mainboard.traj, 200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);	
	
	/* restore speeds */
	strat_set_speed(old_spdd, old_spda);
	
	return err;
}


uint8_t strat_main(void)
{
	uint8_t err, i, why=0;

	///* go in field of corns or go to harvest oranges */
	//err = strat_beginning();
	
	//wait_ms(20000);
	
	/* simple homologation */
	//err = strat_homologation();
	
	/* show arm */ 
	i2c_slavedspic_mode_show_arm();
	//i2c_slavedspic_mode_prepare_harvest_ball();
	
	/* close balls lids */
	ball_lids_close();
	
	
	err = strat_harvest_oranges();
	err = strat_goto_diagonal();
	
	err = strat_goto_basket();
	
	/* skip error code */

	while (1) {
		
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}

	}
	return END_TRAJ;
}
