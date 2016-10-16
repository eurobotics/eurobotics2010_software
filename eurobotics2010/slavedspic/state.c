/*  
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <encoders_dspic.h>
#include <pwm_servo.h>
#include <pwm_mc.h>
#include <dac_mc.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "arm_xy.h"
#include "arm_highlevel.h"
#include "state.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */

#define HIDE_ARM								I2C_SLAVEDSPIC_MODE_HIDE_ARM
#define SHOW_ARM								I2C_SLAVEDSPIC_MODE_SHOW_ARM
#define PREPARE_HARVEST_BALL		I2C_SLAVEDSPIC_MODE_PREPARE_HARVEST_BALL
#define HARVEST_TOMATO					I2C_SLAVEDSPIC_MODE_HARVEST_TOMATO

#define PUTIN_FINGER_BALL 			I2C_SLAVEDSPIC_MODE_PUTIN_FINGER_BALL
#define ARM_PUMP_ON 						I2C_SLAVEDSPIC_MODE_ARM_PUMP_ON						
#define ARM_PUMP_OFF 						I2C_SLAVEDSPIC_MODE_ARM_PUMP_OFF					
#define CORN_ROLLS_IN 					I2C_SLAVEDSPIC_MODE_CORN_ROLLS_IN					
#define CORN_ROLLS_OUT 					I2C_SLAVEDSPIC_MODE_CORN_ROLLS_OUT		
#define CORN_ROLLS_STOP 				I2C_SLAVEDSPIC_MODE_CORN_ROLLS_STOP			

#define HARVEST_CORN						I2C_SLAVEDSPIC_MODE_HARVEST_CORN
#define OUT_CORNS								I2C_SLAVEDSPIC_MODE_OUT_CORNS

#define HARVEST_ORANGE					I2C_SLAVEDSPIC_MODE_HARVEST_ORANGE
#define ARM_GOTO_AH							I2C_SLAVEDSPIC_MODE_ARM_GOTO_AH

#define SET_COUNT								I2C_SLAVEDSPIC_MODE_SET_COUNT

#define WAIT               I2C_SLAVEDSPIC_MODE_WAIT
#define INIT               I2C_SLAVEDSPIC_MODE_INIT
#define EXIT               I2C_SLAVEDSPIC_MODE_EXIT

#define TOMATO				0	
#define ORANGE				1

//static void state_do_eject(uint8_t arm_num, uint8_t pump_num, uint8_t old_mode);

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static struct vt100 local_vt100;
static volatile uint8_t prev_state;

//static uint8_t pickup_side;
static volatile uint8_t changed = 0;

uint8_t state_debug = 0;


void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	changed = 1;
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);
	return 0;
}

/* check that state is the one in parameter and that state did not
 * changed */
uint8_t state_check(uint8_t mode)
{
	int16_t c;
	if (mode != mainboard_command.mode)
		return 0;

	if (changed)
		return 0;

	/* force quit when CTRL-C is typed */
	c = cmdline_getchar();
	if (c == -1)
		return 1;
	if (vt100_parser(&local_vt100, c) == KEY_CTRL_C) {
		mainboard_command.mode = EXIT;
		return 0;
	}
	return 1;
}

uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}

static void state_do_hide_arm(void)
{
	uint8_t err;
	
	if (!state_check(HIDE_ARM))
			return;
	
	slavedspic.status = I2C_BUSY;
	
	arm_goto_hide_intermediate();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	

	arm_goto_hide_final();
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	

	slavedspic.status = I2C_IDLE;
	mainboard_command.mode = WAIT;
}

static void state_do_show_arm(void)
{
	uint8_t err;
	
	if (!state_check(SHOW_ARM))
			return;
	
	slavedspic.status = I2C_BUSY;
	
	arm_goto_hide_intermediate();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	

	arm_goto_prepare_get_ball();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	

	slavedspic.status = I2C_IDLE;
	mainboard_command.mode = WAIT;
}


static void state_do_prepare_harvest_ball(void)
{
	uint8_t err;
	
	if (!state_check(PREPARE_HARVEST_BALL))
			return;

	slavedspic.status = I2C_BUSY;	
	arm_goto_prepare_get_ball();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	

	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}

#if 0
static void state_do_harvest_tomato(void)
{
	uint8_t err, try_countdown;
	
	if (!state_check(HARVEST_TOMATO))
			return;
			
	slavedspic.status = I2C_BUSY_BE_QUIET;
	
	if(slavedspic.balls_count == 14){
		mainboard_command.mode = WAIT;
		return;	
	}

	pump_on();
	
	arm_goto_get_tomato_intermediate();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);

	WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
			
	arm_goto_get_tomato_final();
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		
	WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),100);

#define NUM_TRYES 6
	try_countdown = NUM_TRYES;
	while(sensor_get(S_SUCKER_OBJ) && try_countdown)
	{
		/* tomato catched */
		if(sensor_get(S_VACUUM)){
			
			try_countdown--;
						
			arm_goto_up_tomato();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);

			if(!sensor_get(S_VACUUM))
				goto fin;	
				
			/* last tomato hold in arm */
			if(slavedspic.balls_count == 13){
				slavedspic.balls_count++;
				
				arm_goto_hold_last_tomato();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
				
				goto fin;						
			}
										
			arm_goto_put_tomato_intermediate();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			/* tomato is in safe position */
			slavedspic.status = I2C_BUSY;
			
			/* wait for transitory */
			wait_ms(100);
							
			if(!sensor_get(S_VACUUM))
				goto fin;				
	
			arm_goto_put_tomato_final();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		
			/* wait for transitory */
			wait_ms(50);
			
			if(sensor_get(S_VACUUM)){
				slavedspic.balls_count++;
				
				pump_off();
				wait_ms(200);
				WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),200);
				
				if(slavedspic.balls_count == 13){				
					if(finger_get_side(BALL_FINGER_AX12) == I2C_LEFT_SIDE)	
						ball_finger_goto(BALL_FINGER_LEFT_LAST);
					else
						ball_finger_goto(BALL_FINGER_RIGHT_LAST);						
				}
				else{
					if(finger_get_side(BALL_FINGER_AX12) == I2C_LEFT_SIDE)	
						ball_finger_goto(BALL_FINGER_RIGHT);
					else
						ball_finger_goto(BALL_FINGER_LEFT);						
				}
				
				// XXX improve syncro?
				wait_ms(200);
			}
			
	fin:
			try_countdown = 0;
			
			if(slavedspic.balls_count != 14)
				pump_off();			
	
		}
		/* tomato not yet catched */
		else{
			
			arm_goto_get_tomato_intermediate();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
			
			arm_goto_get_tomato_final();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
			
			try_countdown--;
		}
	}		
	
	if(slavedspic.balls_count != 14){
		pump_off();			

		if((try_countdown != NUM_TRYES) && (try_countdown != 0)){
			arm_goto_put_tomato_intermediate();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		}
			
		arm_goto_prepare_get_tomato();
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
	}	
	
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}
#endif

static void state_do_putin_finger_ball(void)
{	
	if (!state_check(PUTIN_FINGER_BALL))
			return;
		
	slavedspic.status = I2C_BUSY;				
	
	if(finger_get_side(BALL_FINGER_AX12) == I2C_LEFT_SIDE)	
		ball_finger_goto(BALL_FINGER_RIGHT);
	else
		ball_finger_goto(BALL_FINGER_LEFT);						
	
	slavedspic.status = I2C_IDLE;				
	mainboard_command.mode = WAIT;
}

static void state_do_arm_pump_on(void)
{	
	if (!state_check(ARM_PUMP_ON))
			return;

	slavedspic.status = I2C_BUSY;
	pump_set(PUMP_ON);

	slavedspic.status = I2C_IDLE;				
	mainboard_command.mode = WAIT;
}

static void state_do_arm_pump_off(void)
{	
	if (!state_check(ARM_PUMP_OFF))
			return;
			
	slavedspic.status = I2C_BUSY;
	pump_set(PUMP_OFF);
	
	slavedspic.status = I2C_IDLE;				
	mainboard_command.mode = WAIT;
}

static void state_do_corn_rolls_in(void)
{	
	if (!state_check(CORN_ROLLS_IN))
			return;

	slavedspic.status = I2C_BUSY;
	corn_rolls_set(ROLLS_MODE_IN, 16535);
	
	slavedspic.status = I2C_IDLE;				
	mainboard_command.mode = WAIT;
}

static void state_do_corn_rolls_out(void)
{	
	if (!state_check(CORN_ROLLS_OUT))
			return;
			
	slavedspic.status = I2C_BUSY;
	corn_rolls_set(ROLLS_MODE_OUT, 16535);
	
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}

static void state_do_corn_rolls_stop(void)
{	
	if (!state_check(CORN_ROLLS_STOP))
			return;
	
	slavedspic.status = I2C_BUSY;
	corn_rolls_set(ROLLS_MODE_OUT, 0);
		
	slavedspic.status = I2C_IDLE;	
	mainboard_command.mode = WAIT;
}

static void state_do_harvest_corn(void)
{	
	uint16_t position_actual=0, position_goal=0;
	int16_t position_error;
	uint8_t timeout=0;
	
	if (!state_check(HARVEST_CORN))
			return;
	
	slavedspic.status = I2C_BUSY;
	
	/* up to seven corns */
	if(slavedspic.corns_count == 7){
		slavedspic.status = I2C_IDLE;	
		mainboard_command.mode = WAIT;
		return;
	}
	
	/* be sure that push servo is inside */
	servo_corn_push_in();
	
	/* rolls in */
	corn_rolls_set(ROLLS_MODE_IN, 16535);
		
	/* wait corn inside */
	WAIT_COND_OR_TIMEOUT(sensor_get(S_CORN_INSIDE_EXTRA),2000);
	
	if(sensor_get(S_CORN_INSIDE_EXTRA))
	{
		slavedspic.corns_count++;	
		if(slavedspic.corns_count < 7){
			
			/* store in one side */	
			if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE){	
				servo_corn_hold_right_up();
				wait_ms(200);
				
				slavedspic.corns_right_count++;
				if(slavedspic.corns_right_count == 3){
					corn_finger_goto(CORN_FINGER_RIGHT_3TH);
					position_goal = CORN_FINGER_RIGHT_3TH;
				}
				else{
					corn_finger_goto(CORN_FINGER_RIGHT_INTER);
					position_goal = CORN_FINGER_RIGHT_INTER;
				}
			}
			else{
				servo_corn_hold_left_up();
				wait_ms(200);
	
				slavedspic.corns_left_count++;
				if(slavedspic.corns_left_count == 3){	
					corn_finger_goto(CORN_FINGER_LEFT_3TH);
					position_goal = CORN_FINGER_LEFT_3TH;
				}						
				else{
					corn_finger_goto(CORN_FINGER_LEFT_INTER);					
					position_goal = CORN_FINGER_LEFT_INTER;
				}
			}
			
			/* hold corns both sides*/	
			wait_ms(100);

		 	/* prevent corn jam */
		 	timeout = 50;
		 	position_error = 50;
			while(ABS(position_error)>10 &&timeout){	 
				if(!ax12_user_read_int(&gen.ax12, CORN_FINGER_AX12, AA_PRESENT_POSITION_L, &position_actual)){
					position_error = position_goal-position_actual;	
					//printf("pos_goal = %d pos_actual = %d pos_error = %d\n\r", position_goal, position_actual, position_error);		
				}
				wait_ms(100);
				timeout--;
			}	

			servo_corn_hold_left_down();
			servo_corn_hold_right_down();

		}
		else{			
			/* store in one side */	
			if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE)	
				corn_finger_goto(CORN_FINGER_RIGHT_LAST);
			else
				corn_finger_goto(CORN_FINGER_LEFT_LAST);					

			/* wait to get position */
			wait_ms(500);
		}	
 	}
 	
 	/* prevent asimetrical store and last corn side store */
 	if(slavedspic.corns_count < 7 && slavedspic.corns_count!=0){
		if(slavedspic.corns_left_count < slavedspic.corns_right_count){			
			if(slavedspic.corns_right_count == 3)	
				corn_finger_goto(CORN_FINGER_RIGHT_3TH);
			else
				corn_finger_goto(CORN_FINGER_RIGHT_INTER);					
		}		
		else if(slavedspic.corns_right_count < slavedspic.corns_left_count){
			if(slavedspic.corns_left_count == 3)	
				corn_finger_goto(CORN_FINGER_LEFT_3TH);
			else
				corn_finger_goto(CORN_FINGER_LEFT_INTER);					
		}		
		else if(slavedspic.corns_count == 6){
				corn_finger_goto(CORN_FINGER_LEFT_3TH);
		}		

	}
 		
 	/* end, down hoders, stop rolls */	
	corn_rolls_set(ROLLS_MODE_OUT, 0);
	slavedspic.status = I2C_IDLE;	
	mainboard_command.mode = WAIT;
}

void corn_push(void)
{
	servo_corn_push_out();
	wait_ms(400);
	servo_corn_push_in();	
}

void empty_last_corn(uint8_t corns_count)
{
	/* last corn, in center */
	if(corns_count == 7){
		
			corn_rolls_set(ROLLS_MODE_OUT,16535);
			wait_ms(100);
		
			if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE)	
				corn_finger_goto(CORN_FINGER_LEFT_3TH);
			else
				corn_finger_goto(CORN_FINGER_RIGHT_3TH);	
				
			wait_ms(100);
 try:
			corn_push();
			
			/* wait to corn a bit out */
			WAIT_COND_OR_TIMEOUT(!sensor_get(S_CORN_INSIDE_EXTRA),300);
			
			if(sensor_get(S_CORN_INSIDE_EXTRA)){
				corn_rolls_set(ROLLS_MODE_IN,16535);
				wait_ms(500);
				corn_rolls_set(ROLLS_MODE_OUT,16535);
				goto try;
			}
	
			slavedspic.corns_count--;

			//corn_rolls_set(ROLLS_MODE_OUT,0);
	}
}

void empty_left_corns(uint8_t corns_left_count)
{
	// NOTE: SENSOR_INSIDE threshold adjust to 200
	uint8_t sensor_count, i;
	uint16_t timeout;
	
	if(corns_left_count > 0){
		
		STMCH_DEBUG("%s corns_left_count = %d", __FUNCTION__, corns_left_count);
					
		/* free corns */
		servo_corn_hold_left_up();
		
		if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE){
			corn_finger_goto(CORN_FINGER_RIGHT_3TH);
			corn_rolls_set(ROLLS_MODE_LEFT, 16335);
			wait_ms(500);	
		}
		else{
			corn_finger_goto(CORN_FINGER_RIGHT_3TH);
		}
		
		/* prevent corn jam residues */		
		servo_corn_hold_right_down();				

					
		/* out corns one by one */
		corn_rolls_set(ROLLS_MODE_OUT, 16335);

		for(i=0; i<corns_left_count; i++){
			
			STMCH_DEBUG("%s corn %d", __FUNCTION__, i+1);
			
			/* wait to fall corn */
			sensor_count = 0;
			timeout = 1200;
			while((sensor_count < 100) && (timeout!=0)){
				
				if(sensor_get(S_CORN_INSIDE))
					sensor_count+=10;
				else
					sensor_count = 0;
			
				wait_ms(10);
				timeout-=10;
			}
			
			/* push it or abort */
			if(sensor_get(S_CORN_INSIDE_EXTRA)){
				slavedspic.corns_left_count--;
				slavedspic.corns_count--;
 try_push:
				corn_push();
			}
			else{
				STMCH_DEBUG("%s end with break (%d)", __FUNCTION__, slavedspic.corns_left_count);			
				break;
			}
			
			/* wait to corn a bit out */
			WAIT_COND_OR_TIMEOUT((!sensor_get(S_CORN_INSIDE)) && (!sensor_get(S_CORN_INSIDE_EXTRA)),500);
			
			if(sensor_get(S_CORN_INSIDE) || sensor_get(S_CORN_INSIDE_EXTRA))
				goto try_push;
		}	
	}
	
	/* wait last corn */
	wait_ms(500);
	
	/* stop rolls */
	//corn_rolls_set(ROLLS_MODE_OUT,0);
}


void empty_right_corns(uint8_t corns_right_count)
{
	// NOTE: SENSOR_INSIDE threshold adjust to 200
	uint8_t sensor_count, i;
	uint16_t timeout;
	
	if(corns_right_count > 0){
		
		STMCH_DEBUG("%s corns_right_count = %d", __FUNCTION__, corns_right_count);
					
		/* free corns */
		servo_corn_hold_right_up();
		
		if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE){
			corn_finger_goto(CORN_FINGER_LEFT_3TH);
		}
		else{
			corn_finger_goto(CORN_FINGER_LEFT_3TH);
			corn_rolls_set(ROLLS_MODE_RIGHT, 16335);
			wait_ms(500);	
		}
	
		/* prevent corn jam residues */		
		servo_corn_hold_left_down();				
					
		/* out corns one by one */
		corn_rolls_set(ROLLS_MODE_OUT, 16335);

		for(i=0; i<corns_right_count; i++){
			
			STMCH_DEBUG("%s corn %d", __FUNCTION__, i+1);
			
			/* wait to fall corn */
			sensor_count = 0;
			timeout = 1200;
			while((sensor_count < 100) && (timeout!=0)){
				
				if(sensor_get(S_CORN_INSIDE))
					sensor_count+=10;
				else
					sensor_count = 0;
			
				wait_ms(10);
				timeout-=10;
			}
			
			/* push it or abort */
			if(sensor_get(S_CORN_INSIDE_EXTRA)){
				slavedspic.corns_right_count--;
				slavedspic.corns_count--;
 try_push:
				corn_push();
			}
			else{
				STMCH_DEBUG("%s end with break (%d)", __FUNCTION__, slavedspic.corns_right_count);
				break;
			}
			
			/* wait to corn a bit out */
			WAIT_COND_OR_TIMEOUT((!sensor_get(S_CORN_INSIDE)) && (!sensor_get(S_CORN_INSIDE_EXTRA)),500);
			
			if(sensor_get(S_CORN_INSIDE) || sensor_get(S_CORN_INSIDE_EXTRA))
				goto try_push;
		}	
	}
	
	/* wait last corn */
	wait_ms(500);
	
	/* stop rolls */
	//corn_rolls_set(ROLLS_MODE_OUT,0);
}


static void state_do_out_corns(void)
{			
	
#if 0	
	slavedspic.corns_count = 7;
	slavedspic.corns_left_count = 3;
	slavedspic.corns_right_count = 3;
#endif	
	
	if (!state_check(OUT_CORNS))
			return;
	
	
	if(slavedspic.corns_count == 0){
		return;	
	}
	
	slavedspic.status = I2C_BUSY;

	/* be sure that push servo is inside */
	servo_corn_push_in();

	/* try empty last corn */
	empty_last_corn(slavedspic.corns_count);

	if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE){
		empty_right_corns(slavedspic.corns_right_count);
		empty_left_corns(slavedspic.corns_left_count);		
	}
	else{
		empty_left_corns(slavedspic.corns_left_count);				
		empty_right_corns(slavedspic.corns_right_count);
	}
	
	/* prepare finger for input next corn */
	if(slavedspic.corns_left_count == slavedspic.corns_right_count){
		if(finger_get_side(CORN_FINGER_AX12) == I2C_LEFT_SIDE){
			
			if(slavedspic.corns_left_count == 3)
				corn_finger_goto(CORN_FINGER_LEFT_3TH);
			else
				corn_finger_goto(CORN_FINGER_LEFT_INTER);
		}
		else{
			if(slavedspic.corns_right_count == 3)
				corn_finger_goto(CORN_FINGER_RIGHT_3TH);
			else
				corn_finger_goto(CORN_FINGER_RIGHT_INTER);
		}
	}
	else if(slavedspic.corns_left_count < slavedspic.corns_right_count){
		corn_finger_goto(CORN_FINGER_RIGHT_INTER);
	}
	else if(slavedspic.corns_left_count > slavedspic.corns_right_count){
		corn_finger_goto(CORN_FINGER_LEFT_INTER);		
	}
	
	servo_corn_hold_left_down();				
	servo_corn_hold_right_down();				
	
	/* wait last corn out */
	wait_ms(500);
	
	corn_rolls_set(ROLLS_MODE_OUT,0);
	slavedspic.status = I2C_IDLE;	
	mainboard_command.mode = WAIT;
}	



void arm_goto_get_orange(void)
{
#define ORANGE_ANGLE_1 0
#define ORANGE_HEIGHT_1 0

#define ORANGE_ANGLE_2 0
#define ORANGE_HEIGHT_2 0

#define ORANGE_ANGLE_3 0
#define ORANGE_HEIGHT_3 0

#define ORANGE_ANGLE_4 0
#define ORANGE_HEIGHT_4 0

#define ORANGE_ANGLE_5 0
#define ORANGE_HEIGHT_5 0

#define ORANGE_ANGLE_6 0
#define ORANGE_HEIGHT_6 0

	uint8_t err;
	int16_t angle=0, height=0;
		
	if(mainboard_command.harvest_orange.position == CUSTOM_POSITION){
		angle = mainboard_command.harvest_orange.angle;
		height = mainboard_command.harvest_orange.height;
		
		arm_do_xy(&right_arm, angle, height);
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	
		
	}
	else{
		if(mainboard_command.harvest_orange.position == ORANGE_POSITION_1){
			angle = ORANGE_ANGLE_1;
			height = ORANGE_HEIGHT_1;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_2){
			angle = ORANGE_ANGLE_2;
			height = ORANGE_HEIGHT_2;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_3){
			angle = ORANGE_ANGLE_3;
			height = ORANGE_HEIGHT_3;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_4){
			angle = ORANGE_ANGLE_4;
			height = ORANGE_HEIGHT_4;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_5){
			angle = ORANGE_ANGLE_5;
			height = ORANGE_HEIGHT_5;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_6){
			angle = ORANGE_ANGLE_6;
			height = ORANGE_HEIGHT_6;
		}
		
		/* catch orange */
		arm_do_xy(&right_arm, angle, height);
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);		
	}
		
}

void arm_goto_up_orange(void)
{
#define DELTA_H 50
#define DELTA_A 30
	
	uint8_t err;
	int16_t angle=0, height=0;
		
	if(mainboard_command.harvest_orange.position == CUSTOM_POSITION){
		angle = mainboard_command.harvest_orange.angle;
		height = mainboard_command.harvest_orange.height;
		
		arm_do_xy(&right_arm, angle, height);
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);	
		
	}
	else{
		if(mainboard_command.harvest_orange.position == ORANGE_POSITION_1){
			angle = ORANGE_ANGLE_1;
			height = ORANGE_HEIGHT_1;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_2){
			angle = ORANGE_ANGLE_2;
			height = ORANGE_HEIGHT_2;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_3){
			angle = ORANGE_ANGLE_3;
			height = ORANGE_HEIGHT_3;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_4){
			angle = ORANGE_ANGLE_4;
			height = ORANGE_HEIGHT_4;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_5){
			angle = ORANGE_ANGLE_5;
			height = ORANGE_HEIGHT_5;
		}
		else if(mainboard_command.harvest_orange.position == ORANGE_POSITION_6){
			angle = ORANGE_ANGLE_6;
			height = ORANGE_HEIGHT_6;
		}
		
		/* up orange */	
		arm_do_xy(&right_arm, angle-DELTA_A, height-DELTA_H);
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);		
	}
		
}

void harvest_ball(uint8_t type)
{
	uint8_t err, try_countdown;
	uint16_t position_actual=0, position_goal=0;
	int16_t position_error, position_th;
	uint8_t timeout=0;

	
	slavedspic.status = I2C_BUSY_BE_QUIET;
	
	if(slavedspic.balls_count == 14){
		mainboard_command.mode = WAIT;
		return;	
	}

	pump_on();
	
	if(type == TOMATO){
		arm_goto_get_tomato_intermediate();
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
	
		WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
				
		arm_goto_get_tomato_final();
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			STMCH_DEBUG(PSTR("arm err %x\r\n"), err);		
	}
	else
		arm_goto_get_orange();


	WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),500);
		
	if(type == ORANGE)
		wait_ms(mainboard_command.harvest_orange.vacuum_time_div10*10);

#define NUM_TRYES 6
	try_countdown = NUM_TRYES;	
	while(sensor_get(S_SUCKER_OBJ) && try_countdown)
	{
		/* ball catched */
		if(sensor_get(S_VACUUM)){

			try_countdown--;
					
			if(type == TOMATO){			
				arm_goto_up_tomato();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			}
			else{
				//arm_goto_up_orange();
				
				arm_goto_prepare_get_ball();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
				arm_goto_prepare_get_ball2();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			}
				
			if(!sensor_get(S_VACUUM))
				goto fin;	

			/* last ball hold in arm */
			if(slavedspic.balls_count == 13){
				slavedspic.balls_count++;
				
				arm_goto_hold_last_ball();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
				
				goto fin;						
			}
										
			arm_goto_put_ball_intermediate();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			
			if(type == TOMATO){				
				arm_goto_put_ball_intermediate2();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
					STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
			}
			
			/* tomato is in safe position */
			slavedspic.status = I2C_BUSY;
			
			/* wait for transitory */
			wait_ms(100);
								
			if(!sensor_get(S_VACUUM))
				goto fin;				
	
			arm_goto_put_ball_final();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		
			/* wait for transitory */
			wait_ms(50);
			
			if(sensor_get(S_VACUUM)){
				slavedspic.balls_count++;
				
				pump_off();
				WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),500);
				//wait_ms(250);
				
				if(slavedspic.balls_count == 13){				
					if(finger_get_side(BALL_FINGER_AX12) == I2C_LEFT_SIDE){	
						ball_finger_goto(BALL_FINGER_LEFT_LAST);
						position_goal = BALL_FINGER_LEFT_LAST;
					}
					else{
						ball_finger_goto(BALL_FINGER_RIGHT_LAST);
						position_goal = BALL_FINGER_RIGHT_LAST;
					}						
				}
				else{
					if(finger_get_side(BALL_FINGER_AX12) == I2C_LEFT_SIDE){	
						ball_finger_goto(BALL_FINGER_RIGHT);
						position_goal = BALL_FINGER_RIGHT;
					}
					else{
						ball_finger_goto(BALL_FINGER_LEFT);	
						position_goal = BALL_FINGER_LEFT;
					}	
				}
				
				// XXX improve syncro?
				//wait_ms(200);

				/* prevent corn jam */
			 	timeout = 5;
			 	position_error = 1000;
			 	if(type == TOMATO)
			 		position_th = 300;
			 	else
			 		position_th = 200;
				while(ABS(position_error)>position_th && timeout){	 
					if(!ax12_user_read_int(&gen.ax12, BALL_FINGER_AX12, AA_PRESENT_POSITION_L, &position_actual)){
						position_error = position_goal-position_actual;	
						//printf("pos_goal = %d pos_actual = %d pos_error = %d\n\r", position_goal, position_actual, position_error);		
					}
					wait_ms(100);
					timeout--;
				}	
			
			}
			
	fin:
			try_countdown = 0;
			
			if(slavedspic.balls_count != 14)
				pump_off();			
	
		}
		/* ball not yet catched */
		else{
			
			if(type == TOMATO){
				arm_goto_get_tomato_intermediate();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
				
				WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
				
				arm_goto_get_tomato_final();
				err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
				if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
				
				WAIT_COND_OR_TIMEOUT(sensor_get(S_VACUUM),50);
				
				try_countdown--;
			}
			else
				try_countdown = 0;
		}
	}		
	
	if(slavedspic.balls_count != 14){
		pump_off();			

		if((try_countdown != NUM_TRYES)){ //|| (try_countdown != 0)){
		//if((try_countdown < NUM_TRYES) && (try_countdown > 0)){
			arm_goto_put_ball_intermediate();
			err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
			if (err != ARM_TRAJ_END)
				STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		}
			
		arm_goto_prepare_get_ball();
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
		STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
	}	
	
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}

static void state_do_harvest_tomato(void)
{
	if (!state_check(HARVEST_TOMATO))
			return;
			
	harvest_ball(TOMATO);
	
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}

static void state_do_harvest_orange(void)
{
	if (!state_check(HARVEST_ORANGE))
			return;
			
	harvest_ball(ORANGE);
	
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}

static void state_do_arm_goto_ah(void)
{
	uint8_t err;
	
	if (!state_check(ARM_GOTO_AH))
			return;
	
	slavedspic.status = I2C_BUSY;			
			
	arm_do_xy(&right_arm, mainboard_command.arm_goto.angle, mainboard_command.arm_goto.height);
	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
	STMCH_DEBUG(PSTR("arm err %x\r\n"), err);
		
	slavedspic.status = I2C_IDLE;		
	mainboard_command.mode = WAIT;
}


static void state_do_set_count(void)
{	
	if (!state_check(SET_COUNT))
			return;
	
	slavedspic.status = I2C_BUSY;
	
	if(mainboard_command.set_count.value == BALLS_COUNT)
		slavedspic.balls_count = mainboard_command.set_count.value;
	else if(mainboard_command.set_count.value == CORNS_COUNT)
		slavedspic.corns_count = mainboard_command.set_count.value;
			
	slavedspic.status = I2C_IDLE;	
	mainboard_command.mode = WAIT;
}


///* move finger if we are not in lazy harvest */
//void state_finger_goto(uint8_t mode, uint16_t position)
//{
//	if (mode == LAZY_HARVEST)
//		return;
//	finger_goto(position);
//}

//void state_manivelle(int16_t step_deg)
//{
//	double add_h = 0.;
//	double add_d = 160.;
//	double l = 70.;
//	double step = RAD(step_deg);
//	microseconds us;
//	double al = RAD(0);
//	double ar = RAD(180);
//
//	time_wait_ms(500);
//
//	us = time_get_us2();
//	while (1) {
//		al += step;
//		ar += step;
//		arm_do_xy(&left_arm, add_d+l*sin(al), add_h+l*cos(al), 10);
//		arm_do_xy(&right_arm, add_d+l*sin(ar), add_h+l*cos(ar), 10);
//		time_wait_ms(25);
//		if (time_get_us2() - us > (4000L * 1000L))
//			break;
//	}
//}

//static void state_do_manivelle(void)
//{
//	if (!state_check(MANIVELLE))
//		return;
//	state_manivelle(30);
//	while (state_check(MANIVELLE));
//}


///* manual mode, arm position is sent from mainboard */
//static void state_do_manual(void)
//{
//	if (!state_check(MANUAL))
//		return;
//	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
//	while (state_check(MANUAL));
//}

/* wait mode */
static void state_do_wait(void)
{
	if (!state_check(WAIT))
		return;
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(WAIT));
}

/* init mode */
static void state_do_init(void)
{
	if (!state_check(INIT))
		return;
	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
	while (state_check(INIT));
}


/* main state machine */
void state_machine(void)
{
	while (state_get_mode() != EXIT) {
		changed = 0;
		state_do_init();
		state_do_hide_arm();
		state_do_show_arm();
		state_do_prepare_harvest_ball();
		state_do_harvest_tomato();
		state_do_putin_finger_ball();
		state_do_arm_pump_on();
		state_do_arm_pump_off();
		state_do_corn_rolls_in();
		state_do_corn_rolls_out();
		state_do_corn_rolls_stop();
		state_do_harvest_corn();
		state_do_out_corns();
		state_do_harvest_orange();
		state_do_arm_goto_ah();
		state_do_set_count();

		state_do_wait();
		
//		state_do_manual();
//		state_do_manivelle();
	}
}

void state_init(void)
{
	vt100_init(&local_vt100);
	mainboard_command.mode = WAIT;
	
	/* ball store system */
	AX12_write_int(&gen.ax12, BALL_FINGER_AX12, AA_MOVING_SPEED_L, 100);
	ball_finger_goto(BALL_FINGER_LEFT);
	slavedspic.balls_count = 0;

	/* arm */
	pump_off();	

	/* corn store system */
	AX12_write_int(&gen.ax12, CORN_FINGER_AX12, AA_MOVING_SPEED_L, 1023);
	corn_finger_goto(CORN_FINGER_LEFT);
	servo_corn_hold_right_up();
	servo_corn_hold_left_up();
	servo_corn_push_in();
	slavedspic.corns_count = 0;
	slavedspic.corns_right_count = 0;
	slavedspic.corns_left_count = 0;	

}
