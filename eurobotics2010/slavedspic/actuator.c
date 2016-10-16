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
 *  Revision : $Id: actuator.c,v 1.4 2009/04/24 19:30:41 zer0 Exp $
 *
 */

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <dac_mc.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "main.h"

#define FINGER_DEBUG(args...) DEBUG(E_USER_FINGER, args)
#define FINGER_NOTICE(args...) NOTICE(E_USER_FINGER, args)
#define FINGER_ERROR(args...) ERROR(E_USER_FINGER, args)

struct finger {
	int8_t event;
	uint16_t destination;
	uint8_t num;
};

struct finger ball_finger;
struct finger corn_finger;

static void finger_goto_cb(void *);



void pwm_right_corn_mot_set(int16_t value)
{
	if(value == 0){
			pwm_mc_init(&gen.pwm_mc_mod1_ch2, 3000, 
							CH2_COMP&PDIS1H&PDIS1L&PDIS2H&PDIS2L&PDIS3H&PDIS3L);			
	}
	else{
			pwm_mc_init(&gen.pwm_mc_mod1_ch2, 3000, 
							CH2_COMP&PDIS1H&PDIS1L&PEN2H&PEN2L&PDIS3H&PDIS3L);		
	}		
	pwm_mc_set(&gen.pwm_mc_mod1_ch2, value);	
}

void pwm_left_corn_mot_set(int16_t value)
{
	if(value == 0){
		pwm_mc_init(&gen.pwm_mc_mod2_ch1, 3000, 
								CH1_COMP&PDIS1H&PDIS1L);
	}
	else{
		pwm_mc_init(&gen.pwm_mc_mod2_ch1, 3000, 
							CH1_COMP&PEN1H&PEN1L);
	}	
	pwm_mc_set(&gen.pwm_mc_mod2_ch1, value);	
}	

void corn_rolls_set(uint8_t mode, int16_t val)
{
	int16_t val_right=0;
	int16_t val_left=0;
	
	val = ABS(val);
	
	if(mode == ROLLS_MODE_OUT){
		val_right = -val;
		val_left = -val;
	}
	else if(mode == ROLLS_MODE_IN){
		val_right = val;
		val_left = val;
	}
	else if(mode == ROLLS_MODE_RIGHT){
		val_right = val;
		val_left = -val;
	}
	else if(mode == ROLLS_MODE_LEFT){
		val_right = -val;
		val_left = val;
	}
	
	pwm_right_corn_mot_set(val_right);
	pwm_left_corn_mot_set(val_left);
		
}

/* schedule a single event for the finger */
static void finger_schedule_event(struct finger *finger)
{
	uint8_t flags;
	int8_t ret;

	IRQ_LOCK(flags);
	ret = scheduler_add_event(SCHEDULER_SINGLE,
				  (void *)finger_goto_cb,
				  finger, 1, ARM_PRIO);
	if (ret == -1) {
		IRQ_UNLOCK(flags);
		FINGER_ERROR("Cannot load finger event");
		return;
	}
	finger->event = ret;
	IRQ_UNLOCK(flags);
}

static void finger_goto_cb(void *data)
{
	uint8_t flags;
	struct finger *finger = data;
	uint16_t position;
	uint8_t num;

	IRQ_LOCK(flags);
	finger->event = -1;
	position = finger->destination;
	num = finger->num;
	IRQ_UNLOCK(flags);
	FINGER_DEBUG("goto_cb %d", position);
	ax12_user_write_int(&gen.ax12,num,
			    AA_GOAL_POSITION_L, position);
}

/* load an event that will move the ax12 for us */
void ball_finger_goto(uint16_t position)
{
	uint8_t flags;
	FINGER_NOTICE("goto %d", position);

	IRQ_LOCK(flags);
	ball_finger.destination = position;
	ball_finger.num = BALL_FINGER_AX12;
	if (ball_finger.event != -1) {
		IRQ_UNLOCK(flags);
		return; /* nothing to do, event already scheduled */
	}
	IRQ_UNLOCK(flags);
	finger_schedule_event(&ball_finger);
}

void corn_finger_goto(uint16_t position)
{
	uint8_t flags;
	FINGER_NOTICE("goto %d", position);

	IRQ_LOCK(flags);
	corn_finger.destination = position;
	corn_finger.num = CORN_FINGER_AX12;
	if (corn_finger.event != -1) {
		IRQ_UNLOCK(flags);
		return; /* nothing to do, event already scheduled */
	}
	IRQ_UNLOCK(flags);
	finger_schedule_event(&corn_finger);
}

static void finger_init(void)
{
	ball_finger.event = -1;
	ball_finger.destination = 0;

	corn_finger.event = -1;
	corn_finger.destination = 0;

	/* XXX set pos ? */
}

uint16_t finger_get_goal_pos(uint8_t num)
{
	if(num == BALL_FINGER_AX12)
		return ball_finger.destination;
	else if(num == CORN_FINGER_AX12)
		return corn_finger.destination;

	return -1;
}

uint8_t finger_get_side(uint8_t num)
{
	if(num == BALL_FINGER_AX12){
		if (ball_finger.destination <= BALL_FINGER_CENTER)
			return I2C_LEFT_SIDE;
		return 0;
	}
	else if(num == CORN_FINGER_AX12){
		if (corn_finger.destination <= CORN_FINGER_CENTER)
			return I2C_LEFT_SIDE;
		return 0;
	}
	
	return -1;
}

#define SERVO_CORN_HOLD_R_UP		150
#define SERVO_CORN_HOLD_R_DOWN	350 //750
#define SERVO_CORN_HOLD_L_UP		650 
#define SERVO_CORN_HOLD_L_DOWN	450 //50
#define SERVO_CORN_PUSH_OUT			320
#define SERVO_CORN_PUSH_IN			171

void servo_corn_hold_right_up(void)
{
	pwm_servo_set(SERVO_CORN_HOLD_R, SERVO_CORN_HOLD_R_UP);
}

void servo_corn_hold_right_down(void)
{
	pwm_servo_set(SERVO_CORN_HOLD_R, SERVO_CORN_HOLD_R_DOWN);
}

void servo_corn_hold_left_up(void)
{
	pwm_servo_set(SERVO_CORN_HOLD_L, SERVO_CORN_HOLD_L_UP);
}

void servo_corn_hold_left_down(void)
{
	pwm_servo_set(SERVO_CORN_HOLD_L, SERVO_CORN_HOLD_L_DOWN);
}


void servo_corn_push_out(void)
{
	pwm_servo_set(SERVO_CORN_PUSH, SERVO_CORN_PUSH_OUT);
}

void servo_corn_push_in(void)
{
	pwm_servo_set(SERVO_CORN_PUSH, SERVO_CORN_PUSH_IN);	
}

void actuator_init(void)
{
	finger_init();
}
