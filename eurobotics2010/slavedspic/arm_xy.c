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
 *  Revision : $Id: arm_xy.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 * Fabrice DESCLAUX <serpilliere@droids-corp.org>
 * Olivier MATZ <zer0@droids-corp.org>
 */

#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/pgmspace.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>

#include "main.h"
#include "cmdline.h"
#include "arm_xy.h"
#include "ax12_user.h"

#define ARM_DEBUG(args...) DEBUG(E_USER_ARM, args)
#define ARM_NOTICE(args...) NOTICE(E_USER_ARM, args)
#define ARM_ERROR(args...) ERROR(E_USER_ARM, args)

#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

/* physical location/dimensions of arm */
#define ARM_S_LEN 120.
#define ARM_E_LEN 70.

#define ARM_S_H_MIN -120.
#define ARM_S_H_MAX 110.
#define	ARM_E_ANG_MAX 200.

/* timeout after 1 second if position is not reached */
#define ARM_GLOBAL_TIMEOUT 2000000L

/* timeout 100ms after position is reached if not in window */
#define ARM_WINDOW_TIMEOUT 200000L

/* default (template) period, but real one is variable */
#define ARM_PERIOD 50000L
#define ARM_MAX_H_DIST 300L //10L
#define ARM_MAX_A_DIST 300L

/* we pos reached, check arm in window every period */
#define ARM_SURVEY_PERIOD 25000UL /* in us */

/* number of steps/s */
#define ARM_AX12_MAX_SPEED (800L)

/* Maximum number of steps in one ARM_PERIOD */
#define ARM_MAX_E (((ARM_AX12_MAX_SPEED*ARM_PERIOD)/1000000L))
/* 4000 steps/CS => 800step/ms */
#define ARM_MAX_S ((800L*ARM_PERIOD)/1000L)


/* window limits in ax12/cs unit */
#define ARM_SHOULDER_WINDOW_POS  250
#define ARM_ELBOW_WINDOW_POS     8
#define ARM_ELBOW_WINDOW_SPEED   100
#define ARM_WRIST_WINDOW_POS     8
#define ARM_WRIST_WINDOW_SPEED   100

/* default and max speeds */
#define SHOULDER_DEFAULT_SPEED   800
#define ELBOW_DEFAULT_SPEED      0x3ff
#define SHOULDER_MAX_SPEED       800 //200
#define ELBOW_MAX_SPEED          0x3ff

/* window status flags */
#define SHOULDER_NOT_IN_WIN 1
#define ELBOW_NOT_IN_WIN    2

static void angle_rad2robot_r(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot);
static void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad);

static void arm_schedule_event(struct arm *arm, uint32_t time);

struct arm right_arm = {
	.config = {
		.angle_rad2robot = angle_rad2robot_r,
		.angle_robot2rad = angle_robot2rad_r,
		.elbow_ax12 = R_ELBOW_AX12,
	},
};


/* process shoulder + elbow angles from height and attack angle */
int8_t cart2angle(int32_t h_int, int32_t a_int, double *alpha, double *beta)
{
	double h, a;
	double elbow, shoulder;

	a = a_int;
	h = h_int;

	if ((h > ARM_S_H_MAX)|| (h < ARM_S_H_MIN) ||
			(a > ARM_E_ANG_MAX)|| (a < -ARM_E_ANG_MAX))
		return -1;
	
	shoulder = asin(h/ARM_S_LEN);
	elbow = RAD(a)-shoulder;
	
	*alpha = shoulder;
	*beta = elbow;

	return 0;
}


/* process height and attack angle from shoulder + elbow angles */
void angle2cart(double alpha, double beta, int32_t *h, int32_t *a)
{
	int32_t tmp_h, tmp_a;

	tmp_h = ARM_S_LEN * sin(alpha);
	tmp_a = alpha + beta;

	*h = tmp_h;
	*a = tmp_a;
}


/*** right arm */

#define ARM_RIGHT_S_OFFSET -3900. //-3900.
#define ARM_RIGHT_E_OFFSET 516.

/* convert an angle in radian into a robot-specific unit 
 * for shoulder and elbow */
static void angle_rad2robot_r(double shoulder_rad, double elbow_rad,
			      double *shoulder_robot, double *elbow_robot)
{
	*shoulder_robot = shoulder_rad * 4.0 * 1.4 * 3600. / (2*M_PI) + ARM_RIGHT_S_OFFSET;
	*elbow_robot = elbow_rad * 3.41 * 360. / (2*M_PI) + ARM_RIGHT_E_OFFSET;
}

/* convert  a robot-specific unit into an angle in radian 
 * for shoulder and elbow */
static void angle_robot2rad_r(double shoulder_robot, double elbow_robot,
			      double *shoulder_rad, double *elbow_rad)
{
	*shoulder_rad = ((shoulder_robot + ARM_RIGHT_S_OFFSET) * (2*M_PI))/(4.0 * 1.4 * 3600.);
	*elbow_rad = ((elbow_robot - ARM_RIGHT_E_OFFSET) * (2*M_PI))/(3.41 * 360.);
}


/*
 * Fill the arm_status structure according to request.
 *
 * return:
 *     0  => success
 *   < 0  => error
 */
static int8_t arm_do_step(struct arm *arm)
{
	const struct arm_config *conf = &arm->config;
	const struct arm_request *req = &arm->req;
	struct arm_status *status = &arm->status;

	int8_t ret;
	int32_t diff_h, diff_a; //diff_d; /* position delta in steps */
	int32_t next_h, next_a;	//next_d; /* next position in steps */
	int32_t h, a;

	double as_cur_rad, ae_cur_rad; 		/* current angle in rad */
	double as_next_rad, ae_next_rad; 	/* next angle in rad */
	double as_cur, ae_cur; 						/* current angle in angle_steps */
	double as_next, ae_next; 					/* next angle in angle_steps */

	int32_t as_diff, ae_diff; /* angle delta in angle_steps */
	int32_t s_speed, e_speed; /* elbow/shoulder speed in angle_steps */
	
	double as_coef, ae_coef;
	
	/* process diff between final request and current pos */
	diff_h = req->h_mm - status->h_mm;
	diff_a = req->a_deg - status->a_deg;
	ARM_NOTICE("goal:a=%ld,h=%ld cur:a=%ld,h=%ld diff:a=%ld,h=%ld",
		  req->a_deg, req->h_mm, status->a_deg, status->h_mm,
		  diff_a, diff_h);


	/* if distance to next point is too large, saturate it */
	h = ABS(diff_h);
	if (h > ARM_MAX_H_DIST) {
		diff_h = diff_h * ARM_MAX_H_DIST / h;
	}
	a = ABS(diff_a);
	if (a > ARM_MAX_A_DIST) {
		diff_a = diff_a * ARM_MAX_A_DIST / a;
	}

	ARM_NOTICE("l=%ld a=%ld ; after max dist: diff:a=%ld,h=%ld", h, a, diff_a, diff_h);
	
	
	/* process next position */
	next_h = status->h_mm + diff_h;
	next_a = status->a_deg + diff_a;
	ARM_DEBUG("next:a=%ld,h=%ld", next_a, next_h);

	/* calculate the current angle of arm in radian */
	ret = cart2angle(status->h_mm, status->a_deg, &as_cur_rad, &ae_cur_rad);
	if (ret)
		return ret;
	ARM_DEBUG("as_cur_rad=%f ae_cur_rad=%f", as_cur_rad, ae_cur_rad);

	/* calculate the next angle of arm in radian */
	ret = cart2angle(next_h, next_a, &as_next_rad, &ae_next_rad);
	if (ret)
		return ret;
	ARM_DEBUG("as_next_rad=%f ae_next_rad=%f", as_next_rad, ae_next_rad);

	/* convert radian in angle_steps */
	conf->angle_rad2robot(as_cur_rad, ae_cur_rad,
			     &as_cur, &ae_cur);
	ARM_DEBUG("as_cur=%f ae_cur=%f", as_cur, ae_cur);
	conf->angle_rad2robot(as_next_rad, ae_next_rad,
			     &as_next, &ae_next);
	ARM_DEBUG("as_next=%f ae_next=%f", as_next, ae_next);



	/* process angle delta in angle_steps */
	as_diff = as_next - as_cur;
	ae_diff = ae_next - ae_cur;
	ARM_DEBUG("as_diff=%ld ae_diff=%ld", as_diff, ae_diff);


	/* update position status */
	status->h_mm = next_h;
	status->a_deg = next_a;
	status->shoulder_angle_steps = as_next;
	status->elbow_angle_steps = ae_next;
	status->shoulder_angle_rad = as_next_rad;
	status->elbow_angle_rad = ae_next_rad;

	/* we reached destination, nothing to do */
	if (as_diff == 0 && ae_diff == 0) {
		status->shoulder_speed = SHOULDER_DEFAULT_SPEED;
		status->elbow_speed = ELBOW_DEFAULT_SPEED;
		status->next_update_time = 0;
		ARM_NOTICE("reaching end");
		return 0;
	}

	/* test if one actuator is already in position */
	if (as_diff == 0) {
		ARM_DEBUG("shoulder reached destination");
		ae_coef = (double)ARM_MAX_E / (double)ae_diff;
		status->next_update_time = ARM_PERIOD * ABS(ae_coef);
		e_speed = ABS(ae_coef) * ABS(ae_diff);
		s_speed = ARM_MAX_S;
	}
	else if (ae_diff == 0) {
		ARM_DEBUG("elbow reached destination");
		as_coef = (double)ARM_MAX_S / (double)as_diff;
		status->next_update_time = ARM_PERIOD / ABS(as_coef);
		e_speed = ARM_MAX_E;
		s_speed = ABS(as_coef) * ABS(as_diff);
	}
	else {
		as_coef = (double)ARM_MAX_S / (double)as_diff;
		ae_coef = (double)ARM_MAX_E / (double)ae_diff;
	    
		ARM_DEBUG("as_coef=%f ae_coef=%f", as_coef, ae_coef);
	    
		/* if elbow is limitating */
		if (ABS(as_coef) >= ABS(ae_coef)) {
			ARM_DEBUG("elbow limit");
			status->next_update_time = ARM_PERIOD / ABS(ae_coef);
			s_speed = ABS(ae_coef) * ABS(as_diff);
			e_speed = ABS(ae_coef) * ABS(ae_diff);
		}
		/* else, shoulder is limitating */
		else {
			ARM_DEBUG("shoulder limit");
			status->next_update_time = ARM_PERIOD / ABS(as_coef);
			s_speed = ABS(as_coef) * ABS(as_diff);
			e_speed = ABS(as_coef) * ABS(ae_diff);
		}
	}

	ARM_NOTICE("next update: %ld", status->next_update_time);

	/* convert speed in specific unit */
	status->shoulder_speed = (s_speed * CS_PERIOD) / ARM_PERIOD;
	status->elbow_speed = (e_speed * 0x3ff) / ARM_MAX_E;

	ARM_DEBUG("speeds: s=%ld e=%ld", status->shoulder_speed, status->elbow_speed);

	/* avoid limits */
	if (status->shoulder_speed == 0)
		status->shoulder_speed = 1;
	if (status->elbow_speed == 0)
		status->elbow_speed = 1;
	if (status->shoulder_speed >= SHOULDER_MAX_SPEED)
		status->shoulder_speed = SHOULDER_MAX_SPEED;
	if (status->elbow_speed >= ELBOW_MAX_SPEED)
		status->elbow_speed = ELBOW_MAX_SPEED;

	ARM_DEBUG("speeds (sat): s=%ld e=%ld", status->shoulder_speed, status->elbow_speed);

	return 0;
}

static void arm_delete_event(struct arm *arm)
{
	if (arm->status.event == -1)
		return;
	ARM_DEBUG("Delete arm event");
	scheduler_del_event(arm->status.event);
	arm->status.event = -1;
}

/* write values to ax12 + cs */
static void arm_apply(struct arm *arm)
{
	struct cs_block *csb = arm->config.csb;
	const struct arm_status *st = &arm->status;

	ARM_DEBUG("arm_apply");

	if (arm->config.simulate)
		return;

	/* set speed and pos of shoulder */
	quadramp_set_1st_order_vars(&csb->qr, 
				    st->shoulder_speed,
				    st->shoulder_speed);
	cs_set_consign(&csb->cs, st->shoulder_angle_steps);

	/* set speed and position of elbow */
	ax12_user_write_int(&gen.ax12, arm->config.elbow_ax12,
			    AA_MOVING_SPEED_L, st->elbow_speed);
	ax12_user_write_int(&gen.ax12, arm->config.elbow_ax12,
			    AA_GOAL_POSITION_L, st->elbow_angle_steps);
}

/* return true if one of the mask condition is true */
uint8_t arm_test_traj_end(struct arm *arm, uint8_t mask)
{
	if ((mask & ARM_TRAJ_END) && (arm->status.state & ARM_FLAG_IN_WINDOW))
		return ARM_TRAJ_END;

	if ((mask & ARM_TRAJ_NEAR) && (arm->status.state & ARM_FLAG_LAST_STEP))
		return ARM_TRAJ_NEAR;

	if ((mask & ARM_TRAJ_TIMEOUT) && (arm->status.state & ARM_FLAG_TIMEOUT))
		return ARM_TRAJ_TIMEOUT;

	if ((mask & ARM_TRAJ_ERROR) && (arm->status.state & ARM_FLAG_ERROR))
		return ARM_TRAJ_ERROR;

	return 0;
}

uint8_t arm_wait_traj_end(struct arm *arm, uint8_t mask)
{
	uint8_t ret;
	while(1) {
		ret = arm_test_traj_end(arm, mask);
		if (ret)
			return ret;
	}
}

/* return true if one of the mask condition is true */
uint8_t arm_in_window(struct arm *arm, uint8_t *status)
{
	int8_t err;
/* 	uint16_t spd; */
	int16_t pos;
	int32_t cs_err;

	*status = 0;

	if (arm->config.simulate)
		return 1;

	/* shoulder, just check position */
	cs_err = cs_get_error(&arm->config.csb->cs);
	if (ABS(cs_err) > ARM_SHOULDER_WINDOW_POS)
		*status |= SHOULDER_NOT_IN_WIN;

#if 0	
	/* check elbow speed */
	err = ax12_user_read_int(&gen.ax12, arm->config.elbow_ax12,
				 AA_PRESENT_SPEED_L, &spd);
	if (err)
		goto fail;
	if (spd > ARM_ELBOW_WINDOW_SPEED)
		return 0;

	/* check wrist speed */
	err = ax12_user_read_int(&gen.ax12, arm->config.wrist_ax12,
				 AA_PRESENT_SPEED_L, &spd);
	if (err)
		goto fail;
	if (spd > ARM_WRIST_WINDOW_SPEED)
		return 0;
#endif	
	/* check elbow pos */
	err = ax12_user_read_int(&gen.ax12, arm->config.elbow_ax12,
				 AA_PRESENT_POSITION_L, (uint16_t *)&pos);
	if (err)
		goto fail;
	if (ABS(arm->status.elbow_angle_steps - pos) > ARM_ELBOW_WINDOW_POS)
		*status |= ELBOW_NOT_IN_WIN;
	
	if (*status)
		return 0;

	ARM_NOTICE("arm is in window (%ld us after reach pos)",
		   time_get_us2() - arm->status.pos_reached_time);
	return 1; /* ok, we are in window */

 fail:
	return 0;
}

/* event callback */
static void arm_do_xy_cb(struct arm *arm)
{
	uint8_t win_status;

	arm->status.event = -1;

	/* if consign haven't reach destination */
	if ((arm->status.state & ARM_FLAG_LAST_STEP) == 0) {
		if (arm_do_step(arm))
			arm->status.state |= ARM_FLAG_ERROR;

		/* it's the first call for the traj */
		if (arm->status.state == ARM_STATE_INIT) {
			arm->status.state |= ARM_FLAG_MOVING;
		}

		/* we have more steps to do */
		if (arm->status.next_update_time == 0) {
			arm->status.state &= ~ARM_FLAG_MOVING;
			arm->status.state |= ARM_FLAG_LAST_STEP;
			arm->status.pos_reached_time = time_get_us2();
		}
		arm_apply(arm);
	}
	/* last step is reached, we can check that arm is in window */
	else if ((arm->status.state & ARM_FLAG_IN_WINDOW) == 0) {
		if (arm_in_window(arm, &win_status))
			arm->status.state |= ARM_FLAG_IN_WINDOW;
		
		/* check for window arm timeout */
		else {
			microseconds t;
			int32_t diff1; //, diff2;
			t = time_get_us2();
			diff1 = t - arm->status.pos_reached_time; 
			if (diff1 > ARM_WINDOW_TIMEOUT){
				ARM_NOTICE("win timeout at %ld win_status=%x",
					   t, win_status);
				arm->status.state |= ARM_FLAG_TIMEOUT;
			}
		}
	}

	/* check for global arm timeout */
	if ((time_get_us2() - arm->status.start_time) > ARM_GLOBAL_TIMEOUT) {
		ARM_NOTICE("global timeout at %ld", time_get_us2());
		arm->status.state |= ARM_FLAG_TIMEOUT;
	}
	
	/* reload event if needed */
	if ((arm->status.state & ARM_FLAG_FINISHED) == ARM_FLAG_FINISHED) {
		ARM_NOTICE("arm traj finished");
		return; /* no more event, position reached */
	}
	if (arm->status.state & (ARM_FLAG_ERROR|ARM_FLAG_TIMEOUT)) {
		ARM_NOTICE("error or timeout");
		return; /* no more event */
	}
	else if (arm->status.state & ARM_FLAG_LAST_STEP) {
		/* theorical position is reached, but reload an event
		 * for position survey (window), every 25ms */
		arm_schedule_event(arm, ARM_SURVEY_PERIOD);
	}
	else {
		/* reload event for next position step */
		arm_schedule_event(arm, arm->status.next_update_time);
	}
}

/* schedule a single event for this arm */
static void arm_schedule_event(struct arm *arm, uint32_t time)
{
	uint8_t flags;
	int8_t ret;

	arm_delete_event(arm);
	if (time < SCHEDULER_UNIT)
		time = SCHEDULER_UNIT;
	IRQ_LOCK(flags);
	ret = scheduler_add_event(SCHEDULER_SINGLE,
				  (void *)arm_do_xy_cb,
				  arm, time/SCHEDULER_UNIT, ARM_PRIO);
	if (ret == -1) {
		IRQ_UNLOCK(flags);
		ARM_ERROR("Cannot load arm event");
		return;
	}
	arm->status.event = ret;
	IRQ_UNLOCK(flags);
}

int8_t arm_do_xy(struct arm *arm, int16_t a_deg, int16_t h_mm)
{
	ARM_NOTICE("arm_do_xy: a_deg=%d h_mm=%d", a_deg, h_mm);

	/* remove previous event if any */
	arm_delete_event(arm);

	/* init mandatory params */
	arm->req.a_deg = a_deg; 
	arm->req.h_mm = h_mm; 
	arm->status.start_time = time_get_us2();
	arm->status.state = ARM_STATE_INIT;

	/* all the job will be done asynchronously now */
	arm_schedule_event(arm, 0);
	return 0;
}

void arm_dump(struct arm *arm)
{
	printf_P(PSTR("config: simulate=%d\r\n"),
		 arm->config.simulate);
	printf_P(PSTR("req: a_deg=%ld h_mm=%ld \r\n"),
		 arm->req.a_deg, arm->req.h_mm);
	printf_P(PSTR("status: "));
	if (arm->status.state == ARM_STATE_INIT)
		printf_P(PSTR("ARM_STATE_INIT "));
	if (arm->status.state & ARM_FLAG_MOVING)
		printf_P(PSTR("ARM_FLAG_MOVING "));
	if (arm->status.state & ARM_FLAG_LAST_STEP)
		printf_P(PSTR("ARM_FLAG_LAST_STEP "));
	if (arm->status.state & ARM_FLAG_IN_WINDOW)
		printf_P(PSTR("ARM_FLAG_IN_WINDOW "));
	if (arm->status.state & ARM_FLAG_ERROR)
		printf_P(PSTR("ARM_FLAG_ERROR "));
	if (arm->status.state & ARM_FLAG_TIMEOUT)
		printf_P(PSTR("ARM_FLAG_TIMEOUT "));
	printf_P(PSTR("\r\n"));

	printf_P(PSTR("   a_deg=%ld h_mm=%ld \r\n"),
		 arm->status.a_deg, arm->status.h_mm);
	printf_P(PSTR("   cur_shl_steps=%ld cur_elb_steps=%ld\r\n"),
		 arm->status.shoulder_angle_steps, arm->status.elbow_angle_steps);
	printf_P(PSTR("   cur_shl_rad=%f cur_elb_rad=%f\r\n"),
		 arm->status.shoulder_angle_rad, arm->status.elbow_angle_rad);
	printf_P(PSTR("   cur_shl_deg=%f cur_elb_deg=%f\r\n"),
		 DEG(arm->status.shoulder_angle_rad), DEG(arm->status.elbow_angle_rad));
	printf_P(PSTR("   event=%d next_update_time=%ld\r\n"),
		 arm->status.event, arm->status.next_update_time);
	printf_P(PSTR("   start_time=%ld pos_reached_time=%ld\r\n"),
		 arm->status.start_time, arm->status.pos_reached_time);
}

#define CALIB_ANGLE (RAD(45.))

void arm_calibrate(void)
{
	double shoulder, elbow;

	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x1);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x04);

	angle_rad2robot_r(0.0,0.0, &shoulder, &elbow);
	ax12_user_write_int(&gen.ax12, R_ELBOW_AX12, AA_GOAL_POSITION_L, (uint16_t)elbow);
	wait_ms(400);

	dac_mc_set(ARM_DAC, 10000);
	wait_ms(400);
	
	dac_mc_set(ARM_DAC, 6000);
	wait_ms(800);

	printf_P(PSTR("Init arm, please wait..."));
	
	//dac_mc_set(ARM_DAC, -100);
	//wait_ms(3000);

	cs_set_consign(&slavedspic.arm.cs, 0);
	encoders_dspic_set_value(ARM_ENCODER, 300); //490 390
	wait_ms(100);

	dac_mc_set(ARM_DAC, -1000);

	printf_P(PSTR("ok\r\n"));

	//wait_ms(1000);
}

/* init arm config */
void arm_init(void)
{
	int32_t shoulder_robot;
	uint16_t elbow_robot;
	double shoulder_rad, elbow_rad;
	int32_t h, a;
	uint8_t err = 0;

	memset(&right_arm.status, 0, sizeof(right_arm.status));
	right_arm.status.event = -1;

	arm_calibrate();

	/* set des slopes XXX */

	/* set maximum moving speeds */
	err |= ax12_user_write_int(&gen.ax12, R_ELBOW_AX12, AA_MOVING_SPEED_L, 0x1ff);

	/* right arm init */
	shoulder_robot = encoders_dspic_get_value(ARM_ENCODER);
	err |= ax12_user_read_int(&gen.ax12, R_ELBOW_AX12, AA_PRESENT_POSITION_L, &elbow_robot);
	
	angle_robot2rad_r((double)shoulder_robot, (double)elbow_robot,
			  						&shoulder_rad, &elbow_rad);
	angle2cart(shoulder_rad, elbow_rad, &h, &a);
	printf_P(PSTR("right arm: h:%ld a:%ld\r\n"), h, a);
	right_arm.status.h_mm = h;
	right_arm.status.a_deg = a;
	right_arm.status.state = ARM_FLAG_FINISHED;
	right_arm.config.csb = &slavedspic.arm;

	if (err)
		ARM_ERROR("ARM INIT ERROR");
}
