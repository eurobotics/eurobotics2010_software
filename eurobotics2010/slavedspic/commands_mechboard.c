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
 *  Revision : $Id: commands_mechboard.c,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <pwm_mc.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "state.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "arm_xy.h"
#include "arm_highlevel.h"
#include "actuator.h"

extern uint16_t state_debug;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_BD | DO_POWER;
		if (!strcmp_P(res->arg2, PSTR("on")))
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & slavedspic.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		if (!strcmp_P(res->arg2, PSTR("on")))
//			arm_calibrate();
		bit = DO_CS;
	}
//	else if (!strcmp_P(res->arg1, PSTR("bd")))
//		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;


	if (!strcmp_P(res->arg2, PSTR("on")))
		slavedspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
			dac_mc_set(ARM_DAC, 0);
		}
		slavedspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & slavedspic.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#bd#power";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("red"))) {
//		mechboard.our_color = I2C_COLOR_RED;
	}
	else if (!strcmp_P(res->color, PSTR("green"))) {
//		mechboard.our_color = I2C_COLOR_GREEN;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "green#red";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
	.f = cmd_color_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_color,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_color_arg0, 
		(prog_void *)&cmd_color_color, 
		NULL,
	},
};

/**********************************************************/
/* arm_show */

/* this structure is filled when cmd_arm_show is parsed successfully */
struct cmd_arm_show_result {
	fixed_string_t arg0;
	fixed_string_t arg2;
};

/* function called when cmd_arm_show is parsed successfully */
static void cmd_arm_show_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	arm_dump(&right_arm);
}

prog_char str_arm_show_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_show_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_show_result, arg0, str_arm_show_arg0);
prog_char str_arm_show_arg2[] = "show";
parse_pgm_token_string_t cmd_arm_show_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_arm_show_result, arg2, str_arm_show_arg2);

prog_char help_arm_show[] = "Show arm status";
parse_pgm_inst_t cmd_arm_show = {
	.f = cmd_arm_show_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_show_arg0,
//		(prog_void *)&cmd_arm_show_arg1,
		(prog_void *)&cmd_arm_show_arg2,
		NULL,
	},
};

/**********************************************************/
/* arm_goto */

/* this structure is filled when cmd_arm_goto is parsed successfully */
struct cmd_arm_goto_result {
	fixed_string_t arg0;
//	fixed_string_t arg1;
	int16_t arg2;
	int16_t arg3;
//	int16_t arg4;
};

/* function called when cmd_arm_goto is parsed successfully */
static void cmd_arm_goto_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_arm_goto_result *res = parsed_result;
	uint8_t err;

		arm_do_xy(&right_arm, res->arg2, res->arg3);//, res->arg4);
		err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
		if (err != ARM_TRAJ_END)
			printf_P(PSTR("err %x\r\n"), err);
}

prog_char str_arm_goto_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_result, arg0, str_arm_goto_arg0);
parse_pgm_token_num_t cmd_arm_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg2, INT16);
parse_pgm_token_num_t cmd_arm_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg3, INT16);
//parse_pgm_token_num_t cmd_arm_goto_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_arm_goto_result, arg4, INT16);

prog_char help_arm_goto[] = "Arm goto a_deg,h_mm";
parse_pgm_inst_t cmd_arm_goto = {
	.f = cmd_arm_goto_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_goto,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_goto_arg0,
//		(prog_void *)&cmd_arm_goto_arg1,
		(prog_void *)&cmd_arm_goto_arg2,
		(prog_void *)&cmd_arm_goto_arg3,
//		(prog_void *)&cmd_arm_goto_arg4,
		NULL,
	},
};

/**********************************************************/
/* arm_goto_fixed */

/* this structure is filled when cmd_arm_goto_fixed is parsed successfully */
struct cmd_arm_goto_fixed_result {
	fixed_string_t arg0;
	//fixed_string_t arg1;
	fixed_string_t arg2;
	//fixed_string_t arg3;
};

/* function called when cmd_arm_goto_fixed is parsed successfully */
static void cmd_arm_goto_fixed_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_arm_goto_fixed_result *res = parsed_result;
	uint8_t err;

	if (strcmp_P(res->arg2, PSTR("hide_inter")) == 0)
		arm_goto_hide_intermediate();
	else if (strcmp_P(res->arg2, PSTR("hide_final")) == 0)
		arm_goto_hide_final();
	else if (strcmp_P(res->arg2, PSTR("prepare_get_ball")) == 0)
		arm_goto_prepare_get_ball();
	else if (strcmp_P(res->arg2, PSTR("get_tomato_inter")) == 0)
		arm_goto_get_tomato_intermediate();
	else if (strcmp_P(res->arg2, PSTR("get_tomato_final")) == 0)
		arm_goto_get_tomato_final();
	else if (strcmp_P(res->arg2, PSTR("put_ball_inter")) == 0)
		arm_goto_put_ball_intermediate();
	else if (strcmp_P(res->arg2, PSTR("put_ball_inter2")) == 0)
		arm_goto_put_ball_intermediate2();
	else if (strcmp_P(res->arg2, PSTR("put_ball_final")) == 0)
		arm_goto_put_ball_final();
	else if (strcmp_P(res->arg2, PSTR("up_tomato")) == 0)
		arm_goto_up_tomato();
	else if (strcmp_P(res->arg2, PSTR("last_ball")) == 0)
		arm_goto_hold_last_ball();
	else if (strcmp_P(res->arg2, PSTR("get_orange_small")) == 0)
		arm_goto_get_orange_small();
	else if (strcmp_P(res->arg2, PSTR("get_orange_medium")) == 0)
		arm_goto_get_orange_medium();
	else if (strcmp_P(res->arg2, PSTR("get_orange_tall")) == 0)
		arm_goto_get_orange_tall();	
	else
		return;

	err = arm_wait_traj_end(&right_arm, ARM_TRAJ_ALL);
	if (err != ARM_TRAJ_END)
		printf_P(PSTR("arm err %x\r\n"), err);

}

prog_char str_arm_goto_fixed_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_goto_fixed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_fixed_result, arg0, str_arm_goto_fixed_arg0);
//prog_char str_arm_goto_fixed_arg1[] = "left#right#both";
//parse_pgm_token_string_t cmd_arm_goto_fixed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_fixed_result, arg1, str_arm_goto_fixed_arg1);
prog_char str_arm_goto_fixed_arg2[] = "hide_inter#hide_final#prepare_get_ball#get_tomato_inter#get_tomato_final#put_ball_inter#put_ball_inter2#put_ball_final#up_tomato#last_ball";
parse_pgm_token_string_t cmd_arm_goto_fixed_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_fixed_result, arg2, str_arm_goto_fixed_arg2);
//prog_char str_arm_goto_fixed_arg3[] = "p1#p2";
//parse_pgm_token_string_t cmd_arm_goto_fixed_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_arm_goto_fixed_result, arg3, str_arm_goto_fixed_arg3);

prog_char help_arm_goto_fixed[] = "Goto fixed positions";
parse_pgm_inst_t cmd_arm_goto_fixed = {
	.f = cmd_arm_goto_fixed_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_goto_fixed,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_goto_fixed_arg0,
		//(prog_void *)&cmd_arm_goto_fixed_arg1,
		(prog_void *)&cmd_arm_goto_fixed_arg2,
		//(prog_void *)&cmd_arm_goto_fixed_arg3,
		NULL,
	},
};

/**********************************************************/
/* arm_simulate */

/* this structure is filled when cmd_arm_simulate is parsed successfully */
struct cmd_arm_simulate_result {
	fixed_string_t arg0;
//	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_arm_simulate is parsed successfully */
static void cmd_arm_simulate_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_arm_simulate_result *res = parsed_result;
	uint8_t val;

	if (strcmp_P(res->arg2, PSTR("simulate")) == 0)
		val = 1;
	else
		val = 0;

//	if (strcmp_P(res->arg1, PSTR("left")) == 0)
//		left_arm.config.simulate = 1;
//	else if (strcmp_P(res->arg1, PSTR("right")) == 0)
		right_arm.config.simulate = 1;
//	else {
//		left_arm.config.simulate = 1;
//		right_arm.config.simulate = 1;
//	}
}

prog_char str_arm_simulate_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_simulate_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_simulate_result, arg0, str_arm_simulate_arg0);
//prog_char str_arm_simulate_arg1[] = "left#right#both";
//parse_pgm_token_string_t cmd_arm_simulate_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_simulate_result, arg1, str_arm_simulate_arg1);
prog_char str_arm_simulate_arg2[] = "simulate#real";
parse_pgm_token_string_t cmd_arm_simulate_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_arm_simulate_result, arg2, str_arm_simulate_arg2);

prog_char help_arm_simulate[] = "Simulation or real for arm";
parse_pgm_inst_t cmd_arm_simulate = {
	.f = cmd_arm_simulate_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm_simulate,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_simulate_arg0,
//		(prog_void *)&cmd_arm_simulate_arg1,
		(prog_void *)&cmd_arm_simulate_arg2,
		NULL,
	},
};

/**********************************************************/
/* corn_rolls tests */

/* this structure is filled when cmd_pwm is parsed successfully */
struct cmd_corn_rolls_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

/* function called when cmd_pwm is parsed successfully */
static void cmd_corn_rolls_parsed(void * parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_corn_rolls_result * res = parsed_result;
	uint8_t mode=0xFF;

	if (!strcmp_P(res->arg1, PSTR("out")))
		mode = ROLLS_MODE_OUT;
	else if (!strcmp_P(res->arg1, PSTR("in")))
		mode = ROLLS_MODE_IN;
	else if (!strcmp_P(res->arg1, PSTR("right")))
		mode = ROLLS_MODE_RIGHT;
	else if (!strcmp_P(res->arg1, PSTR("left")))
		mode = ROLLS_MODE_LEFT;

	if(mode!=0xFF)
		corn_rolls_set(mode, res->arg2);

	printf_P(PSTR("Done\r\n"));
}

prog_char str_corn_rolls_arg0[] = "corn_rolls";
parse_pgm_token_string_t cmd_corn_rolls_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_corn_rolls_result, arg0, str_corn_rolls_arg0);
prog_char str_corn_rolls_arg1[] = "out#in#right#left";
parse_pgm_token_string_t cmd_corn_rolls_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_corn_rolls_result, arg1, str_corn_rolls_arg1);
parse_pgm_token_num_t cmd_corn_rolls_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_corn_rolls_result, arg2, INT16);

prog_char help_corn_rolls[] = "Set corn_rolls mode and value";
parse_pgm_inst_t cmd_corn_rolls = {
	.f = cmd_corn_rolls_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_corn_rolls,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_corn_rolls_arg0, 
		(prog_void *)&cmd_corn_rolls_arg1, 
		(prog_void *)&cmd_corn_rolls_arg2, 
		NULL,
	},
};

/**********************************************************/
/* ball finger */

/* this structure is filled when cmd_finger is parsed successfully */
struct cmd_ball_finger_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_ball_finger is parsed successfully */
static void cmd_ball_finger_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_ball_finger_result *res = parsed_result;
	uint16_t dest = 0;

	if (strcmp_P(res->arg1, PSTR("left")) == 0)
		dest = BALL_FINGER_LEFT;
	else if (strcmp_P(res->arg1, PSTR("left_last")) == 0)
		dest = BALL_FINGER_LEFT_LAST;
	else if (strcmp_P(res->arg1, PSTR("right")) == 0)
		dest = BALL_FINGER_RIGHT;
	else if (strcmp_P(res->arg1, PSTR("right_last")) == 0)
		dest = BALL_FINGER_RIGHT_LAST;
	else if (strcmp_P(res->arg1, PSTR("center")) == 0)
		dest = BALL_FINGER_CENTER;
	ball_finger_goto(dest);
}

prog_char str_ball_finger_arg0[] = "ball_finger";
parse_pgm_token_string_t cmd_ball_finger_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_ball_finger_result, arg0, str_ball_finger_arg0);
prog_char str_ball_finger_arg1[] = "left#left_last#right#right_last#center";
parse_pgm_token_string_t cmd_ball_finger_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_ball_finger_result, arg1, str_ball_finger_arg1);

prog_char help_ball_finger[] = "Move ball_finger";
parse_pgm_inst_t cmd_ball_finger = {
	.f = cmd_ball_finger_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_ball_finger,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_ball_finger_arg0,
		(prog_void *)&cmd_ball_finger_arg1,
		NULL,
	},
};

/**********************************************************/
/* corn finger */

/* this structure is filled when cmd_finger is parsed successfully */
struct cmd_corn_finger_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_corn_finger is parsed successfully */
static void cmd_corn_finger_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_corn_finger_result *res = parsed_result;
	uint16_t dest = 0;

	if (strcmp_P(res->arg1, PSTR("left")) == 0)
		dest = CORN_FINGER_LEFT;
	else if (strcmp_P(res->arg1, PSTR("left_inter")) == 0)
		dest = CORN_FINGER_LEFT_INTER;
	else if (strcmp_P(res->arg1, PSTR("left_last")) == 0)
		dest = CORN_FINGER_LEFT_LAST;
	else if (strcmp_P(res->arg1, PSTR("right")) == 0)
		dest = CORN_FINGER_RIGHT;
	else if (strcmp_P(res->arg1, PSTR("right_inter")) == 0)
		dest = CORN_FINGER_RIGHT_INTER;
	else if (strcmp_P(res->arg1, PSTR("right_last")) == 0)
		dest = CORN_FINGER_RIGHT_LAST;
	else if (strcmp_P(res->arg1, PSTR("right_out")) == 0)
		dest = CORN_FINGER_RIGHT_OUT;
	else if (strcmp_P(res->arg1, PSTR("left_out")) == 0)
		dest = CORN_FINGER_LEFT_OUT;
	else if (strcmp_P(res->arg1, PSTR("right_3th")) == 0)
		dest = CORN_FINGER_RIGHT_OUT;
	else if (strcmp_P(res->arg1, PSTR("left_3th")) == 0)
		dest = CORN_FINGER_LEFT_OUT;


	corn_finger_goto(dest);
}

prog_char str_corn_finger_arg0[] = "corn_finger";
parse_pgm_token_string_t cmd_corn_finger_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_corn_finger_result, arg0, str_corn_finger_arg0);
prog_char str_corn_finger_arg1[] = "left#left_inter#left_last#left_out#left_3th#right#right_inter#right_last#right_out#right_3th";
parse_pgm_token_string_t cmd_corn_finger_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_corn_finger_result, arg1, str_corn_finger_arg1);

prog_char help_corn_finger[] = "Move corn_finger";
parse_pgm_inst_t cmd_corn_finger = {
	.f = cmd_corn_finger_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_corn_finger,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_corn_finger_arg0,
		(prog_void *)&cmd_corn_finger_arg1,
		NULL,
	},
};


/**********************************************************/
/* pump */

/* this structure is filled when cmd_pump is parsed successfully */
struct cmd_pump_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_pump is parsed successfully */
static void cmd_pump_parsed(void *parsed_result, 
			    __attribute__((unused)) void *data)
{
	struct cmd_pump_result *res = parsed_result;
	int16_t val = 0;

	if (strcmp_P(res->arg1, PSTR("on")) == 0)
		val = PUMP_ON;
	else if (strcmp_P(res->arg1, PSTR("off")) == 0)
		val = PUMP_OFF;
	else if (strcmp_P(res->arg1, PSTR("reverse")) == 0)
		val = PUMP_REVERSE;

	pump_set(val);
}

prog_char str_pump_arg0[] = "pump";
parse_pgm_token_string_t cmd_pump_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pump_result, arg0, str_pump_arg0);
prog_char str_pump_arg1[] = "on#off#reverse";
parse_pgm_token_string_t cmd_pump_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pump_result, arg1, str_pump_arg1);

prog_char help_pump[] = "activate pump";
parse_pgm_inst_t cmd_pump = {
	.f = cmd_pump_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pump,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pump_arg0,
		(prog_void *)&cmd_pump_arg1,
		NULL,
	},
};

/**********************************************************/
/* corn_servo */

/* this structure is filled when cmd_servo_corn is parsed successfully */
struct cmd_servo_corn_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_servo_corn is parsed successfully */
static void cmd_servo_corn_parsed(void *parsed_result, 
			    __attribute__((unused)) void *data)
{
	struct cmd_servo_corn_result *res = parsed_result;

	if (strcmp_P(res->arg1, PSTR("hold_r_up")) == 0)
		servo_corn_hold_right_up();
	else if (strcmp_P(res->arg1, PSTR("hold_r_down")) == 0)
		servo_corn_hold_right_down();
	else if (strcmp_P(res->arg1, PSTR("hold_l_up")) == 0)
		servo_corn_hold_left_up();
	else if (strcmp_P(res->arg1, PSTR("hold_l_down")) == 0)
		servo_corn_hold_left_down();
	else if (strcmp_P(res->arg1, PSTR("push_out")) == 0)
		servo_corn_push_out();
	else if (strcmp_P(res->arg1, PSTR("push_in")) == 0)
		servo_corn_push_in();
	else if (strcmp_P(res->arg1, PSTR("push")) == 0)
		corn_push();


	printf("Done\n\r");
}

prog_char str_servo_corn_arg0[] = "corn_servo";
parse_pgm_token_string_t cmd_servo_corn_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_servo_corn_result, arg0, str_servo_corn_arg0);
prog_char str_servo_corn_arg1[] = "hold_r_up#hold_r_down#hold_l_up#hold_l_down#push_out#push_in#push";
parse_pgm_token_string_t cmd_servo_corn_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_servo_corn_result, arg1, str_servo_corn_arg1);

prog_char help_servo_corn[] = "activate servos of corn store system";
parse_pgm_inst_t cmd_servo_corn = {
	.f = cmd_servo_corn_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_servo_corn,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_servo_corn_arg0,
		(prog_void *)&cmd_servo_corn_arg1,
		NULL,
	},
};


/**********************************************************/
/* State1 */

/* this structure is filled when cmd_state1 is parsed successfully */
struct cmd_state1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_state1 is parsed successfully */
static void cmd_state1_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state1_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("init"))) {
		state_init();
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("hide_arm")))
		command.mode = I2C_SLAVEDSPIC_MODE_HIDE_ARM;
	else if (!strcmp_P(res->arg1, PSTR("show_arm")))
		command.mode = I2C_SLAVEDSPIC_MODE_SHOW_ARM;
	else if (!strcmp_P(res->arg1, PSTR("prepare_harvest_ball")))
		command.mode = I2C_SLAVEDSPIC_MODE_PREPARE_HARVEST_BALL;
	else if (!strcmp_P(res->arg1, PSTR("harvest_tomato")))
		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_TOMATO;
	else if (!strcmp_P(res->arg1, PSTR("finger_ball")))
		command.mode = I2C_SLAVEDSPIC_MODE_PUTIN_FINGER_BALL;
	
	state_set_mode(&command);
}

prog_char str_state1_arg0[] = "slavedspic";
parse_pgm_token_string_t cmd_state1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg0, str_state1_arg0);
prog_char str_state1_arg1[] = "init#hide_arm#show_arm#prepare_harvest_ball#harvest_tomato#finger_ball";
parse_pgm_token_string_t cmd_state1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg1, str_state1_arg1);

prog_char help_state1[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state1 = {
	.f = cmd_state1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state1_arg0, 
		(prog_void *)&cmd_state1_arg1, 
		NULL,
	},
};

/**********************************************************/
/* State2 */

/* this structure is filled when cmd_state2 is parsed successfully */
struct cmd_state2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_state2 is parsed successfully */
static void cmd_state2_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state2_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("init"))) {
		state_init();
		return;
	}


	if (!strcmp_P(res->arg1, PSTR("pump_on")))
		command.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_ON;
	else if (!strcmp_P(res->arg1, PSTR("pump_off")))
		command.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_OFF;
	else if (!strcmp_P(res->arg1, PSTR("rolls_in")))
		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_IN;
	else if (!strcmp_P(res->arg1, PSTR("rolls_out")))
		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_OUT;
	else if (!strcmp_P(res->arg1, PSTR("rolls_stop")))
		command.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_STOP;
	else if (!strcmp_P(res->arg1, PSTR("harvest_corn")))
		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_CORN;
	else if (!strcmp_P(res->arg1, PSTR("out_corns")))
		command.mode = I2C_SLAVEDSPIC_MODE_OUT_CORNS;
	
	state_set_mode(&command);
}

prog_char str_state2_arg0[] = "slavedspic";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "pump_on#pump_off#rolls_in#rolls_out#rolls_stop#harvest_corn#out_corns";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);

prog_char help_state2[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state2 = {
	.f = cmd_state2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state2_arg0, 
		(prog_void *)&cmd_state2_arg1, 
		NULL,
	},
};


/**********************************************************/
/* State3 */

/* this structure is filled when cmd_state3 is parsed successfully */
struct cmd_state3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
	int16_t arg3;
};

/* function called when cmd_state3 is parsed successfully */
static void cmd_state3_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state3_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("init"))) {
		state_init();
		return;
	}


	if (!strcmp_P(res->arg1, PSTR("arm_goto_ah"))){
		command.mode = I2C_SLAVEDSPIC_MODE_ARM_GOTO_AH;
		command.arm_goto.angle = res->arg2;
		command.arm_goto.height = res->arg3;
	}
	else if (!strcmp_P(res->arg1, PSTR("harvest_orange"))){
		command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_ORANGE;
		command.harvest_orange.position = CUSTOM_POSITION;
		command.harvest_orange.angle = res->arg2;
		command.harvest_orange.height = res->arg3;
		command.harvest_orange.vacuum_time_div10 = 0;
		
	}
	
	state_set_mode(&command);
}

prog_char str_state3_arg0[] = "slavedspic";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "arm_goto_ah#harvest_orange";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, INT16);
parse_pgm_token_num_t cmd_state3_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg3, INT16);

prog_char help_state3[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state3 = {
	.f = cmd_state3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state3_arg0, 
		(prog_void *)&cmd_state3_arg1, 
		(prog_void *)&cmd_state3_arg2, 
		(prog_void *)&cmd_state3_arg3, 
		NULL,
	},
};

///**********************************************************/
///* State2 */
//
///* this structure is filled when cmd_state2 is parsed successfully */
//struct cmd_state2_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	fixed_string_t arg2;
//};
//
///* function called when cmd_state2 is parsed successfully */
//static void cmd_state2_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state2_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//	uint8_t side;
//
//	if (!strcmp_P(res->arg2, PSTR("left")))
//		side = I2C_LEFT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("right")))
//		side = I2C_RIGHT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("center")))
//		side = I2C_CENTER_SIDE;
//	else
//		side = I2C_AUTO_SIDE;
//
//	if (!strcmp_P(res->arg1, PSTR("prepare_pickup"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//		command.prep_pickup.side = side;
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("push_temple_disc"))) {
//		command.mode = I2C_MECHBOARD_MODE_PUSH_TEMPLE_DISC;
//		command.push_temple_disc.side = side;
//	}
//
//
//	state_set_mode(&command);
//}
//
//prog_char str_state2_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
//prog_char str_state2_arg1[] = "prepare_pickup#push_temple_disc";
//parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
//prog_char str_state2_arg2[] = "left#right#auto#center";
//parse_pgm_token_string_t cmd_state2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg2, str_state2_arg2);
//
//prog_char help_state2[] = "set mechboard mode";
//parse_pgm_inst_t cmd_state2 = {
//	.f = cmd_state2_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state2,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state2_arg0, 
//		(prog_void *)&cmd_state2_arg1, 
//		(prog_void *)&cmd_state2_arg2, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State3 */
//
///* this structure is filled when cmd_state3 is parsed successfully */
//struct cmd_state3_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	uint8_t level;
//};
//
///* function called when cmd_state3 is parsed successfully */
//static void cmd_state3_parsed(void *parsed_result, 
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state3_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("prepare_build"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_BUILD;
//		command.prep_build.level_l = res->level;
//		command.prep_build.level_r = res->level;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("prepare_inside"))) {
//		command.mode = I2C_MECHBOARD_MODE_PREPARE_INSIDE;
//		command.prep_inside.level_l = res->level;
//		command.prep_inside.level_r = res->level;
//	}
//	else if (!strcmp_P(res->arg1, PSTR("autobuild"))) {
//		command.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
//		command.autobuild.level_left = res->level;
//		command.autobuild.level_right = res->level;
//		command.autobuild.count_left = 2;
//		command.autobuild.count_right = 2;
//		command.autobuild.distance_left = I2C_AUTOBUILD_DEFAULT_DIST;
//		command.autobuild.distance_right = I2C_AUTOBUILD_DEFAULT_DIST;
//		command.autobuild.do_lintel = 1;
//	}
//	state_set_mode(&command);
//}
//
//prog_char str_state3_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
//prog_char str_state3_arg1[] = "prepare_build#autobuild#prepare_inside";
//parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
//parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, level, UINT8);
//
//prog_char help_state3[] = "set mechboard mode";
//parse_pgm_inst_t cmd_state3 = {
//	.f = cmd_state3_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state3,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state3_arg0, 
//		(prog_void *)&cmd_state3_arg1, 
//		(prog_void *)&cmd_state3_arg2, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State4 */
//
///* this structure is filled when cmd_state4 is parsed successfully */
//struct cmd_state4_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	uint8_t level_l;
//	uint8_t count_l;
//	uint8_t dist_l;
//	uint8_t level_r;
//	uint8_t count_r;
//	uint8_t dist_r;
//	uint8_t do_lintel;
//};
//
///* function called when cmd_state4 is parsed successfully */
//static void cmd_state4_parsed(void *parsed_result, 
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state4_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//
//	if (!strcmp_P(res->arg1, PSTR("autobuild"))) {
//		command.mode = I2C_MECHBOARD_MODE_AUTOBUILD;
//		command.autobuild.distance_left = res->dist_l;
//		command.autobuild.distance_right = res->dist_r;
//		command.autobuild.level_left = res->level_l;
//		command.autobuild.level_right = res->level_r;
//		command.autobuild.count_left = res->count_l;
//		command.autobuild.count_right = res->count_r;
//		command.autobuild.do_lintel = res->do_lintel;
//	}
//	state_set_mode(&command);
//}
//
//prog_char str_state4_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state4_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state4_result, arg0, str_state4_arg0);
//prog_char str_state4_arg1[] = "autobuild";
//parse_pgm_token_string_t cmd_state4_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state4_result, arg1, str_state4_arg1);
//parse_pgm_token_num_t cmd_state4_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, level_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, count_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, dist_l, UINT8);
//parse_pgm_token_num_t cmd_state4_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, level_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg6 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, count_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg7 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, dist_r, UINT8);
//parse_pgm_token_num_t cmd_state4_arg8 = TOKEN_NUM_INITIALIZER(struct cmd_state4_result, do_lintel, UINT8);
//
//prog_char help_state4[] = "set mechboard mode (autobuild level_l count_l dist_l level_r count_r dist_r lintel)";
//parse_pgm_inst_t cmd_state4 = {
//	.f = cmd_state4_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state4,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state4_arg0, 
//		(prog_void *)&cmd_state4_arg1, 
//		(prog_void *)&cmd_state4_arg2, 
//		(prog_void *)&cmd_state4_arg3, 
//		(prog_void *)&cmd_state4_arg4, 
//		(prog_void *)&cmd_state4_arg5, 
//		(prog_void *)&cmd_state4_arg6, 
//		(prog_void *)&cmd_state4_arg7, 
//		(prog_void *)&cmd_state4_arg8, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State5 */
//
///* this structure is filled when cmd_state5 is parsed successfully */
//struct cmd_state5_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//	fixed_string_t arg2;
//	fixed_string_t arg3;
//};
//
///* function called when cmd_state5 is parsed successfully */
//static void cmd_state5_parsed(void *parsed_result,
//			      __attribute__((unused)) void *data)
//{
//	struct cmd_state5_result *res = parsed_result;
//	struct i2c_cmd_mechboard_set_mode command;
//	uint8_t side;
//
//	if (!strcmp_P(res->arg2, PSTR("left")))
//		side = I2C_LEFT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("right")))
//		side = I2C_RIGHT_SIDE;
//	else if (!strcmp_P(res->arg2, PSTR("center")))
//		side = I2C_CENTER_SIDE;
//	else
//		side = I2C_AUTO_SIDE;
//
//	command.mode = I2C_MECHBOARD_MODE_PREPARE_PICKUP;
//	command.prep_pickup.side = side;
//
//	if (!strcmp_P(res->arg3, PSTR("harvest")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_HARVEST;
//	else if (!strcmp_P(res->arg3, PSTR("lazy_harvest")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_LAZY_HARVEST;
//	else if (!strcmp_P(res->arg3, PSTR("pickup")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_PICKUP;
//	else if (!strcmp_P(res->arg3, PSTR("clear")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_CLEAR;
//	else if (!strcmp_P(res->arg3, PSTR("store")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_STORE;
//	else if (!strcmp_P(res->arg3, PSTR("lazy_pickup")))
//		command.prep_pickup.next_mode = I2C_MECHBOARD_MODE_LAZY_PICKUP;
//
//	state_set_mode(&command);
//}
//
//prog_char str_state5_arg0[] = "mechboard";
//parse_pgm_token_string_t cmd_state5_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg0, str_state5_arg0);
//prog_char str_state5_arg1[] = "prepare_pickup";
//parse_pgm_token_string_t cmd_state5_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg1, str_state5_arg1);
//prog_char str_state5_arg2[] = "left#right#auto#center";
//parse_pgm_token_string_t cmd_state5_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg2, str_state5_arg2);
//prog_char str_state5_arg3[] = "harvest#pickup#store#lazy_harvest#lazy_pickup#clear";
//parse_pgm_token_string_t cmd_state5_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_state5_result, arg3, str_state5_arg3);
//
//prog_char help_state5[] = "set mechboard mode 2";
//parse_pgm_inst_t cmd_state5 = {
//	.f = cmd_state5_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state5,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state5_arg0, 
//		(prog_void *)&cmd_state5_arg1, 
//		(prog_void *)&cmd_state5_arg2, 
//		(prog_void *)&cmd_state5_arg3, 
//		NULL,
//	},
//};

/**********************************************************/
/* State_Machine */

/* this structure is filled when cmd_state_machine is parsed successfully */
struct cmd_state_machine_result {
	fixed_string_t arg0;
};

/* function called when cmd_state_machine is parsed successfully */
static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
				     __attribute__((unused)) void *data)
{
	state_machine();
}

prog_char str_state_machine_arg0[] = "state_machine";
parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);

prog_char help_state_machine[] = "launch state machine";
parse_pgm_inst_t cmd_state_machine = {
	.f = cmd_state_machine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_machine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_machine_arg0, 
		NULL,
	},
};

///**********************************************************/
///* State_Debug */
//
///* this structure is filled when cmd_state_debug is parsed successfully */
//struct cmd_state_debug_result {
//	fixed_string_t arg0;
//	uint8_t on;
//};
//
///* function called when cmd_state_debug is parsed successfully */
//static void cmd_state_debug_parsed(void *parsed_result,
//				   __attribute__((unused)) void *data)
//{
//	struct cmd_state_debug_result *res = parsed_result;
//	state_debug = res->on;
//}
//
//prog_char str_state_debug_arg0[] = "state_debug";
//parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
//parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);
//
//prog_char help_state_debug[] = "Set debug timer for state machine";
//parse_pgm_inst_t cmd_state_debug = {
//	.f = cmd_state_debug_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state_debug,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state_debug_arg0, 
//		(prog_void *)&cmd_state_debug_on, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* Servo_Lintel */
//
///* this structure is filled when cmd_servo_lintel is parsed successfully */
//struct cmd_servo_lintel_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//};
//
///* function called when cmd_servo_lintel is parsed successfully */
//static void cmd_servo_lintel_parsed(void *parsed_result,
//				    __attribute__((unused)) void *data)
//{
//	struct cmd_servo_lintel_result *res = parsed_result;
//	if (!strcmp_P(res->arg1, PSTR("out")))
//		servo_lintel_out();
//	else if (!strcmp_P(res->arg1, PSTR("1lin")))
//		servo_lintel_1lin();
//	else if (!strcmp_P(res->arg1, PSTR("2lin")))
//		servo_lintel_2lin();
//	
//}
//
//prog_char str_servo_lintel_arg0[] = "servo_lintel";
//parse_pgm_token_string_t cmd_servo_lintel_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_servo_lintel_result, arg0, str_servo_lintel_arg0);
//prog_char str_servo_lintel_arg1[] = "out#1lin#2lin";
//parse_pgm_token_string_t cmd_servo_lintel_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_servo_lintel_result, arg1, str_servo_lintel_arg1);
//
//prog_char help_servo_lintel[] = "Servo_Lintel function";
//parse_pgm_inst_t cmd_servo_lintel = {
//	.f = cmd_servo_lintel_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_servo_lintel,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_servo_lintel_arg0, 
//		(prog_void *)&cmd_servo_lintel_arg1, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* Pump_Current */
//
///* this structure is filled when cmd_pump_current is parsed successfully */
//struct cmd_pump_current_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//};
//
///* function called when cmd_pump_current is parsed successfully */
//static void cmd_pump_current_parsed(__attribute__((unused)) void *parsed_result,
//				    __attribute__((unused)) void *data)
//{
//	printf_P(PSTR("l1=%d l2=%d r1=%d r2=%d\r\n"),
//		 mechboard.pump_left1_current, mechboard.pump_left2_current,
//		 sensor_get_adc(ADC_CSENSE3), sensor_get_adc(ADC_CSENSE4));
//}
//
//prog_char str_pump_current_arg0[] = "pump_current";
//parse_pgm_token_string_t cmd_pump_current_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pump_current_result, arg0, str_pump_current_arg0);
//prog_char str_pump_current_arg1[] = "show";
//parse_pgm_token_string_t cmd_pump_current_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pump_current_result, arg1, str_pump_current_arg1);
//
//prog_char help_pump_current[] = "dump pump current";
//parse_pgm_inst_t cmd_pump_current = {
//	.f = cmd_pump_current_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_pump_current,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_pump_current_arg0, 
//		(prog_void *)&cmd_pump_current_arg1, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* Manivelle */
//
///* this structure is filled when cmd_manivelle is parsed successfully */
//struct cmd_manivelle_result {
//	fixed_string_t arg0;
//	uint8_t step;
//};
//
///* function called when cmd_manivelle is parsed successfully */
//static void cmd_manivelle_parsed(__attribute__((unused)) void *parsed_result,
//				 __attribute__((unused)) void *data)
//{
//	struct cmd_manivelle_result *res = parsed_result;
//	state_manivelle(res->step);
//}
//
//prog_char str_manivelle_arg0[] = "manivelle";
//parse_pgm_token_string_t cmd_manivelle_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_manivelle_result, arg0, str_manivelle_arg0);
//parse_pgm_token_num_t cmd_manivelle_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_manivelle_result, step, UINT8);
//
//prog_char help_manivelle[] = "Manivelle function";
//parse_pgm_inst_t cmd_manivelle = {
//	.f = cmd_manivelle_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_manivelle,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_manivelle_arg0, 
//		(prog_void *)&cmd_manivelle_arg1, 
//		NULL,
//	},
//};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0, 
		NULL,
	},
};
