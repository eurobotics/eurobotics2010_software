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
 *  Revision : $Id: commands_sensorboard.c,v 1.2 2009/04/24 19:30:42 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

#include <stdio.h>
#include <string.h>

#include <aversive\pgmspace.h>
#include <aversive\wait.h>
#include <aversive\error.h>

#include <uart.h>
#include <pwm_mc.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cmdline.h"
#include "../common/i2c_commands.h"
//#include "i2c_protocol.h"
#include "beacon.h"

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_BD | DO_POWER;
		if (!strcmp_P(res->arg2, PSTR("on")))
			sensorboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			sensorboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & sensorboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & sensorboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & sensorboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & sensorboard.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		//strat_hardstop();
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;


	if (!strcmp_P(res->arg2, PSTR("on")))
		sensorboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
			pwm_mc_set(BEACON_PWM, 0);
			//pwm_ng_set(BEACON_PWM, 0);
			//pwm_ng_set(SCANNER_PWM, 0);
		}
		sensorboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & sensorboard.flags) ? "on":"off");
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
static void cmd_color_parsed(void *parsed_result, void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		sensorboard.our_color = I2C_COLOR_YELLOW;
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		sensorboard.our_color = I2C_COLOR_BLUE;
	}
	else if (!strcmp_P(res->color, PSTR("show"))) {
		if(sensorboard.our_color == I2C_COLOR_YELLOW)
			printf("color is YELLOW\n\r");
		else
			printf("color is BLUE\n\r");
		
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "yellow#blue#show";
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
/* Beacon */

/* this structure is filled when cmd_beacon is parsed successfully */
struct cmd_beacon_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_beacon is parsed successfully */
static void cmd_beacon_parsed(void *parsed_result, void *data)
{
	
	struct cmd_beacon_result *res = (struct cmd_beacon_result *) parsed_result;
	if (!strcmp_P(res->arg1, PSTR("on"))) {
		beacon_start();
	}
	else if (!strcmp_P(res->arg1, PSTR("off"))) {
		beacon_stop();
	}

	printf_P(PSTR("Done\r\n"));
}

prog_char str_beacon_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg0, str_beacon_arg0);
prog_char str_beacon_arg1[] = "on#off";
parse_pgm_token_string_t cmd_beacon_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg1, str_beacon_arg1);

prog_char help_beacon[] = "Enable/Disable Beacon (on/off)";
parse_pgm_inst_t cmd_beacon = {
	.f = cmd_beacon_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_arg0, 
		(prog_void *)&cmd_beacon_arg1, 
		NULL,
	},
};

/* status */
/* this structure is filled when cmd_beacon is parsed successfully */
struct cmd_beacon_status_result {
	fixed_string_t status;
	
};

/**********************************************************/
/* Opponent */

/* this structure is filled when cmd_opponent is parsed successfully */
struct cmd_opponent_result {
	fixed_string_t arg0;
	int16_t robot_x;
	int16_t robot_y;
	int16_t robot_a;
};

/* function called when cmd_opponent is parsed successfully */
static void cmd_opponent_parsed(void * parsed_result, void *data)
{
	struct cmd_opponent_result *res = parsed_result;
	int16_t opponent_x, opponent_y, opponent_dist, opponent_angle;
	uint8_t flags;
	
	IRQ_LOCK(flags);
	beacon.robot_x = res->robot_x;
	beacon.robot_y = res->robot_y;
	beacon.robot_a = res->robot_a;
	
	opponent_x = (int16_t)(beacon.opponent_x);
	opponent_y = (int16_t)(beacon.opponent_y);
	opponent_angle = (int16_t)(beacon.opponent_angle);
	opponent_dist = (int16_t)(beacon.opponent_dist);
	IRQ_UNLOCK(flags);
	
	printf("opponent is %d %d %d %d\n\r",
	 			opponent_x, opponent_y, opponent_angle, opponent_dist);
	
}

prog_char str_opponent_arg0[] = "opponent";
parse_pgm_token_string_t cmd_opponent_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg0, str_opponent_arg0);
parse_pgm_token_num_t cmd_opponent_robot_x = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, robot_x, INT16);
parse_pgm_token_num_t cmd_opponent_robot_y = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, robot_y, INT16);
parse_pgm_token_num_t cmd_opponent_robot_a = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, robot_a, INT16);

prog_char help_opponent[] = "Get opponent position (x,y,angle,dist)";
parse_pgm_inst_t cmd_opponent = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0, 
		(prog_void *)&cmd_opponent_robot_x, 
		(prog_void *)&cmd_opponent_robot_y, 
		(prog_void *)&cmd_opponent_robot_a, 
		NULL,
	},
};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(void *parsed_result, void *data)
{
	//struct cmd_test_result *res = parsed_result;
	
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
