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
 *  Revision : $Id: i2c_protocol.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <i2c_mem.h>
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

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "i2c_protocol.h"


#define I2C_STATE_MAX 3

#define I2C_TIMEOUT 100 /* ms */
#define I2C_MAX_ERRORS 5

static volatile uint8_t i2c_poll_num = 0;
static volatile uint8_t i2c_state = 0;
static volatile uint16_t i2c_errors = 0;

#define OP_READY 0 /* no i2c op running */
#define OP_POLL  1 /* a user command is running */
#define OP_CMD   2 /* a polling (req / ans) is running */

#define WATCH_DOG_TIMEOUT	10

static volatile uint8_t running_op = OP_READY;

#define I2C_MAX_LOG 1
static uint8_t error_log = 0;

/* used for gpios */
volatile uint8_t gpio_addr = 0;

/* used for commands */
volatile uint16_t command_dest=-1;
volatile uint16_t command_size=0;
uint8_t command_buf[I2C_SEND_BUFFER_SIZE];

static int8_t i2c_read_gpios_01_values(void);
static int8_t i2c_read_gpios_23_values(void);
static int8_t i2c_req_slavedspic_status(void);

#define I2C_ERROR(args...) do {						\
		if (error_log < I2C_MAX_LOG) {				\
			ERROR(E_USER_I2C_PROTO, args);			\
			error_log ++;					\
			if (error_log == I2C_MAX_LOG) {			\
				ERROR(E_USER_I2C_PROTO,			\
				      "i2c logs are now warnings");	\
			}						\
		}							\
		else							\
			WARNING(E_USER_I2C_PROTO, args);		\
	} while(0)



uint8_t dummy = 0;

void i2c_protocol_init(void)
{
}

void i2c_protocol_debug(void)
{
	printf_P(PSTR("I2C protocol debug infos:\r\n"));
	printf_P(PSTR("  i2c_state=%d\r\n"), i2c_state);
	printf_P(PSTR("  i2c_errors=%d\r\n"), i2c_errors);
	printf_P(PSTR("  running_op=%d\r\n"), running_op);
	printf_P(PSTR("  command_size=%d\r\n"), command_size);
	printf_P(PSTR("  command_dest=%d\r\n"), command_dest);
	printf_P(PSTR("  i2c_status=%x\r\n"), i2c_status());
}

static void i2cproto_next_state(uint8_t inc)
{
	i2c_state += inc;
	if (i2c_state >= I2C_STATE_MAX) {
		i2c_state = 0;
		i2c_poll_num ++;
	}
}

void i2cproto_wait_update(void)
{
	uint8_t poll_num;
	poll_num = i2c_poll_num;
	WAIT_COND_OR_TIMEOUT((i2c_poll_num-poll_num) > 1, 150);
}

/* called periodically : the goal of this 'thread' is to send requests
 * and read answers on i2c slaves in the correct order. */
void i2c_poll_slaves(void *dummy)
{
	uint8_t flags;
	int8_t err;
	static uint8_t a = 0;

	void * p_error;
	p_error = &&error1;

	static uint8_t watchdog_cnt = 0;

	/* watchdog */
	watchdog_cnt++;	
	if(watchdog_cnt == WATCH_DOG_TIMEOUT){
		
		if(running_op == OP_CMD)
			I2C_ERROR("I2C wathdog timeout wating COMMAND");
		else{
			I2C_ERROR("I2C wathdog timeout wating POLLING %d", i2c_state);
			i2cproto_next_state(1);
		}	
		running_op = OP_READY;
		i2c_errors = 0;
		i2c_reset();
		return;
	}
		
	/* already running */
	IRQ_LOCK(flags);
	if (running_op != OP_READY) {
		IRQ_UNLOCK(flags);
		return;
	}

	/* reset watchdog */
	watchdog_cnt = 0;
	
	a++;
	if (a & 0x4)
		LED2_TOGGLE();


	/* if a command is ready to be sent, so send it */
	if (command_size) {
		running_op = OP_CMD;
		err = i2c_write(command_dest, I2C_CMD_GENERIC, command_buf, command_size);		

		if (err)
			goto *p_error;
		IRQ_UNLOCK(flags);
		return;
	}

	/* no command, so do the polling */
	running_op = OP_POLL;
	//i2c_state = 2;
	switch(i2c_state) {

		/* poll status of gpios and boards */
		#define I2C_READ_GPIOS_01_VALUES 0
		case I2C_READ_GPIOS_01_VALUES:
			if ((err = i2c_read_gpios_01_values()))
				goto *p_error;
			break;
	
		#define I2C_READ_GPIOS_23_VALUES 1
		case I2C_READ_GPIOS_23_VALUES:
			if ((err = i2c_read_gpios_23_values()))
				goto *p_error;
			break;

		#define I2C_REQ_SLAVEDSPIC 			 2
		case I2C_REQ_SLAVEDSPIC:
			if ((err = i2c_req_slavedspic_status()))
				goto *p_error;
			break;

		/* nothing, go to the first request */
		default:
			i2c_state = 0;
			running_op = OP_READY;
	}
	IRQ_UNLOCK(flags);
	return;

 error1:
	running_op = OP_READY;
	IRQ_UNLOCK(flags);
	i2c_errors++;
	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C send is_cmd=%d proto_state=%d " 
		      "err=%d i2c_status=%x", !!command_size, i2c_state, err, i2c_status());
		i2c_reset();
		i2c_errors = 0;
	}
}

/* called when the xmit is finished */
void i2c_write_event(uint16_t size)
{
	if (size > 0) {
		if (running_op == OP_POLL) {
			i2cproto_next_state(1);
		}
		else
			command_size = 0;
	}
	else {
		i2c_errors++;
		NOTICE(E_USER_I2C_PROTO, "send error state=%d size=%d "
			"op=%d", i2c_state, size, running_op);
		
		//command_size = 0;	
			
		if (i2c_errors > I2C_MAX_ERRORS) {
			I2C_ERROR("I2C error, slave not ready");
			i2c_reset();
			i2c_errors = 0;
		}
		
		if (running_op == OP_POLL) {
			/* skip associated answer */
			i2cproto_next_state(2);
		}
	}
	running_op = OP_READY;
}

/* called read event */
void i2c_read_event(uint8_t * buf, uint16_t size)
{
	volatile uint8_t i2c_state_save = i2c_state;
	void * p_error;
	p_error = &&error2;
	
	if (running_op == OP_POLL)
		i2cproto_next_state(1);

	/* recv is only trigged after a poll */
	running_op = OP_READY;
	
	if ( size == 0) {
		goto *p_error;
	}

	if(i2c_state_save == I2C_READ_GPIOS_01_VALUES ||
		i2c_state_save == I2C_READ_GPIOS_23_VALUES)
	{
		struct i2c_gpios_status * ans = 
			(struct i2c_gpios_status *)buf;
		
		if (size != sizeof (*ans))
			goto *p_error;

		
		if(gpio_addr == I2C_GPIOS_01_ADDR){
			gen.i2c_gpio0 = ans->gpio0;
			gen.i2c_gpio1 = ans->gpio1;
			
		}
		else if(gpio_addr == I2C_GPIOS_23_ADDR){
			gen.i2c_gpio2 = ans->gpio0;
			gen.i2c_gpio3 = ans->gpio1;
		
		}
		
	}

	switch (buf[0]) {
	
		case I2C_ANS_SLAVEDSPIC_STATUS: {
			struct i2c_slavedspic_status * ans = 
				(struct i2c_slavedspic_status *)buf;
			
			if (size != sizeof (*ans))
				goto *p_error;
	
			/* status */
			slavedspic.status = ans->status;
			slavedspic.dummy = ans->dummy;
			slavedspic.balls_count = ans->balls_count;
			slavedspic.corns_count = ans->corns_count;
			
			//printf("slavedspic led = %d\r\n", slavedspic.led);
	
			break;
		}
	

		default:
			break;
	}

	return;
	
 error2:
	i2c_errors++;
	NOTICE(E_USER_I2C_PROTO, "recv error state=%d op=%d", 
	       i2c_state, running_op);
	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C error, slave not ready");
		i2c_reset();
		i2c_errors = 0;
	}
}

static int8_t
i2c_send_command(uint8_t addr, uint8_t *buf, uint8_t size) 
{
	uint8_t flags;
  microseconds us = time_get_us2();

	while ((time_get_us2() - us) < (I2C_TIMEOUT)*1000L) {
		IRQ_LOCK(flags);
		if (command_size == 0) {
			memcpy(command_buf, buf, size);
			command_dest = addr;
			command_size = size;
			
			IRQ_UNLOCK(flags);
			return 0;
		}
		IRQ_UNLOCK(flags);
	}
	/* this should not happen... except if we are called from an
	 * interrupt context, but it's forbidden */
	i2c_write_event(2);
	I2C_ERROR("I2C command send failed");
	return -EBUSY;
}


/* REQUEST PULLING DATA (READ)*/
static int8_t i2c_read_gpios_01_values(void)
{
	struct i2c_gpios_status buf;
	int8_t err;

	gpio_addr = I2C_GPIOS_01_ADDR;
	err = i2c_read(I2C_GPIOS_01_ADDR, I2C_REQ_GPIOS_STATUS,	sizeof(buf));
	
	//printf("gp0 = %d gp1 = %d gp2 = %d gp3 = %d\r\n",
	//gen.i2c_gpio0, gen.i2c_gpio1, gen.i2c_gpio2, gen.i2c_gpio3);
	
	return err;
}

static int8_t i2c_read_gpios_23_values(void)
{
	struct i2c_gpios_status buf;
	int8_t err;

	gpio_addr = I2C_GPIOS_23_ADDR;
	err = i2c_read(I2C_GPIOS_23_ADDR, I2C_REQ_GPIOS_STATUS,	sizeof(buf));
	return err;
}

static int8_t i2c_req_slavedspic_status(void)
{
	struct i2c_slavedspic_status buf;
	int8_t err;

	err = i2c_read(I2C_SLAVEDSPIC_ADDR, I2C_REQ_SLAVEDSPIC_STATUS, sizeof(buf));
	return err;
}


/* COMMANDS (WRITE) */
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state)
{
	struct i2c_cmd_led_control buf;
	buf.hdr.cmd = I2C_CMD_LED_CONTROL;
	buf.led_num = led;
	buf.state = state;
	return i2c_send_command(addr, (uint8_t*)&buf, sizeof(buf));
}


int8_t i2c_slavedspic_mode_exit(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_EXIT;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_wait(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_WAIT;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_init(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_INIT;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_hide_arm(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_HIDE_ARM;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_show_arm(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_SHOW_ARM;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_prepare_harvest_ball(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_PREPARE_HARVEST_BALL;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_harvest_tomato(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_HARVEST_TOMATO;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_putin_finger_ball(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_PUTIN_FINGER_BALL;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_arm_pump_on(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_ON;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_arm_pump_off(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM_PUMP_OFF;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_corn_rolls_in(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_IN;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_corn_rolls_out(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_OUT;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_corn_rolls_stop(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_CORN_ROLLS_STOP;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_harvest_corn(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_HARVEST_CORN;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_out_corns(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_OUT_CORNS;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_harvest_orange(int8_t angle, uint8_t height, uint8_t vacuum_time_div10)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_HARVEST_ORANGE;
	buf.harvest_orange.position = CUSTOM_POSITION;
	buf.harvest_orange.angle = angle;
	buf.harvest_orange.height = height;
	buf.harvest_orange.vacuum_time_div10 = vacuum_time_div10;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_arm_goto_ah(int8_t angle, uint8_t height)
{
	struct i2c_cmd_slavedspic_set_mode buf;
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM_GOTO_AH;
	buf.arm_goto.angle = angle;
	buf.arm_goto.height = height;
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}
