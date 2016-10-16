
/*  
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
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
 *  Revision : $Id$
 *
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

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
#include "beacon.h"

void beacon_pull_opponent(void);

int8_t beacon_connected=0;
int16_t link_id = 0;
int16_t error_id = 0;

char line_buff[LINE_BUFF_SIZE];
int8_t cmd_buff[CMD_LINE_SIZE];
uint16_t cmd_size = 0;
int8_t i=0;

/* BEACON WT11 COMMANDS */

void beacon_cmd_wt11_local_reset(void)
{
	/* change to cmd mode */
	wait_ms(1000);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1000);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* reset wt11 */
	uart_send(BEACON_UART,'\r');	
	uart_send(BEACON_UART,'\n');	
	uart_send(BEACON_UART,'R');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,'T');	
	uart_send(BEACON_UART,'\n');	
}

void beacon_cmd_wt11_call(void)
{
	const char send_buf[] = "CALL 00:07:80:85:04:70 1 RFCOMM\n";	
//	char recv_buf[LINE_BUFF_SIZE];	
	int16_t i=0; //, j=0, k=0;
//	microseconds time_us;
//	int16_t c=0;
//	int16_t ret;

//	/* local reset wt11 */
//	beacon_cmd_wt11_local_reset();
	
//	/* flush uart */
//	while((c=uart_recv_nowait(BEACON_UART))!=-1);	
	
	/* send call cmd */
	for(i=0; i<32; i++){
		uart_send(BEACON_UART, send_buf[i]);
	}	
	
//	/* receive response */
//	i=0;
//	time_us = time_get_us2();
//	while((time_get_us2()-time_us)< 10000000){
//		
//		c=uart_recv_nowait(BEACON_UART);
//		
//		if(c == -1)
//			continue;
//		
//		if(i<LINE_BUFF_SIZE){
//			recv_buf[i] = c;
//			i++;	
//			uart_send_nowait(0,recv_buf[i-1]);
//		}	
//		
//	}
//
//	/* parse response */
//	for(k=0, j=0; k<i; k++, j++){
//		
//		line[j] = recv_buf[k];
//					
//		if(line[j] == '\n' || line[j] == '\r'){
//
//			if(j==0){
//				/* skip line */
//				j = -1;			
//				continue;
//			}
//			else{
//				/* add end of line and reset pointer */							
//				line[j] = '\0';
//				j = -1;			
//			}							
//										
//			DEBUG(E_USER_BEACON,"parse: %s\n\r",line);
//			
//			/* connection pass */
//		 	ret = sscanf(line, "CONNECT %d RFCOMM 1", &link_id);
//			if(ret == 1){
//				NOTICE(E_USER_BEACON, "connected %d", link_id);						
//				return 1;	
//			}
//
//			/* connection fails */
//		 	ret = sscanf(line, "NO CARRIER %d ERROR %d RFC_CONNECTION_FAILED", &link_id, &error_id);
//			if(ret == 2){
//				ERROR(E_USER_BEACON, "ERROR %d connecting %d", error_id, link_id);						
//				return -1;	
//			}
//		}
//	}	
//	return -1;
}

void beacon_cmd_wt11_close(void)
{
	/* change to cmd mode */
	wait_ms(1200);
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	uart_send(BEACON_UART,'+');	
	wait_ms(1200);
	
	uart_send(BEACON_UART,'\n');		
	uart_send(BEACON_UART,'\r');		

	/* close conection */
	uart_send(BEACON_UART,'C');	
	uart_send(BEACON_UART,'L');	
	uart_send(BEACON_UART,'O');	
	uart_send(BEACON_UART,'S');	
	uart_send(BEACON_UART,'E');	
	uart_send(BEACON_UART,' ');	
	uart_send(BEACON_UART,'0');		
	uart_send(BEACON_UART,'\n');		
}	


/* SEND AND RECEVE DAEMONS */

void beacon_send_cmd(int8_t *buff, uint16_t size){
	int16_t i;
	
	if(size > CMD_LINE_SIZE){
		ERROR(E_USER_BEACON, "Command size is too large");	
		return;
	}
		
	for(i=0; i<size; i++){
		cmd_buff[i] = buff[i];	
	}		
	cmd_size = size;
}

void beacon_send_daemon(void * dummy)
{
	int16_t i;
	static uint8_t a=0;
		

	if(cmd_size){

		for(i=0; i<cmd_size; i++){
			uart_send(BEACON_UART, cmd_buff[i]);	
		}	
		cmd_size = 0;

		uart_send(BEACON_UART, '\n');
		uart_send(BEACON_UART, '\r');		
	}
	else{

		if(mainboard.flags & DO_OPP){
			a++;
			if (a & 0x4)
				LED3_TOGGLE();

			beacon_pull_opponent();	
		}
	}	
}


void parse_line(char * buff) 
{
	int16_t ret;
	uint8_t flags;
	int16_t arg0, arg1, arg2, arg3;

	DEBUG(E_USER_BEACON,"from beacon: %s",buff);
	
	/* BEACON ANSWERS */

	/* beacon wt11 open link connection pass */
 	ret = sscanf(buff, "CONNECT %d RFCOMM 1", &link_id);
	if(ret == 1){
		beacon_connected = 1;
		NOTICE(E_USER_STRAT, "beacon wt11 link open PASS (%d)", link_id);						
	}

	/* beacon wt11 open link connection fails */
 	ret = sscanf(buff, "NO CARRIER %d ERROR %d RFC_CONNECTION_FAILED", &link_id, &error_id);
	if(ret == 2){
		beacon_connected = 0;
		ERROR(E_USER_STRAT, "beacon wt11 link open FAIL(%d,%d)", error_id, link_id);						
	}
	
	/* beacon wt11 closed conection */
	
	/* beacon wt11 lossed conection */
	
	/* opponent */
 	ret = sscanf(buff, "opponent is %d %d %d %d",
 							 &arg0, &arg1, &arg2, &arg3);
	if(ret == 4){
		IRQ_LOCK(flags);
		beaconboard.opponent_x = arg0;
		beaconboard.opponent_y = arg1;		
		beaconboard.opponent_a = arg2;
		beaconboard.opponent_d = arg3;
		IRQ_UNLOCK(flags);					
	}
	
}


void line_char_in(char c)
{
	
	if(c == '\r' || c == '\n'){
		if(i!=0){			
			line_buff[i] = '\0';
			parse_line(line_buff);
			i=0;
		}
	}
	else{
		line_buff[i++] = c;
		i &= 0x3F;
	}		
}

void beacon_recv_daemon(void)
{
	char c;
	
	c=uart_recv_nowait(BEACON_UART);
		
		if(c != -1)
			line_char_in(c);		
}

void beacon_daemon(void * dummy)
{
	int16_t i;
	static uint8_t a=0;
	char c=0;
	
	/* receive aswers */
	while(c != -1){	
		c=uart_recv_nowait(BEACON_UART);
		if(c != -1)
			line_char_in(c);	
	}
	
	/* pulling and send commands */
	if(cmd_size){

		for(i=0; i<cmd_size; i++){
			uart_send(BEACON_UART, cmd_buff[i]);	
		}	
		cmd_size = 0;

		uart_send(BEACON_UART, '\n');
		uart_send(BEACON_UART, '\r');		
	}
	else{

		if(mainboard.flags & DO_OPP){
			a++;
			if (a & 0x4)
				LED3_TOGGLE();

			beacon_pull_opponent();	
		}
	}	
}


void beacon_init(void)
{
}

/* BEACON COMMANDS */

/* set color */
void beacon_cmd_color(void)
{
	int8_t buff[20];
	uint16_t size;
	
	if(mainboard.our_color == I2C_COLOR_YELLOW)
		size = sprintf((char *)buff,"\n\rcolor yellow");
	else
		size = sprintf((char *)buff,"\n\rcolor blue");
	
	beacon_send_cmd(buff, size);
}


/* get opponent */
void beacon_cmd_opponent(void)
{
	int8_t buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);

	size = sprintf((char *)buff,"opponent %d %d %d",
								robot_x, robot_y, robot_a);

	beacon_send_cmd(buff, size);
}

/* pull opponent position */
void beacon_pull_opponent(void)
{
	int8_t buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);

	size = sprintf((char *)buff,"opponent %d %d %d",
								robot_x, robot_y, robot_a);

	for(i=0; i<size; i++){
		uart_send(BEACON_UART, buff[i]);	
	}	
	
	uart_send(BEACON_UART, '\n');
	uart_send(BEACON_UART, '\r');		

}


/* beacon on */
void beacon_cmd_beacon_on(void)
{
	int8_t buff[] = "\n\rbeacon on";
	uint16_t size = 11;
	
	beacon_send_cmd(buff, size);
}

/* beacon off*/
void beacon_cmd_beacon_off(void)
{
	int8_t buff[] = "\n\rbeacon off";
	uint16_t size = 12;
	
	beacon_send_cmd(buff, size);
}
