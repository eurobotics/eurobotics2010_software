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
 *  Revision : $Id: main.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
//#include <avr/eeprom.h>

#include <aversive.h>
#include <aversive\pgmspace.h>
#include <aversive\wait.h>
#include <aversive\error.h>

#include <configuration_bits_config.h>

#include <uart.h>
//#include <i2c.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>


#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

//#include "../common/eeprom_mapping.h"
#include "../common/i2c_commands.h"

#include "main.h"
#include "cmdline.h"
#include "sensor.h"
//#include "actuator.h"
#include "cs.h"
//#include "i2c_protocol.h"
#include "beacon.h"


struct genboard gen;
struct sensorboard sensorboard;

/***********************/

//void bootloader(void)
//{
//#define BOOTLOADER_ADDR 0x3f000
//	if (pgm_read_byte_far(BOOTLOADER_ADDR) == 0xff) {
//		printf_P(PSTR("Bootloader is not present\r\n"));
//		return;
//	}
//	cli();
//	BRAKE_ON();
//	/* ... very specific :( */
//	TIMSK0 = 0;
//	TIMSK1 = 0;
//	TIMSK2 = 0;
//	TIMSK3 = 0;
//	TIMSK4 = 0;
//	TIMSK5 = 0;
//	EIMSK = 0;
//	UCSR0B = 0;
//	UCSR1B = 0;
//	UCSR2B = 0;
//	UCSR3B = 0;
//	SPCR = 0;
//	TWCR = 0;
//	ACSR = 0;
//	ADCSRA = 0;
//
//	EIND = 1;
//	__asm__ __volatile__ ("ldi r31,0xf8\n");
//	__asm__ __volatile__ ("ldi r30,0x00\n");
//	__asm__ __volatile__ ("eijmp\n");
//	
//	/* never returns */
//}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	cpt++;
	sei();
	//if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}


void timer_init(void)
{
	/* Init Timer1 */
	unsigned int match_value;
	WriteTimer1(0);
	ConfigIntTimer1(T1_INT_ON);
	match_value = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
              T1_PS_1_1 & T1_SYNC_EXT_OFF &
              T1_SOURCE_INT, match_value);
}

/* Timer 1 interrupt handler */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
 	// Interrupt Service Routine code goes here */
  IFS0bits.T1IF=0;
    
  // Register scheduler interrupt
  main_timer_interrupt();
}

void io_pins_init(void)
{
	// mostfet H bridge (outputs)
	_TRISC6 = 0;	// PH1
	_ODCC6 	= 1;	// PH1 is open collector
	_TRISC7 = 0;	// PH2
	_ODCC7 	= 1;	// PH2 is open collector
	
	_LATC6 	= 0;	// motor stoped
	_LATC7	= 0;
	
	// keyence sensors (inputs capture)
	_TRISC4 = 1;	// PZ2_1 (IC1)
	_IC1R		= 20;	// IC1 <- RP20
	_TRISC5 = 1;	// PZ2_2 (IC2)
	_IC2R		= 21;	// IC2 <- RP21
	
	// ee-sx671a sensors (inputs capture)
	_TRISC9 = 1;	// HERR1 (IC7)
	_IC7R		= 25; // IC7 <- RP25
	_TRISC8 = 1;	// HERR2 (IC8)
	_IC8R		= 24; // IC8 <- RP24
	
	// encoder (inputs)
	_QEA1R 	= 10;	// QEA1 <- RP10
	_TRISB10= 1;	// QEA1
	_QEB1R 	= 11;	// QEB1 <- RP11
	_TRISB11= 1;	// QEB1

	// stop bumper (input)
	_TRISB9 = 1;	// BUMPER
	
	// wt11 reset (output)
	_TRISB13= 0;	// RESET_BLUE
	_LATB13	= 0;	// RESET_BLUE OFF
	
	// i2c (in/out open collector??)
	
	// uart
	_U1RXR 	= 8;	// U1RX <- RP8
	_TRISB8 = 1;	// U1RX is input
  _RP7R 	= 3;	// U1TX -> RP7
	_TRISB7	= 0;	// U1TX is output
}

int main(void)
{
	//	//eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_SENSORBOARD);
	//	/* check eeprom to avoid to run the bad program */
	//	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
	//	    EEPROM_MAGIC_SENSORBOARD) {
	//		sei();
	//		printf_P(PSTR("Bad eeprom value\r\n"));
	//		while(1);
	//	}
	
	/* remapeable pins */
	io_pins_init();
	
	/* oscillator */
	oscillator_init();

	memset(&gen, 0, sizeof(gen));
	memset(&sensorboard, 0, sizeof(sensorboard));
	sensorboard.flags = DO_ENCODERS | DO_CS | DO_POWER; // DO_BD
	
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* ENCODERS */
	encoders_dspic_init();

//	/* I2C */
//	i2c_protocol_init();
//	i2c_init(I2C_MODE_SLAVE, I2C_SENSORBOARD_ADDR);
//	i2c_register_recv_event(i2c_recvevent);

	/* TIMER */
	timer_init();
	
	/* PWM */
	pwm_mc_channel_init(&gen.pwm_mc_mod2_ch1,
	                    PWM_MC_MODE_BIPOLAR|PWM_MC_MODE_SIGN_INVERTED, 
	                    2, 1, NULL, 0, NULL, 0);
	pwm_mc_init(&gen.pwm_mc_mod2_ch1, 15000, CH1_COMP&PEN1H&PEN1L);
	pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);		

	/* SCHEDULER */
	scheduler_init();

	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

	/* beacon */
	beacon_init();
	scheduler_add_periodical_event_priority(beacon_calc, NULL, 
						20000L / SCHEDULER_UNIT,BEACON_PRIO);

	sei();


	beacon_calibre_pos();

	//gen.log_level = 5;
	//gen.logs[0] = E_USER_BEACON;

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Dass das Gluck deinen Haus setzt.\r\n"));
	cmdline_interact();

	return 0;
}
