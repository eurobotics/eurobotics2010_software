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
 *  Revision : $Id: main.c,v 1.9 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
//#include <avr/eeprom.h>

#include <configuration_bits_config.h>

#include <uart.h>
#include <i2c_mem.h>

#include <encoders_dspic.h>
#include <dac_mc.h>
#include <pwm_servo.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>
//#include <adc.h>

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

#include <parse.h>
#include <rdline.h>

//#include "../common/eeprom_mapping.h"
#include "../common/i2c_commands.h"

#include "main.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"
#include "i2c_protocol.h"
#include "beacon.h"


struct genboard gen;
struct mainboard mainboard;
struct slavedspic slavedspic;
struct beaconboard beaconboard;

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

void do_time_monitor(void *dummy)
{
	//uint16_t seconds;
	static uint16_t seconds;
	
	//seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
	seconds ++;
	//eeprom_write_word(EEPROM_TIME_ADDRESS, seconds);
}

void do_led_blink(void *dummy)
{
#if 0	 /* loopback beacon uart */
	char c;
	c = uart_recv(1);
	uart_send(1,c);
#endif
	
#if 1 /* simple blink */
	LED1_TOGGLE();
#endif
}

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
	ConfigIntTimer1(T1_INT_ON);
	WriteTimer1(0);
	match_value = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP &
              T1_PS_1_1 & T1_SYNC_EXT_OFF &
              T1_SOURCE_INT, match_value);
}

/* Timer 1 interrupt handler */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
 	/* Interrupt Service Routine code goes here */
  IFS0bits.T1IF=0;
    
  // Register scheduler interrupt
  main_timer_interrupt();
}

void io_pins_init(void)
{
	
	/* leds */
	_TRISA4 = 0;	// MAIN_LED1
	_TRISA8 = 0;	// MAIN_LED2
	_TRISC2 = 0;	// MAIN_LED3
	_TRISC8 = 0;	// MAIN_LED4
	
	/* brushless motors */
	_TRISA10 = 0; // L_MOT_REV	_TRISA7  = 0; // L_MOT_BREAK
	_LATA7 = 0;
	
	_TRISB10 = 0; // R_MOT_REV	_TRISB11 = 0; // R_MOT_BREAK
	_LATB11 = 0;

	/* servos  */
	_RP22R = 0b10010; // OC1 -> RP22(RC6) -> L_BALLS_LID_PWM
	_RP23R = 0b10011; // OC2 -> RP23(RC7) -> R_BALLS_LID_PWM

	
	// encoders (inputs)	
	_QEA1R 	= 21;	// QEA1 <- RP21(RC5) <- R_ENC_CHA
	_TRISC5 = 1;	
	_QEB1R 	= 20;	// QEB1 <- RP20(RC4) <- R_ENC_CHB
	_TRISC4	= 1;

	_QEA2R 	= 19;	// QEA2 <- RP19(RC3) <- L_ENC_CHA
	_TRISC3 = 1;	
	_QEB2R 	= 4;	// QEB2 <- RP4(RB4)  <- L_ENC_CHB
	_TRISB4	= 1;	


	// analog inputs
			
	// i2c (in/out open collector??)
	_ODCB6 = 1;
	_ODCB5 = 1;

	// current sense (analog input)
	
	// uarts
	_U1RXR 	= 8;	// U1RX <- RP8(RB8) <- MAIN_UART_RX
	_TRISB8 = 1;	// U1RX is input
  _RP7R 	= 3;	// U1TX -> RP7(RB7) -> MAIN_UART_TX
	_TRISB7	= 0;	// U1TX is output
	
	_U2RXR 	= 9;	// U2RX <- RP9(RB9)  <- BEACON_UART_RX
	_TRISB9 = 1;	// U2RX is input  _RP25R 	= 5;	// U2TX -> RP25(RC9) -> BEACON_UART_TX	_TRISC9	= 0;	// U2TX is output
}


#if 0
volatile uint8_t val[32];
static uint8_t i=0;
uint8_t error = 0;
void i2c_test_read_event(uint8_t *rBuff, uint16_t size)
{
		error = 0;
		
		if(size == 1 && rBuff[0]==val[0])
			printf("%d val_rd = %d \n\r", i++, rBuff[0]);
		else{
			printf("%d Error lectura: leido %d\r\n", i++, rBuff[0]);
			error = 1;
			//i2c_reset();
			//wait_ms(100);
		}			
			
		val[0]++;
}

void i2c_test_write_event(uint16_t size)
{
	if(size ==1)
		printf("%d val_wr = %d \n\r", i, val[0]);
	else{
		printf("%d Error escritura: %d bytes escritos\n\r",i, size);
		error = 1;
		//i2c_reset();
		//wait_ms(100);
	}
}

void i2c_test(void)
{
	uint8_t ret;
	
	while(1){
		ret = i2c_write(0x21, 0x02, val, 1);	
		wait_ms(20);
		ret = i2c_read(0x21, 0x02, 1);
		wait_ms(20);
	
	}
}	
#endif


//uint8_t pepe = 0;

int main(void)
{
	uint16_t seconds;
	
//	//eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_MAINBOARD);
//	/* check eeprom to avoid to run the bad program */
//	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
//	    EEPROM_MAGIC_MAINBOARD) {
//		sei();
//		printf_P(PSTR("Bad eeprom value\r\n"));
//		while(1);
//	}

	/* remapeable pins */
	io_pins_init();

	/* brake */
	BRAKE_ON();
	
	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();

	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	mainboard.flags = DO_ENCODERS | DO_RS |
		DO_POS | DO_POWER | DO_BD;
	
	beaconboard.opponent_x = I2C_OPPONENT_NOT_THERE;

	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* Beacon */
	beacon_init();

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* ENCODERS */
	encoders_dspic_init();

	/* I2C */
	i2c_init();
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	//i2c_register_read_event(i2c_test_read_event);
	//i2c_register_write_event(i2c_test_write_event);
	i2c_protocol_init();

	/* TIMER */
	timer_init();

	/* DAC_MC */
	dac_mc_channel_init(&gen.dac_mc_left, 1, CHANNEL_L,											DAC_MC_MODE_SIGNED,										 	&LATA, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_left, 0);

	dac_mc_channel_init(&gen.dac_mc_right, 1, CHANNEL_R,											DAC_MC_MODE_SIGNED|DAC_MC_MODE_SIGN_INVERTED,										 	&LATB, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_right, 0);


	/* servos */
	pwm_servo_init(&gen.pwm_servo_oc1, 1, 800, 2400);
	pwm_servo_init(&gen.pwm_servo_oc2, 2, 800, 2400);
	pwm_servo_enable();


	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						1000000L / SCHEDULER_UNIT, 
						LED_PRIO);

	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

	wait_ms(500);

	/* start i2c slave polling */
	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL,
						8000L / SCHEDULER_UNIT, I2C_POLL_PRIO);

	/* start beacon send cmd polling */
	scheduler_add_periodical_event_priority(beacon_daemon, NULL,
					8000L / SCHEDULER_UNIT, BEACON_POLL_PRIO);

	/* strat */
 	gen.logs[0] = E_USER_STRAT;
 	gen.logs[1] = E_USER_I2C_PROTO;
 	//gen.logs[2] = E_USER_BEACON;
 	//gen.logs[2] = E_OA;
 	gen.log_level = 5;
	strat_reset_infos();

	/* strat-related event */
	scheduler_add_periodical_event_priority(strat_event, NULL,
						25000L / SCHEDULER_UNIT,
						STRAT_PRIO);

	/* eeprom time monitor */
	//scheduler_add_periodical_event_priority(do_time_monitor, NULL,
	//					1000000L / SCHEDULER_UNIT,
	//					EEPROM_TIME_PRIO);

	sei();

	//i2c_test();

#if 0
	while(1){
		wait_ms(200);
		pepe ^=1;
		i2c_led_control(I2C_SLAVEDSPIC_ADDR,1,pepe);
		printf("slavedspic led = %d\r\n", slavedspic.led);
	}
#endif
	
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Que sepas que no pasa na ... , pero que ser, eres!!\r\n"));
//	seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
//	printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
	cmdline_interact();

	return 0;
}










