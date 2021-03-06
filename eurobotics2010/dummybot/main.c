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

//#include <configuration_bits_config.h>

//#include <ax12.h>
#include <uart.h>
//#include <spi.h>
//#include <i2c.h>

//#include <encoders_spi.h>
#include <encoders_dspic.h>

//#include <pwm_ng.h>
#include <pwm_mc.h>

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
//#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

//#include "../common/eeprom_mapping.h"
//#include "../common/i2c_commands.h"

#include "main.h"
//#include "ax12_user.h"
//#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"
//#include "i2c_protocol.h"

//#if __AVR_LIBC_VERSION__ == 10602UL
//#error "won't work with this version"
//#endif

///* 0 means "programmed"
// * ---- with 16 Mhz quartz
// * CKSEL 3-0 : 0111
// * SUT 1-0 : 10 
// * CKDIV8 : 1
// * ---- bootloader
// * BOOTZ 1-0 : 01 (4K bootloader)
// * BOOTRST : 0 (reset on bootloader)
// * ---- jtag
// * jtagen : 0
// */

struct genboard gen;
struct mainboard mainboard;
struct mechboard mechboard;
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

void do_time_monitor(void *dummy)
{
	//uint16_t seconds;
	static uint16_t seconds;
	
	//seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
	seconds ++;
	//eeprom_write_word(EEPROM_TIME_ADDRESS, seconds);
}

//void do_led_blink(void *dummy)
//{
//#if 1 /* simple blink */
//	LED1_TOGGLE();
//#endif
//}

static void main_timer_interrupt(void)
{
	static uint8_t cpt = 0;
	cpt++;
	sei();
	if ((cpt & 0x3) == 0)
		scheduler_interrupt();
}

void timer_init(void)
{
	/* Init Timer1 */
	unsigned int match_value;
	//ConfigIntTimer1(T1_INT_PRIOR_4 & T1_INT_ON);
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
 	// Interrupt Service Routine code goes here */
  IFS0bits.T1IF=0;
    
  // Register scheduler interrupt
  main_timer_interrupt();
}

void io_pins_init(void)
{
	
	/* L6203 H bridges (outputs) */
	_TRISB13 	= 0;	// M_DER_IN1
	_TRISC7  	= 0;	// M_DER_IN2
	_TRISB12 	= 0;	// M_DER_EN
	
	_TRISB15 	= 0;	// M_IZQ_IN1
	_TRISC6  	= 0;	// M_IZQ_IN2
	_TRISB14 	= 0;	// M_IZQ_EN
	
	// encoders (inputs)
	_QEA1R 	= 21;	// QEA1 <- RP21
	_TRISC5 = 1;	// QEA1
	_QEB1R 	= 20;	// QEB1 <- RP20
	_TRISC4	= 1;	// QEB1

	_QEA2R 	= 19;	// QEA2 <- RP19
	_TRISC3 = 1;	// QEA2
	_QEB2R 	= 4;	// QEB2 <- RP4
	_TRISB4	= 1;	// QEB2


	// robot start (input)
	_TRISA9 = 1;	// START
	
	// color
	_TRISA4 = 1;	// COLOR
	
	// wt11 reset (output)
	_TRISA8	= 0;	// RESET_BLUE
	_LATA8	= 0;	// RESET_BLUE OFF
	
	// i2c (in/out open collector??)
	
	// current sense (analog input)
	
	// uarts
	_U1RXR 	= 8;	// U1RX <- RP8
	_TRISB8 = 1;	// U1RX is input
  _RP7R 	= 3;	// U1TX -> RP7
	_TRISB7	= 0;	// U1TX is output
}

int main(void)
{
	uint16_t seconds;
	
	/* oscillator */
	//oscillator_init();
	
	/* remapeable pins */
	io_pins_init();

//	/* brake */
//	BRAKE_DDR();
//	BRAKE_OFF();
//
//	/* CPLD reset on PG3 */
//	DDRG |= 1<<3;
//	PORTG &= ~(1<<3); /* implicit */

//	/* LEDS */
//	DDRJ |= 0x0c;
//	DDRL = 0xc0;
//	LED1_OFF();
//	LED2_OFF();
//	LED3_OFF();
//	LED4_OFF();

	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	mainboard.flags = DO_ENCODERS | DO_RS |
		DO_POS | DO_POWER;// | DO_BD;
	//sensorboard.opponent_x = I2C_OPPONENT_NOT_THERE;

	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);
	
//#if CMDLINE_UART == 3
// 	fdevopen(uart3_dev_send, uart3_dev_recv);
//	uart_register_rx_event(3, emergency);
//#elif CMDLINE_UART == 1
// 	fdevopen(uart1_dev_send, uart1_dev_recv);
//	uart_register_rx_event(1, emergency);
//#else
//#  error not supported
//#endif

//	//eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_MAINBOARD);
//	/* check eeprom to avoid to run the bad program */
//	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
//	    EEPROM_MAGIC_MAINBOARD) {
//		sei();
//		printf_P(PSTR("Bad eeprom value\r\n"));
//		while(1);
//	}

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* SPI + ENCODERS */
	encoders_dspic_init();
//	encoders_spi_init(); /* this will also init spi hardware */

	/* I2C */
//	i2c_init(I2C_MODE_MASTER, I2C_MAINBOARD_ADDR);
//	i2c_protocol_init();
//	i2c_register_recv_event(i2c_recvevent);
//	i2c_register_send_event(i2c_sendevent);
//
	/* TIMER */
	timer_init();
//	timer0_register_OV_intr(main_timer_interrupt);

	/* PWM */
	pwm_mc_channel_init(LEFT_PWM,
	                    PWM_MC_MODE_SIGNED, 
	                    1, 1, &PORTB, 15, &PORTC, 6);
	pwm_mc_channel_init(RIGHT_PWM,
	                    PWM_MC_MODE_SIGNED|PWM_MC_MODE_SIGN_INVERTED, 
	                    1, 2, &PORTB, 13, &PORTC, 7);
                    
	pwm_mc_init(LEFT_PWM, 15000, MOD1_IND&PEN1H&PDIS1L &
															 MOD2_IND&PEN2H&PDIS2L);
	pwm_mc_init(RIGHT_PWM, 15000, MOD1_IND&PEN1H&PDIS1L &
															  MOD2_IND&PEN2H&PDIS2L);																 
	pwm_mc_set(LEFT_PWM, 0);
	pwm_mc_set(RIGHT_PWM, 0);
	
//	PWM_NG_TIMER_16BITS_INIT(1, TIMER_16_MODE_PWM_10, 
//				 TIMER1_PRESCALER_DIV_1);
//	PWM_NG_TIMER_16BITS_INIT(4, TIMER_16_MODE_PWM_10, 
//				 TIMER4_PRESCALER_DIV_1);
//	
//	PWM_NG_INIT16(&gen.pwm1_4A, 4, A, 10, PWM_NG_MODE_SIGNED | 
//		      PWM_NG_MODE_SIGN_INVERTED, &PORTD, 4);
//	PWM_NG_INIT16(&gen.pwm2_4B, 4, B, 10, PWM_NG_MODE_SIGNED | 
//		      PWM_NG_MODE_SIGN_INVERTED, &PORTD, 5);
//	PWM_NG_INIT16(&gen.pwm3_1A, 1, A, 10, PWM_NG_MODE_SIGNED,
//		      &PORTD, 6);
//	PWM_NG_INIT16(&gen.pwm4_1B, 1, B, 10, PWM_NG_MODE_SIGNED,
//		      &PORTD, 7);


	/* servos */
//	PWM_NG_TIMER_16BITS_INIT(3, TIMER_16_MODE_PWM_10, 
//				 TIMER1_PRESCALER_DIV_256);
//	PWM_NG_INIT16(&gen.servo1, 3, C, 10, PWM_NG_MODE_NORMAL,
//		      NULL, 0);
//	PWM_NG_TIMER_16BITS_INIT(5, TIMER_16_MODE_PWM_10, 
//				 TIMER1_PRESCALER_DIV_256);
//	PWM_NG_INIT16(&gen.servo2, 5, A, 10, PWM_NG_MODE_NORMAL,
//		      NULL, 0);
//	PWM_NG_INIT16(&gen.servo3, 5, B, 10, PWM_NG_MODE_NORMAL,
//		      NULL, 0);
//	PWM_NG_INIT16(&gen.servo4, 5, C, 10, PWM_NG_MODE_NORMAL,
//		      NULL, 0);
//	pwm_ng_set(&gen.servo2, 290); /* right */
//	pwm_ng_set(&gen.servo3, 400); /* left */
//	/* 2 lintels 180, 485 */
//	/* 1 lintel 155, 520 */

	/* SCHEDULER */
	scheduler_init();

//	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
//						100000L / SCHEDULER_UNIT, 
//						LED_PRIO);

	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	//sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

//	/* start i2c slave polling */
//	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL,
//						8000L / SCHEDULER_UNIT, I2C_POLL_PRIO);

	/* strat */
 	gen.logs[0] = E_USER_STRAT;
 	gen.log_level = 5;
//	strat_reset_infos();

//	/* strat-related event */
//	scheduler_add_periodical_event_priority(strat_event, NULL,
//						25000L / SCHEDULER_UNIT,
//						STRAT_PRIO);

	/* eeprom time monitor */
	scheduler_add_periodical_event_priority(do_time_monitor, NULL,
						1000000L / SCHEDULER_UNIT,
						EEPROM_TIME_PRIO);

	sei();

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Respect et robustesse.\r\n"));
//	seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
//	printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
	cmdline_interact();

	return 0;
}
