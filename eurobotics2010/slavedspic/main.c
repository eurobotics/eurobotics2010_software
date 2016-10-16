/*  
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *	Based on Microb mechboard code.
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
 *  Revision : $Id: main.c,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
//#include <avr/eeprom.h>

#include <configuration_bits_config.h>


#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <i2c_slave_lite.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>
//#include <adc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>


//#include "../common/eeprom_mapping.h"
#include "../common/i2c_commands.h"

#include "main.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "state.h"
#include "actuator.h"
#include "arm_xy.h"
#include "cs.h"
#include "i2c_protocol.h"

/* 0 means "programmed"
 * ---- with 16 Mhz quartz
 * CKSEL 3-0 : 0111
 * SUT 1-0 : 10 
 * CKDIV8 : 1
 * ---- bootloader
 * BOOTZ 1-0 : 01 (4K bootloader)
 * BOOTRST : 0 (reset on bootloader)
 * ---- jtag
 * jtagen : 0
 */

struct genboard gen;
struct slavedspic slavedspic;

extern uint8_t i2c_watchdog_cnt;

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

void do_led_blink(__attribute__((unused)) void *dummy)
{
#if 1 /* simple blink */
	static uint8_t a=0;

	if(a){
		//LED1_ON();
	}	
	else{
		//LED1_OFF();
	}
	a = !a;
#endif
}

void do_i2c_watchdog(void *dummy)
{
	if(i2c_watchdog_cnt == 0){
		i2c_init(I2C_SLAVEDSPIC_ADDR);
		DEBUG(E_USER_I2C_PROTO,"I2C watchdog triggered, reinit i2c hw");
	}
	
	i2c_watchdog_cnt--;
		
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
	/*************************************** 	* Mainboard robot IO portmap and config 	*/
	
	/* NOTE: after reset all pins are inputs */
	
	/* leds */
	_TRISC9 = 0;	// SLAVE_LED1
	
	/* dc motors */
	_TRISB12 = 0;	// L_CORN_MOT_INA
	_TRISB13 = 0;	// L_CORN_MOT_INB
	_LATB12 = 0;
	_LATB13 = 0;

	_TRISC6 = 0;	// R_CORN_MOT_INA
	_TRISC7 = 0;	// R_CORN_MOT_INB
	_LATC6 = 0;
	_LATC7 = 0;

	
	_TRISC8 = 0;	// COMPRESSOR_POWER
	_ODCC8 = 1;		// is open drain
	_TRISA4 = 0;	// VACUUM_REV
	_ODCA4 = 1; 	// is open drain
	_LATC8 = 1;
	_LATA4 = 1;


	/* brushless motors */
	_TRISA10 = 0; // ARM_MOT_REV	_TRISA7  = 0; // ARM_MOT_BREAK
	_LATA7 = 0;
	
	/* servos  */
	_RP17R = 0b10010; // OC1 -> RP17(RC1) -> CORN_PUSH_PWM
	_RP16R = 0b10011; // OC2 -> RP16(RC0) -> R_CORN_HOLD_PWM
	_RP3R  = 0b10100; // OC3 -> RP3(RB3)  -> L_CORN_HOLD_PWM
		
	/* encoders */
	_QEA1R 	= 21;	// QEA1 <- RP21 <- ARM_ENC_CHA
	_TRISC5 = 1;	
	_QEB1R 	= 20;	// QEB1 <- RP20 <- ARM_ENC_CHB
	_TRISC4	= 1;
		
	/* i2c */
	_ODCB6 = 1;
	_ODCB5 = 1;
	
	
	/* analog inputs */
	
	/* uarts, UART2 is for AX12s */
	_U1RXR 	= 8;	// U1RX <- RP8 <- SLAVE_UART_RX
	_TRISB8 = 1;	// U1RX is input
  _RP7R 	= 3;	// U1TX -> RP7 -> SLAVE_UART_TX
	_TRISB7	= 0;	// U1TX is output
	
	_U2RXR 	= 9;	// U2RX <- RP9 <- SERVOS_UART  _RP9R 	= 5;	// U2TX -> RP9 -> SERVOS_UART	_TRISB9	= 0;	// U2TX is output 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector
}

int main(void)
{
	
//	//eeprom_write_byte(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_MECHBOARD);
//	/* check eeprom to avoid to run the bad program */
//	if (eeprom_read_byte(EEPROM_MAGIC_ADDRESS) !=
//	    EEPROM_MAGIC_MECHBOARD) {
//		sei();
//		printf_P(PSTR("Bad eeprom value\r\n"));
//		while(1);
//	}
//

	/* remapeable pins */
	io_pins_init();
	
	/* brakes */
	BRAKE_ON();

	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();

	/* clear structures */
	memset(&gen, 0, sizeof(gen));
	memset(&slavedspic, 0, sizeof(slavedspic));
	
	/* cs is enabled after arm_calibrate() */
	slavedspic.flags = DO_ENCODERS | DO_POWER | DO_BD;

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

	/* I2C */
	i2c_init(I2C_SLAVEDSPIC_ADDR);
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	i2c_protocol_init();

	/* TIMER */
	timer_init();

	/* PWM_MC */
	pwm_mc_channel_init(&gen.pwm_mc_mod1_ch2,
	                    PWM_MC_MODE_BIPOLAR|PWM_MC_MODE_SIGN_INVERTED, 
	                    1, 2, NULL, 0, NULL, 0);
	pwm_mc_channel_init(&gen.pwm_mc_mod2_ch1,
	                    PWM_MC_MODE_BIPOLAR, 
	                    2, 1, NULL, 0, NULL, 0);
	
//	pwm_mc_init(&gen.pwm_mc_mod1_ch2, 3000, 
//							CH2_COMP&PDIS1H&PDIS1L&PEN2H&PEN2L&PDIS3H&PDIS3L);
//	pwm_mc_init(&gen.pwm_mc_mod2_ch1, 3000, 
//							CH1_COMP&PEN1H&PEN1L);
//	
//	pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);
//	pwm_mc_set(&gen.pwm_mc_mod1_ch2, 0);

	pwm_left_corn_mot_set(0);
	pwm_right_corn_mot_set(0);
	


	/* ADC_MC */
	dac_mc_channel_init(&gen.dac_mc_left, 1, CHANNEL_L,											DAC_MC_MODE_SIGNED,										 	&LATA, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_left, 0);

	/* servos */
	pwm_servo_init(&gen.pwm_servo_oc1, 1, 800, 2400);
	pwm_servo_init(&gen.pwm_servo_oc2, 2, 800, 2400);
	pwm_servo_init(&gen.pwm_servo_oc3, 3, 800, 2400);
	pwm_servo_enable();
	
	pwm_servo_set(&gen.pwm_servo_oc1, 171);
	
	/* SCHEDULER */
	scheduler_init();

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						100000L / SCHEDULER_UNIT, 
						LED_PRIO);

	scheduler_add_periodical_event_priority(do_i2c_watchdog, NULL, 
						8000L / SCHEDULER_UNIT, 
						I2C_POLL_PRIO);


	/* all cs management */
	microb_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* TIME */
	time_init(TIME_PRIO);

	/* ax12 */
	ax12_user_init();

	sei();

	/* fingers + other actuators */
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x1);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x04);

	arm_init();
	slavedspic.flags |= DO_CS;
	arm_do_xy(&right_arm, 0, 100);

	actuator_init();
	state_init();

	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Si no es por no ir ... , pero egke ir p'na es tonteria!!\r\n"));

	/* arm management */
 	gen.logs[0] = E_USER_ST_MACH;
 	//gen.logs[1] = E_USER_I2C_PROTO;
 	//gen.logs[2] = E_USER_ARM;
 	//gen.logs[3] = E_USER_FINGER;
	gen.log_level = 5;
	
	state_machine();
	cmdline_interact();

	return 0;
}
