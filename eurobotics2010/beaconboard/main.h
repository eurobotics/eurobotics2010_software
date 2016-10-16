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
 *  Revision : $Id: main.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED1_ON() 	sbi(PORTJ, 2)
#define LED1_OFF() 	cbi(PORTJ, 2)
#define LED1_TOGGLE() 	LED_TOGGLE(PORTJ, 2)

#define LED2_ON() 	sbi(PORTL, 7)
#define LED2_OFF() 	cbi(PORTL, 7)
#define LED2_TOGGLE() 	LED_TOGGLE(PORTL, 7)

#define LED3_ON() 	sbi(PORTJ, 3)
#define LED3_OFF() 	cbi(PORTJ, 3)
#define LED3_TOGGLE() 	LED_TOGGLE(PORTJ, 3)

#define LED4_ON() 	sbi(PORTL, 6)
#define LED4_OFF() 	cbi(PORTL, 6)
#define LED4_TOGGLE() 	LED_TOGGLE(PORTL, 6)
*/

#define BRAKE_ON()      do { Nop(); } while(0)
#define BRAKE_OFF()     do { Nop(); } while(0)

#define BEACON_ENCODER     ((void *)1)
#define BEACON_PWM         ((void *)&gen.pwm_mc_mod2_ch1)

#define BEACON_POS_SENSOR_1  4
#define BEACON_POS_SENSOR_2  3

/** ERROR NUMS */
#define E_USER_I2C_PROTO   195
#define E_USER_SENSOR      196
#define E_USER_BEACON      197

#define LED_PRIO        	 170
#define TIME_PRIO          160
#define ADC_PRIO           120
#define CS_PRIO            100
#define BEACON_PRIO	    		80
#define I2C_POLL_PRIO       20

#define CS_PERIOD 5000L

#define NB_LOGS 4

/* generic to all boards */
struct genboard {
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct pwm_mc pwm_mc_mod2_ch1;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

struct cs_block {
	uint8_t on;
  struct cs cs;
  struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* sensorboard specific */
struct sensorboard {
	
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8

	/* misc flags */
	uint8_t flags;                

	/* control systems */
  struct cs_block beacon;

	/* robot status */
	uint8_t our_color;

};

extern struct genboard gen;
extern struct sensorboard sensorboard;

///* start the bootloader */
//void bootloader(void);


#define wait_cond_or_timeout(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
        __ret;                                                \
})

