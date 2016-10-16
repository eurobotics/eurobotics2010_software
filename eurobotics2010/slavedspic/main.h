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
 *  Revision : $Id: main.h,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED1_ON() 		cbi(LATC, 9)
#define LED1_OFF() 		sbi(LATC, 9)
#define LED1_TOGGLE() LED_TOGGLE(LATC, 9)

/* Brakes on/off: 
 * - arm motor: clear/set RA7.
 * - pickup motors: disable/enable PWM module.
 * - compressor_power: set/- RC8 and RA4 (notice that we only apply brake on).
 */

#define BRAKE_ON()      do{															\
														_LATA7 = 0; 								\
														_LATC8 = 1;	_LATA4 = 1;			\
													} while(0)
#define BRAKE_OFF()			do{															\
														_LATA7 = 1;									\
													} while(0)

#define ARM_ENCODER		((void *)1)
#define ARM_DAC				((void *)&gen.dac_mc_left)

#define COMPRESSOR_POWER	_LATC8
#define VACUUM_REV				_LATA4

#define R_ELBOW_AX12			3
#define BALL_FINGER_AX12 	2
#define CORN_FINGER_AX12 	1

#define R_CORN_MOT	&gen.pwm_mc_mod1_ch2
#define L_CORN_MOT	&gen.pwm_mc_mod2_ch1

#define SERVO_CORN_HOLD_R	&gen.pwm_servo_oc3
#define SERVO_CORN_HOLD_L	&gen.pwm_servo_oc2
#define SERVO_CORN_PUSH		&gen.pwm_servo_oc1


/** ERROR NUMS */
#define E_USER_I2C_PROTO       195
#define E_USER_SENSOR          196
#define E_USER_ARM             197
#define E_USER_FINGER          198
#define E_USER_ST_MACH         199
#define E_USER_CS              200
#define E_USER_AX12            201

#define LED_PRIO           170
#define TIME_PRIO          160
#define ADC_PRIO           120
#define CS_PRIO            100
#define ARM_PRIO            50
#define I2C_POLL_PRIO       20

#define CS_PERIOD 5000L

#define NB_LOGS 4

/* generic to all boards */
struct genboard {
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* dc motors */
	struct pwm_mc pwm_mc_mod1_ch2;
	struct pwm_mc pwm_mc_mod2_ch1;

	/* brushless motors */
	struct dac_mc dac_mc_left;
	
	/* servos */
	struct pwm_servo pwm_servo_oc1;
	struct pwm_servo pwm_servo_oc2;
	struct pwm_servo pwm_servo_oc3;

	
	/* ax12 interface */
	AX12 ax12;

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

/* mechboard specific */
struct slavedspic {
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8

	/* misc flags */
	uint8_t flags;

	/* control systems */
  struct cs_block arm;

	/* robot status */
	volatile uint8_t balls_count;
	volatile uint8_t corns_count;
	volatile uint8_t corns_right_count;
	volatile uint8_t corns_left_count;
	volatile uint8_t status;
};

extern struct genboard gen;
extern struct slavedspic slavedspic;

///* start the bootloader */
//void bootloader(void);

#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
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
