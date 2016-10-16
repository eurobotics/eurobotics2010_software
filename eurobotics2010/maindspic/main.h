/*  
 *  Eurobotics Engineering (2010)
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


#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)


#define LED1_ON() 	cbi(LATA, 4)
#define LED1_OFF() 	sbi(LATA, 4)
#define LED1_TOGGLE() 	LED_TOGGLE(LATA, 4)

#define LED2_ON() 	cbi(LATA, 8)
#define LED2_OFF() 	sbi(LATA, 8)
#define LED2_TOGGLE() 	LED_TOGGLE(LATA, 8)

#define LED3_ON() 	cbi(LATC, 2)
#define LED3_OFF() 	sbi(LATC, 2)
#define LED3_TOGGLE() 	LED_TOGGLE(LATC, 2)

#define LED4_ON() 	cbi(LATC, 8)
#define LED4_OFF() 	sbi(LATC, 8)
#define LED4_TOGGLE() 	LED_TOGGLE(LATC, 8)

#define BRAKE_ON()      do {_LATA7 = 0; _LATB11 = 0;} while(0)
#define BRAKE_OFF()     do {_LATA7 = 1; _LATB11 = 1;} while(0)

/* only 90 seconds, don't forget it :) */
#define MATCH_TIME 89

/* decrease track to decrease angle */
//#define EXT_TRACK_MM 292.0
#define EXT_TRACK_MM 291.0275
//#define EXT_TRACK_MM 302.0188
#define VIRTUAL_TRACK_MM EXT_TRACK_MM

#define ROBOT_LENGTH 308
#define ROBOT_WIDTH 310

/* it is a 3600 imps -> 14400 because we see 1/4 period
 * and diameter: 55mm -> perimeter 173mm 
 * 14400/173 -> 832 imps/10 mm */
/* increase it to go further */
#define IMP_ENCODERS 3600
#define WHEEL_DIAMETER_MM 55.0

///* it is a 2048 imps -> 8192 because we see 1/4 period
// * and diameter: 55mm -> perimeter 173mm 
// * 8192/173 -> 473 */
///* increase it to go further */
//#define IMP_ENCODERS 2048
//#define WHEEL_DIAMETER_MM 55.0

#define WHEEL_PERIM_MM (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 10.
#define DIST_IMP_MM (((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)


#define LEFT_ENCODER        ((void *)2)
#define RIGHT_ENCODER       ((void *)1)

#define LEFT_DAC            ((void *)&gen.dac_mc_left)
#define RIGHT_DAC           ((void *)&gen.dac_mc_right)

/** ERROR NUMS */
#define E_USER_STRAT           194
#define E_USER_I2C_PROTO       195
#define E_USER_SENSOR          196
#define E_USER_CS              197
#define E_USER_BEACON          198

#define LED_PRIO           170
#define TIME_PRIO          160
#define ADC_PRIO           120
#define CS_PRIO            100
#define STRAT_PRIO          50
//#define BEACON_POLL_PRIO    10
#define I2C_POLL_PRIO       20
#define BEACON_POLL_PRIO    10
#define EEPROM_TIME_PRIO     5

#define CS_PERIOD 5000L

#define NB_LOGS 10

/* generic to all boards */
struct genboard{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct dac_mc dac_mc_left;
	struct dac_mc dac_mc_right;

	/* servos */
	struct pwm_servo pwm_servo_oc1;
	struct pwm_servo pwm_servo_oc2;

	/* i2c gpios */
	uint8_t i2c_gpio0;
	uint8_t i2c_gpio1;
	uint8_t i2c_gpio2;
	uint8_t i2c_gpio3;

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

/* mainboard specific */
struct mainboard {

#define DO_ENCODERS  1
#define DO_CS        2
#define DO_RS        4
#define DO_POS       8
#define DO_BD       16
#define DO_TIMER    32
#define DO_POWER    64
#define DO_OPP     128

	/* misc flags */
	uint8_t flags;                

	/* control systems */
  struct cs_block angle;
  struct cs_block distance;

	/* x,y positionning */
	struct robot_system rs;
	struct robot_position pos;
  struct trajectory traj;

	/* robot status */
	uint8_t our_color;
	volatile int16_t speed_a;     /* current angle speed */
	volatile int16_t speed_d;     /* current dist speed */
	int32_t dac_l;                /* current left dac */
	int32_t dac_r;                /* current right dac */
};

/* state of slavedspic, synchronized through i2c */
struct slavedspic {
	uint8_t status;
	uint8_t dummy;
	uint8_t balls_count;
	uint8_t corns_count;
};

/* state of beaconboard, synchronized through i2c */
struct beaconboard {
	
	uint8_t status;
	uint8_t color;
	
	/* opponent pos */
	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;

};

extern struct genboard gen;
extern struct mainboard mainboard;
extern struct slavedspic slavedspic;
extern struct beaconboard beaconboard;

///* start the bootloader */
//void bootloader(void);

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
	if (__ret)					      \
		DEBUG(E_USER_STRAT, "cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})
