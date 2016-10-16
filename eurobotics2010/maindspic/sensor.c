/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>

//#include <adc.h>
#include <scheduler.h>
#include <pwm_servo.h>
#include <dac_mc.h>

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

#include "main.h"
#include "sensor.h"

///************ ADC */
//
//struct adc_infos {
//	uint16_t config;
//	int16_t value;
//	int16_t prev_val;
//        int16_t (*filter)(struct adc_infos *, int16_t);
//};
//
///* reach 90% of the value in 4 samples */
//int16_t rii_light(struct adc_infos *adc, int16_t val)
//{
//	adc->prev_val = val + (int32_t)adc->prev_val / 2;
//	return adc->prev_val / 2;
//}
//
///* reach 90% of the value in 8 samples */
//int16_t rii_medium(struct adc_infos *adc, int16_t val)
//{
//	adc->prev_val = val + ((int32_t)adc->prev_val * 3) / 4;
//	return adc->prev_val / 4;
//}
//
///* reach 90% of the value in 16 samples */
//int16_t rii_strong(struct adc_infos *adc, int16_t val)
//{
//	adc->prev_val = val + ((int32_t)adc->prev_val * 7) / 8;
//	return adc->prev_val / 8;
//}
//
//
//#define ADC_CONF(x) ( ADC_REF_AVCC | ADC_MODE_INT | MUX_ADC##x )
//
///* define which ADC to poll, see in sensor.h */
//static struct adc_infos adc_infos[ADC_MAX] = { 
//	[ADC_CSENSE1] = { .config = ADC_CONF(0), .filter = rii_medium },
//	[ADC_CSENSE2] = { .config = ADC_CONF(1), .filter = rii_medium },
//	[ADC_CSENSE3] = { .config = ADC_CONF(2), .filter = rii_medium },
//	[ADC_CSENSE4] = { .config = ADC_CONF(3), .filter = rii_medium },
//	[ADC_BATTERY1] = { .config = ADC_CONF(8), .filter = rii_strong },
//	[ADC_BATTERY2] = { .config = ADC_CONF(9), .filter = rii_strong },
//
//	/* add adc on "cap" pins if needed */
///* 	[ADC_CAP1] = { .config = ADC_CONF(10) }, */
///* 	[ADC_CAP2] = { .config = ADC_CONF(11) }, */
///* 	[ADC_CAP3] = { .config = ADC_CONF(12) }, */
///* 	[ADC_CAP4] = { .config = ADC_CONF(13) }, */
//};
//
//static void adc_event(int16_t result);
//
///* called every 10 ms, see init below */
//static void do_adc(void *dummy) 
//{
//	/* launch first conversion */
//	adc_launch(adc_infos[0].config);
//}
//
//static void adc_event(int16_t result)
//{
//	static uint8_t i = 0;
//
//	/* filter value if needed */
//	if (adc_infos[i].filter)
//		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i],
//							 result);
//	else
//		adc_infos[i].value = result;
//
//	i ++;
//	if (i >= ADC_MAX)
//		i = 0;
//	else
//		adc_launch(adc_infos[i].config);
//}
//
//int16_t sensor_get_adc(uint8_t i)
//{
//	int16_t tmp;
//	uint8_t flags;
//
//	IRQ_LOCK(flags);
//	tmp = adc_infos[i].value;
//	IRQ_UNLOCK(flags);
//	return tmp;
//}

/************ boolean sensors */


struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_START_SWITCH] = { 1, 0, 0, 1, 0, 1 }, /* 0 */
	[S_RESERVED0] = { 0, 0, 0, 1, 0, 0 }, /* 1 */
	[S_RESERVED1] = { 0, 0, 0, 1, 0, 0 }, /* 2 */
	[S_RESERVED2] = { 0, 0, 0, 1, 0, 0 }, /* 3 */
	[S_RESERVED3] = { 0, 0, 0, 1, 0, 0 }, /* 4 */
	[S_RESERVED4] = { 0, 0, 0, 1, 0, 0 }, /* 5 */
	[S_RESERVED5] = { 0, 0, 0, 1, 0, 0 }, /* 6 */
	[S_RESERVED6] = { 0, 0, 0, 1, 0, 0 }, /* 7 */
		
	[S_GP0_0] = { 1, 0, 0, 1, 0, 1 }, /* 8 */
	[S_GP0_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_7] = { 1, 0, 0, 1, 0, 1 }, /* 15 */

	[S_GP1_0] = { 1, 0, 0, 1, 0, 1 }, /* 16 */
	[S_GP1_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_7] = { 1, 0, 0, 1, 0, 1 }, /* 23 */

	[S_GP2_0] = { 1, 0, 0, 1, 0, 1 }, /* 24 */
	[S_GP2_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP2_7] = { 1, 0, 0, 1, 0, 1 }, /* 31 */

	[S_GP3_0] = { 1, 0, 0, 1, 0, 0 }, /* 32 */
	[S_GP3_1] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_2] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_3] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_4] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_5] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP3_7] = { 1, 0, 0, 1, 0, 1 }, /* 39 */

};

/* value of filtered sensors */
static uint64_t sensor_filtered = 0;

/* sensor mapping : 
 * 0:  PORTA 9 (START)
 * 1-7: reserved
 * 8-15: 	i2c GP0
 * 16-23: i2c GP1
 * 24-31: i2c GP2
 * 32-39: i2c GP3
 */

uint64_t sensor_get_all(void)
{
	uint64_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
	uint64_t tmp = sensor_get_all();
	return (uint8_t)((tmp & ((uint64_t)1 << i))>>i);
}

//uint64_t gpio0_temp=0, gpio1_temp=0, gpio2_temp=0 , gpio3_temp=0;


/* get the physical value of pins */
static uint64_t sensor_read(void)
{
	uint64_t tmp = 0;

	//gpio0_temp = ((uint64_t)((uint16_t)gen.i2c_gpio0))<< 8;
	//gpio1_temp = ((uint64_t)((uint16_t)gen.i2c_gpio1))<< 16;
	//gpio2_temp = ((uint64_t)((uint16_t)gen.i2c_gpio2))<< 24;
	//gpio3_temp = ((uint64_t)((uint16_t)gen.i2c_gpio3))<< 32;


	tmp |= (uint64_t)((PORTA & (_BV(9))) >> 9) << 0;
	/* 1 to 7 reserved */
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio0))<< 8;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio1))<< 16;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio2))<< 24;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio3))<< 32;

	//tmp |= ((gpio0_temp));
	//tmp |= ((gpio1_temp));
	//tmp |= ((gpio2_temp));
	//tmp |= ((gpio3_temp));
	
	/* add reserved sensors here */
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint64_t sensor = sensor_read();
	uint64_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		
		if(sensor_filter[i].filter == 0)
			continue;
		
		if (((uint64_t)1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}
		
		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}

	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}

/* virtual obstacle */

#define DISABLE_CPT_MAX 500
static uint16_t disable = 0; /* used to disable obstacle detection 
			   * during some time */

/* called every 10 ms */
void
sensor_obstacle_update(void)
{
	if (disable > 0) {
		disable --;
		if (disable == 0)
			DEBUG(E_USER_STRAT, "re-enable sensor");
	}
}

void sensor_obstacle_disable(void)
{
	DEBUG(E_USER_STRAT, "disable sensor");
	disable = DISABLE_CPT_MAX;
}

void sensor_obstacle_enable(void)
{
	disable = 0;
}

uint8_t sensor_obstacle_is_disabled(void)
{
	return disable;
}


/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
	//do_adc(NULL);
	do_boolean_sensors(NULL);
	sensor_obstacle_update();
}

void sensor_init(void)
{
	//adc_init();
	//adc_register_event(adc_event);
	
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, 
						ADC_PRIO);
}

