
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive\pgmspace.h>
#include <aversive\wait.h>
#include <aversive\error.h>

#include <uart.h>
#include <i2c.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_mc.h>
#include <encoders_dspic.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <time.h>
#include <quadramp.h>
#include <control_system_manager.h>
//#include <adc.h>

#include <blocking_detection_manager.h>

#include "sensor.h"

#include "../common/i2c_commands.h"
#include "main.h"
#include "beacon.h"

/* some conversions and constants */
#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

/* area */
#define AREA_X 3000
#define AREA_Y 2100

/* convert coords according to our color */
#define COLOR_X(x)     ((sensorboard.our_color==I2C_COLOR_YELLOW)? (x) : (AREA_X-(x)))
#define COLOR_Y(y)     ((sensorboard.our_color==I2C_COLOR_YELLOW)? (y) : (AREA_Y-(y)))
/* fixed beacon coordenates */
#define BEACON_X_OFFSET	(-62)				// beacon calibration includes the offset of 6.2cm
#define BEACON_Y_OFFSET	(AREA_Y/2)
#define BEACON_A_OFFSET	(0)

struct beacon beacon;

//#define BEACON_PWM_VALUE 1000
#define IR_SENSOR_0() (!(_RC4))
#define IR_SENSOR_1() (!(_RC5))
#define MODULO_TIMER (65535L)
#define MULT_ANGLE (1000L)
//#define COEFF_TIMER (2)
//#define COEFF_MULT (100000)
//#define COEFF_MULT2 (1000L)
//#define BEACON_SIZE (9)
#define BEACON_MAX_SAMPLE (3)

#define OPPONENT_POS_X (11)
#define OPPONENT_POS_Y (22)

#define BEACON_DEBUG(args...) DEBUG(E_USER_BEACON, args)
#define BEACON_NOTICE(args...) NOTICE(E_USER_BEACON, args)
#define BEACON_ERROR(args...) ERROR(E_USER_BEACON, args)

static volatile int32_t rising = -1;
static volatile int32_t falling = -1;

static int32_t get_dist(int32_t size, int32_t period);
static int32_t get_angle(int32_t middle, int32_t period, int32_t offset);

static int32_t pos_ref = 0;
static int32_t invalid_count = 0;

//static volatile int32_t beacon_save_count = 0;
//static volatile int32_t beacon_prev_save_count = 0;
//static volatile int32_t count = 0;
//static volatile int32_t count_diff_rising  = 0;
//static volatile int32_t count_diff_falling = 0;
//static int32_t beacon_coeff = 0;

static volatile int32_t beacon_pos;
//static int32_t beacon_sample_size[BEACON_MAX_SAMPLE];

static volatile int32_t beacon_speed;

#define RISING	0
#define FALLING 1
#define N_CAPS	2
static volatile int32_t count_period = 0;
static volatile int32_t count_period_ov = 0;
static volatile int32_t count_edge[N_CAPS][2] = {{0, 0}, {0 ,0}};
static volatile int32_t count_edge_ov[N_CAPS][2] = {{0, 0}, {0 ,0}};
static volatile int8_t valid_pulse[N_CAPS] = {0, 0};
static volatile int8_t valid_period = 0;


int32_t encoders_spi_update_beacon_speed(void * number)
{
	int32_t ret;
	uint8_t flags;

	IRQ_LOCK(flags);
	ret = encoders_dspic_get_value(number);
	beacon_speed = ret - beacon_pos;
	beacon_pos = ret;
//	beacon_prev_save_count = beacon_save_count;
//	beacon_save_count = TMR2;
	IRQ_UNLOCK(flags);
	
//  beacon_coeff = COEFF_TIMER  * COEFF_MULT;
//	beacon_coeff = beacon_speed * COEFF_MULT / ((beacon_prev_save_count - beacon_save_count + MODULO_TIMER + 1)&MODULO_TIMER);

	return beacon_speed;
}


void beacon_init(void)
{
	//int8_t i;

	/* beacon stuff */	
	beacon_reset_pos();
	pos_ref = encoders_dspic_get_value(BEACON_ENCODER);

	memset(&beacon, 0, sizeof(struct beacon));
	beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
			
	beacon_speed = 0;
	
	sensorboard.our_color = I2C_COLOR_YELLOW;

	/*for(i=0;i<BEACON_MAX_SAMPLE;i++)
		beacon_sample_size[i] = 0;*/

	/* initialize input capture 1 and 2 for keyences */
	IC1CONbits.ICM =0b000;		// disable Input Capture module
	IC1CONbits.ICTMR = 1; 		// select Timer2 as the IC time base
	IC1CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC1CONbits.ICM = 0b001; 	// generate capture event on every edge
	
	IC2CONbits.ICM =0b00; 		// disable Input Capture module
	IC2CONbits.ICTMR = 1; 		// select Timer2 as the IC time base
	IC2CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC2CONbits.ICM = 0b001; 	// generate capture event on rising edge


	/* initialize input capture 7 and 8 for ee-xs671a */
	IC7CONbits.ICM =0b000;		// disable Input Capture module
	IC7CONbits.ICTMR = 1; 		// select Timer2 as the IC1 time base
	IC7CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC7CONbits.ICM = 0b011; 	// generate capture event on every rising edge

	IC8CONbits.ICM =0b000;		// disable Input Capture module
	IC8CONbits.ICTMR = 1; 		// select Timer2 as the IC1 time base
	IC8CONbits.ICI = 0b00; 		// interrupt on every capture event
	IC8CONbits.ICM = 0b011; 	// generate capture event on every rising edge

	/* enable capture interrupts and Timer 2 */
	IPC0bits.IC1IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
	IFS0bits.IC1IF = 0; 	// clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; 	// enable IC1 interrupt

	IPC1bits.IC2IP = 5; 	// setup IC2 interrupt priority level XXX, higher than scheduler!
	IFS0bits.IC2IF = 0; 	// clear IC2 Interrupt Status Flag
	IEC0bits.IC2IE = 1; 	// enable IC2 interrupt

	IPC5bits.IC7IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
	IFS1bits.IC7IF = 0; 	// clear IC1 Interrupt Status Flag
	IEC1bits.IC7IE = 1; 	// enable IC1 interrupt

//	IPC5bits.IC8IP = 5; 	// setup IC1 interrupt priority level XXX, higher than scheduler!
//	IFS1bits.IC8IF = 0; 	// clear IC1 Interrupt Status Flag
//	IEC1bits.IC8IE = 1; 	// enable IC1 interrupt

	PR2 = 65535;
	IFS0bits.T2IF = 0; 			// clear T2 Interrupt Status Flag
	T2CONbits.TCKPS = 0b10;	// Timer 2 prescaler = 256, T_timer2 = 1.6us (0b11 for 6.4 us)
	T2CONbits.TON		= 1;		// enable Timer 2

}

void beacon_calibre_pos(void)
{
	sensorboard.flags &= ~DO_CS;

	/* init beacon pos */
	pwm_mc_set(BEACON_PWM, 250);

	/* find rising edge of the mirror*/
//	wait_ms(100);
//	while (sensor_get(BEACON_POS_SENSOR_1));
//	wait_ms(100);
//	while (!sensor_get(BEACON_POS_SENSOR_1));
//
	pwm_mc_set(BEACON_PWM, 0);

	beacon_reset_pos();
	pid_reset(&sensorboard.beacon.pid);
	encoders_dspic_set_value(BEACON_ENCODER, BEACON_OFFSET_CALIBRE);

	cs_set_consign(&sensorboard.beacon.cs, 0);

	sensorboard.flags |= DO_CS;
	
}

void beacon_start(void)
{
	beacon_reset_pos();
	sensorboard.beacon.on = 1;
	cs_set_consign(&sensorboard.beacon.cs, 80/4);
}

void beacon_stop(void)
{
	sensorboard.beacon.on = 0;
	pwm_mc_set(BEACON_PWM, 0);
}

void beacon_reset_pos(void)
{
	pwm_mc_set(BEACON_PWM, 0);
	encoders_dspic_set_value(BEACON_ENCODER, 0);
}


int32_t encoders_spi_get_beacon_speed(void * dummy)
{
	return beacon_speed;
}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
	uint8_t flags;
	
	IFS0bits.IC1IF=0;
	
	/* rising edge */
	if ( IR_SENSOR_0()) {
		IRQ_LOCK(flags);
		count_edge[0][RISING] = (int32_t)IC1BUF;
		count_edge_ov[0][RISING] = _T2IF;
		valid_pulse[0] = 0;
		IRQ_UNLOCK(flags);

	}
	/* falling edge */
	else {
		IRQ_LOCK(flags);
		count_edge[0][FALLING] = (int32_t)IC1BUF;
		count_edge_ov[0][FALLING] = _T2IF;
		valid_pulse[0] = 1;
		IRQ_UNLOCK(flags);
	}
	
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
	uint8_t flags;
	
	IFS0bits.IC2IF=0;

	/* rising edge */
	if ( IR_SENSOR_1()) {
		IRQ_LOCK(flags);
		count_edge[1][RISING] = (int32_t)IC2BUF;
		count_edge_ov[1][RISING] = _T2IF;
		valid_pulse[1] = 0;
		IRQ_UNLOCK(flags);

	}
	/* falling edge */
	else {
		IRQ_LOCK(flags);
		count_edge[1][FALLING] = (int32_t)IC2BUF;
		count_edge_ov[1][FALLING] = _T2IF;
		valid_pulse[1] = 1;
		IRQ_UNLOCK(flags);
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
	uint8_t flags;
	
	IFS1bits.IC7IF=0;
	
	IRQ_LOCK(flags);
	
	TMR2 = 0;
	count_period = (int32_t)IC7BUF;
	count_period_ov = _T2IF;
	_T2IF = 0;	
	
	valid_period = 1;
	IRQ_UNLOCK(flags);
}

//void __attribute__((__interrupt__, no_auto_psv)) _IC8Interrupt(void)
//{
//	uint8_t flags;
//	
//	IFS1bits.IC8IF=0;
//	
//	IRQ_LOCK(flags);
//	TMR2 = 0;
//	count_period[1] = (int32_t)IC8BUF;
//	valid_period[1] = 1;
//	IRQ_UNLOCK(flags);
//
//}


/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

/* return an angle between -180 and 180 */
void abs_xy_to_rel_da(double x_robot, double y_robot, double a_robot, 
											double x_abs, double y_abs,
		      						int32_t *d_rel, int32_t *a_rel_deg)
{
	double a_rel_rad;

	a_rel_rad = atan2(y_abs - y_robot, x_abs - x_robot) - RAD(a_robot);
	if (a_rel_rad < -M_PI) {
		a_rel_rad += M_2PI;
	}
	else if (a_rel_rad > M_PI) {
		a_rel_rad -= M_2PI;
	}
	
	*a_rel_deg = (int32_t)(DEG(a_rel_rad));
	*d_rel = (int32_t)(norm(x_abs-x_robot, y_abs-y_robot));
}


/* return true if the point is in area */
uint8_t is_in_margin(int16_t dx, int16_t dy, int16_t margin)
{
	if ((ABS(dx) < margin) && (ABS(dy) < margin))
		return 1;
	return 0;
}


void sensor_calc(uint8_t num)
{
	static int32_t local_count_period, local_count_period_total=0;
	static int32_t local_count_period_ov;
	static int32_t local_count_edge[2];
	static int32_t local_count_edge_ov[2];
	static int8_t local_valid_pulse;
	static int8_t local_valid_period;
	
	static int32_t count_size, count_size_total=0;
	static int32_t count_middle, count_middle_total=0;
	
	static int32_t local_angle;
	static int32_t local_dist, local_dist_total=0;
	
	int32_t local_robot_x=0;
	int32_t local_robot_y=0;
	int32_t local_robot_a=0;
	
	int32_t result_x=0;
	int32_t result_y=0;
	
	uint8_t flags;
		
	IRQ_LOCK(flags);
	local_count_period = count_period;
	local_count_period_ov = count_period_ov;
	local_count_edge[RISING] = count_edge[num][RISING];
	local_count_edge[FALLING] = count_edge[num][FALLING];
	local_count_edge_ov[RISING] = count_edge_ov[num][RISING];
	local_count_edge_ov[FALLING] = count_edge_ov[num][FALLING];
	local_valid_pulse = valid_pulse[num];
	local_valid_period = valid_period;
	
#if 1
	local_robot_x = beacon.robot_x;
	local_robot_y = beacon.robot_y;
	local_robot_a = beacon.robot_a;
#else
	/* for test other beacon like our */
	local_robot_x = 1500;
	local_robot_y = 1100;
	local_robot_a = 0;
#endif
	
	IRQ_UNLOCK(flags);

	/* filter count_period */
	if(local_valid_period){
		IRQ_LOCK(flags);
		valid_period = 0;
		IRQ_UNLOCK(flags);

		local_count_period += ((MODULO_TIMER + 1)*local_count_period_ov);			
		local_count_period_total = (int32_t)(local_count_period_total*0.8 + local_count_period*0.2);
	}
		
	/* if valid pulse */
	if(local_valid_pulse){
		IRQ_LOCK(flags);
		valid_pulse[num]=0;
		IRQ_UNLOCK(flags);		

		/* add offset to edges counts */
		local_count_edge[RISING] = local_count_edge[RISING] + (MODULO_TIMER + 1)*local_count_edge_ov[RISING];
		local_count_edge[FALLING] = local_count_edge[FALLING] + (MODULO_TIMER + 1)*local_count_edge_ov[FALLING];

		/* calcule pulse size and the middle */	
		if(local_count_edge[RISING]> local_count_edge[FALLING]){
			count_size = local_count_period_total - local_count_edge[RISING] + local_count_edge[FALLING];
			count_middle = (local_count_edge[RISING] + (int32_t)(count_size/2) + local_count_period_total) % local_count_period_total;			
		}
		else{
			count_size = local_count_edge[FALLING] - local_count_edge[RISING];
			count_middle = local_count_edge[RISING] + (int32_t)(count_size/2);
		}
		
//		/* filter pulses by size */
//		if(count_size > 5000){
//			BEACON_DEBUG("count_size_discarted = %ld", count_size);
//			return;
//		}
//		

		if(num==1)
			local_angle = get_angle(count_middle, local_count_period_total, 180);
		else
			local_angle = get_angle(count_middle, local_count_period_total, 0);

		/* discard by angle range */
		if(local_angle > (85*MULT_ANGLE)){
			if(local_angle < (275*MULT_ANGLE)){
				//BEACON_DEBUG("angle_discarted = %ld", local_angle);	
				//return;
				goto error;
			}
		}


		/* filter size and middle counts */
		count_size = ((int32_t)ceil(count_size/10.0))*10;
		count_size_total = (int32_t)(count_size_total*0.8 + count_size*0.2); 
		//count_middle_total = (int32_t)(count_middle_total*0.75 + count_middle*0.25);
		//count_size_total = count_size;
		count_middle_total = count_middle;


		//BEACON_DEBUG(" ");
		//BEACON_DEBUG("count_period = %ld", local_count_period_total);
		//BEACON_DEBUG("count_edge[RISING]  = %ld", local_count_edge[RISING]);
		//BEACON_DEBUG("count_edge[FALLING] = %ld", local_count_edge[FALLING]);
		//BEACON_DEBUG("count_size = %ld", count_size_total);
		//BEACON_DEBUG("count_middle = %ld\r\n", count_middle_total);

		
		//if(num)
		//	local_angle = get_angle(count_middle_total, local_count_period_total, 180);
		//else
		//	local_angle = get_angle(count_middle_total, local_count_period_total, 0);

		
		BEACON_NOTICE("opponent rel beacon angle= %f\t",(double)(local_angle*1.0/MULT_ANGLE));
				
		local_dist = get_dist(count_size_total, local_count_period_total);
		local_dist_total = local_dist*10;
		BEACON_NOTICE("opponent rel beacon dist= %ld\r\n",local_dist_total);

		beacon_angle_dist_to_x_y((int32_t)(local_angle/MULT_ANGLE), local_dist, &result_x, &result_y);

		result_x = COLOR_X(result_x*10 + BEACON_X_OFFSET);
		result_y = COLOR_Y((-result_y*10)+BEACON_Y_OFFSET);
				
		local_angle /= MULT_ANGLE;
		abs_xy_to_rel_da(local_robot_x, local_robot_y, local_robot_a,
										 result_x, result_y, &local_dist, &local_angle);
		
		/* discard if is our robot area */ 
		if(is_in_margin((result_x-local_robot_x), (result_y-local_robot_y), 255)){ // 255 robot_length/2 + error_baliza_max(100mm)
			BEACON_NOTICE("discard xy (%ld %ld) in robot (%ld %ld) margin\r\n",
									 result_x, result_y, local_robot_x, local_robot_y);	
			//return;
			goto error;
		}
		
		/* reset timeout */
		invalid_count = 0;
		
		/* update results */	
		IRQ_LOCK(flags);			
		beacon.opponent_x = result_x;
		beacon.opponent_y = result_y;
		beacon.opponent_angle = local_angle;
		beacon.opponent_dist = local_dist;
		IRQ_UNLOCK(flags);

		BEACON_NOTICE("opponent angle= %ld\t",beacon.opponent_angle);
		BEACON_NOTICE("opponent dist= %ld\r\n",beacon.opponent_dist);
		BEACON_NOTICE("opponent x= %ld\t",beacon.opponent_x);
		BEACON_NOTICE("opponent y= %ld\r\n\n",beacon.opponent_y);

		return;
	}
	else {
		BEACON_NOTICE("non valid pulse\r\n\n");
	}

	error:
		/* 0.5 second timeout */
		if (invalid_count < 25)
			invalid_count++;
		else {
			IRQ_LOCK(flags);
			beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
			IRQ_UNLOCK(flags);
			
			BEACON_NOTICE("opponent not there");
		}	
}

void beacon_calc(void *dummy)
{
	//sensor_calc(0);
	sensor_calc(1);	
}



static int32_t get_dist(int32_t size, int32_t period)
{
	int32_t dist=0;
	double size_rel;
	
	/* calcule relative angle */
	size_rel = size*1.0/period;

	/* dist = offset + (a0 + a1*x + a2*x² + a3x³) */	
	//dist =  (int32_t)((0.0002 + (-0.0041*size_rel) + (0.0507*size_rel*size_rel) + (-1.0814*size_rel*size_rel*size_rel))*10000000);
	//dist += 250;

	/* dist = offset + (a0 + a1*x + a2*x² + a3x³ + a4x^4 + a5x^5) */	
	//dist =  (int32_t)((0.0 + (-0.0002*size_rel) + (0.0083*size_rel*size_rel) + (-0.1468*size_rel*size_rel*size_rel))*1000000000);
	//dist += (int32_t)(((-1.2645*size_rel*size_rel*size_rel*size_rel)+ (-4.2230*size_rel*size_rel*size_rel*size_rel*size_rel))*1000000000);
	//dist += 0;

	/* dist = offset + (a0 + a1*x + a2*x² + a3x³) */	
	dist =  (int32_t)((0.0062 + (-0.1546*size_rel) + (1.1832*size_rel*size_rel) + (-2.4025*size_rel*size_rel*size_rel))*100000);
	dist += 16;       
	return dist;
}

static int32_t get_angle(int32_t middle, int32_t period, int32_t offset)
{
	int32_t ret_angle;
	
	ret_angle = (int32_t)(middle * 360.0 * MULT_ANGLE / period);
	ret_angle = (ret_angle + offset*MULT_ANGLE)%(360*MULT_ANGLE);
	
	return ret_angle;
}

void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y)
{
//	uint8_t flags;

	int32_t local_x = 0;
	int32_t local_y = 0;
	int32_t x_opponent;
	int32_t y_opponent;
	int32_t local_robot_angle = 0;

//	IRQ_LOCK(flags);
//	local_x           = beacon.robot_x;
//	local_y           = beacon.robot_y;
//	local_robot_angle = beacon.robot_angle;
//	IRQ_UNLOCK(flags);

	if (local_robot_angle < 0)
		local_robot_angle += 360;

	x_opponent = cos((local_robot_angle + angle)* 2 * M_PI / 360)* dist;
	y_opponent = sin((local_robot_angle + angle)* 2 * M_PI / 360)* dist;

//	BEACON_DEBUG("x_op= %ld\t",x_opponent);
//	BEACON_DEBUG("y_op= %ld\r\n",y_opponent);
//	BEACON_NOTICE("robot_x= %ld\t",local_x);
//	BEACON_NOTICE("robot_y= %ld\t",local_y);
//	BEACON_NOTICE("robot_angle= %ld\r\n",local_robot_angle);

	*x = local_x + x_opponent;
	*y = local_y + y_opponent;

}


//void beacon_calc(void *dummy)
//{
//	static int32_t local_rising, local_falling;
//	static int32_t middle;
//	static float size = 0;
//	int32_t local_angle;
//	int32_t local_dist;
//
//	int32_t local_count_diff_rising ;
//	int32_t local_count_diff_falling ;
//	int32_t local_beacon_coeff;
//
//	int32_t result_x=0;
//	int32_t result_y=0;
//	int32_t temp=0;
//	int32_t edge=0;
//	//int32_t total_size=0;
//	
//	uint8_t flags;
//	//uint8_t i;
//	int8_t local_valid;
//
//
//	if (falling == -1){
//		/* 0.5 second timeout */
//		if (invalid_count < 25)
//			invalid_count++;
//		else {
//			IRQ_LOCK(flags);
//			beacon.opponent_x = I2C_OPPONENT_NOT_THERE;
//			IRQ_UNLOCK(flags);
//		}	
//		return;
//	}
//
//	invalid_count = 0;
//	IRQ_LOCK(flags);
//	local_valid = valid_beacon;
//	local_count_diff_rising  = count_diff_rising;
//	local_count_diff_falling = count_diff_falling ;
//	local_rising = rising;
//	local_falling = falling;	
//	local_beacon_coeff = beacon_coeff;	
//	IRQ_UNLOCK(flags);
//	
//
//	if (local_valid){
//		invalid_count = 0;
//	
//		BEACON_DEBUG("local_beacon_coeff = %ld", local_beacon_coeff);
//				
//		BEACON_DEBUG("rising= %ld\t",local_rising);
//		BEACON_DEBUG("falling= %ld\r\n",local_falling);
//		BEACON_DEBUG("count diff rising= %ld\t",local_count_diff_rising);
//		BEACON_DEBUG("count diff falling= %ld\r\n",local_count_diff_falling);
//
//		/* recalculate number of pulse by adding the value of the counter, then put value back into motor's round range */
//		
//		local_rising  = ((local_rising + (local_count_diff_rising * local_beacon_coeff) / COEFF_MULT)) %(BEACON_STEP_TOUR);
//		local_falling = ((local_falling + (local_count_diff_falling * local_beacon_coeff) / COEFF_MULT)) %(BEACON_STEP_TOUR);
//
//		BEACON_DEBUG("rising total= %ld\t",local_rising);
//		BEACON_DEBUG("falling total= %ld\r\n",local_falling);
//
//
//		/* if around 360 deg, rising > falling, so invert both and recalculate size and middle */			
//		if(local_falling < local_rising){
//			temp          = local_rising;
//			local_rising  = local_falling;
//			local_falling = temp;
//			size          = BEACON_STEP_TOUR - local_falling + local_rising;
//			middle        = (local_falling + ((int32_t)(size)/2) + BEACON_STEP_TOUR) %(BEACON_STEP_TOUR);
//			edge = local_falling;
//		}
//		/* else rising > falling */
//		else{
//			size   = local_falling - local_rising;		
//			middle = local_rising + (size / 2);
//			edge   = local_rising;
//		}
//
//		//for(i=BEACON_MAX_SAMPLE-1;i>0;i--){
//		//	beacon_sample_size[i] = beacon_sample_size[i-1];		
//		//	total_size += beacon_sample_size[i];
//		//}
//		//beacon_sample_size[0] = size;
//		//total_size += size;
//		//total_size /= BEACON_MAX_SAMPLE;
//
//		//BEACON_DEBUG("rising2= %ld\t",local_rising);
//		//BEACON_DEBUG("falling2= %ld\r\n",local_falling);
//		/* 			BEACON_DEBUG("size= %ld %ld\t",size, total_size); */
//		
//		BEACON_DEBUG("size= %f\r\n",size);
//		BEACON_DEBUG("middle= %ld\r\n",middle);
//
//		local_angle = get_angle(middle,0);
//		//BEACON_NOTICE("opponent angle= %ld\t",local_angle);
//		
//		local_dist = get_dist(size);
//		//BEACON_NOTICE("opponent dist= %ld\r\n",local_dist);
//
//		beacon_angle_dist_to_x_y(local_angle, local_dist, &result_x, &result_y);
//
//		IRQ_LOCK(flags);			
//		beacon.opponent_x = result_x;
//		beacon.opponent_y = result_y;
//		beacon.opponent_angle = local_angle;
//		beacon.opponent_dist = local_dist;
//		/* for I2C test */
//		//beacon.opponent_x = OPPONENT_POS_X;
//		//beacon.opponent_y = OPPONENT_POS_Y;
//		IRQ_UNLOCK(flags);
//
//		//BEACON_NOTICE("opponent x= %ld\t",beacon.opponent_x);
//		//BEACON_NOTICE("opponent y= %ld\r\n\n",beacon.opponent_y);
//	}
//	else {
////		BEACON_NOTICE("non valid\r\n\n");
//	}
//
//	falling = -1;
//}
