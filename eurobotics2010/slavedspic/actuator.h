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
 *  Revision : $Id: actuator.h,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */


#define ROLLS_MODE_OUT		0
#define ROLLS_MODE_IN			1
#define ROLLS_MODE_RIGHT	2
#define ROLLS_MODE_LEFT		3

void corn_rolls_set(uint8_t mode, int16_t val);
void pwm_right_corn_mot_set(int16_t value);
void pwm_left_corn_mot_set(int16_t value);


#define CORN_FINGER_RIGHT 			770
#define CORN_FINGER_RIGHT_3TH		630 //595
#define CORN_FINGER_RIGHT_INTER 680
#define CORN_FINGER_RIGHT_LAST 	210 //240
#define CORN_FINGER_RIGHT_OUT 	600

#define CORN_FINGER_LEFT				0
#define CORN_FINGER_LEFT_3TH		140	//162
#define CORN_FINGER_LEFT_INTER	90
#define CORN_FINGER_LEFT_LAST		570 //530
#define CORN_FINGER_LEFT_OUT 		170


#define CORN_FINGER_CENTER 			385

#define BALL_FINGER_RIGHT				708
#define BALL_FINGER_RIGHT_LAST	640
#define BALL_FINGER_LEFT				322
#define BALL_FINGER_LEFT_LAST		380
#define BALL_FINGER_CENTER			506

void ball_finger_goto(uint16_t position);
void corn_finger_goto(uint16_t position);
uint16_t finger_get_goal_pos(uint8_t num);
uint8_t finger_get_side(uint8_t num);


void servo_corn_hold_right_up(void);
void servo_corn_hold_right_down(void);
void servo_corn_hold_left_up(void);
void servo_corn_hold_left_down(void);
void servo_corn_push_out(void);
void servo_corn_push_in(void);

void actuator_init(void);

