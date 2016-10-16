/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2008)
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
 *  Revision : $Id: strat.h,v 1.6 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* convert coords according to our color */
#define COLOR_Y(y)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (y) : (AREA_Y-(y)))
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (AREA_X-(x)))
#define COLOR_A(a)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (-a))
#define COLOR_A2(a)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (180-a))
#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (-x))
#define COLOR_INVERT(x) ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (!x))

/* area */
#define AREA_X 3000
#define AREA_Y 2100

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1050

#define CORNER_X 3000
#define CORNER_Y COLOR_Y(2100)

/* grid */
#define X(num)	(150 + 225*num)
#define Y(num)	(2100-(128 + 125*(11-num)))

/* useful traj flags */
#define TRAJ_SUCCESS(f) (f & (END_TRAJ|END_NEAR))
#define TRAJ_FLAGS_STD (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER (END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST (END_TRAJ|END_BLOCKING|END_INTR)

/* default speeds */
#define SPEED_DIST_FAST 2000
#define SPEED_ANGLE_FAST 2000
#define SPEED_DIST_SLOW 1500
#define SPEED_ANGLE_SLOW 1500
#define SPEED_DIST_VERY_SLOW 1000
#define SPEED_ANGLE_VERY_SLOW 1000

/* corn detection */
#define CORN_LEFT_X				 230
#define CORN_LEFT_Y				 240
#define CORN_RIGHT_X			 230
#define CORN_RIGHT_Y			-240
#define CORN_MATCH_MARGIN	  80




/* strat infos structures */
struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

struct conf {
	
#define STRAT_CONF_HARVEST_TOMATOES		 		0x01
#define STRAT_CONF_HARVEST_STATIC_CORNS	 	0x02
#define STRAT_CONF_HARVEST_FALL_CORNS 		0x04
	uint8_t flags;
};

struct corn {
	 int16_t x;
   int16_t y;
};

struct basket {
	int16_t checkpoint_x;
	int16_t checkpoint_y;
};

struct orange {
	int8_t angle;
	int8_t arm_angle;
	int8_t arm_heigh;	
};

struct rampe {
	int16_t checkpoint_x;
	int16_t checkpoint_y;	
};

/* all infos related to strat */
struct strat_infos {
	uint8_t dump_enabled;
	uint8_t balls_count;
	uint8_t corns_count;
	uint8_t opp_was_on_rampe;
	uint8_t tomato_event;
	uint8_t static_corn_event;
	uint8_t fall_corn_event;
	uint8_t corn_detect_num;
	uint8_t event_elements_enable;
	struct conf conf;
	struct bbox area_bbox;
	struct corn corn[18];
	struct basket basket;
	struct orange orange[6];
	struct rampe rampe;
};
extern struct strat_infos strat_infos;

/* in strat.c */
void strat_dump_infos(const char *caller); /* show current known state
					      of area */
void strat_dump_conf(void);
void strat_reset_infos(void); /* reset current known state */

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

void strat_dump_flags(void);

uint8_t strat_main(void);
void strat_event(void *dummy);

/* in strat_harvesters.c */
#define OFF				0
#define ON				1
#define LEFT_ON		2
#define RIGHT_ON	3
void strat_event_tomato(void);
void strat_enable_harvest_tomatoes(void);
void strat_disable_harvest_tomatoes(void);
void strat_event_static_corn(void);
void strat_enable_harvest_static_corns(void);
void strat_disable_harvest_static_corns(void);
void strat_event_fall_corn(void);
void strat_enable_harvest_fall_corns(void);
void strat_disable_harvest_fall_corns(void);
void strat_manage_event_tomato(void);
void strat_manage_event_static_corn(void);
void strat_manage_event_fall_corn(void);

/* in strat_subtraj */
uint8_t strat_goto_and_avoid_harvesting(int16_t x, int16_t y, uint8_t harvest_flags);
uint8_t strat_harvest_oranges(void);
uint8_t strat_harvest_corn(uint8_t num);
uint8_t strat_goto_basket(void);
uint8_t strat_goto_diagonal(void);

///* in strat_static_columns.c */
//uint8_t strat_static_columns(uint8_t configuration);
//uint8_t strat_static_columns_pass2(void);
//
///* in strat_lintel.c */
//uint8_t strat_goto_lintel_disp(struct lintel_dispenser *disp);
//uint8_t strat_pickup_lintels(void);
//
///* in strat_column_disp.c */
//uint8_t strat_eject_col(int16_t eject_a, int16_t pickup_a);
//uint8_t strat_pickup_columns(void);
//uint8_t strat_goto_col_disp(struct column_dispenser **disp);
//
///* in strat_building.c */
//uint8_t strat_goto_disc(int8_t level);
//uint8_t strat_goto_build_zone(struct build_zone *build_zone, uint8_t level);
//uint8_t strat_build_new_temple(struct build_zone *build_zone);
//uint8_t strat_goto_temple(struct temple *temple);
//uint8_t strat_grow_temple(struct temple *temple);
//uint8_t strat_grow_temple_column(struct temple *temple);
//struct temple *strat_get_best_temple(void);
//struct temple *strat_get_our_temple_on_disc(uint8_t valid);
//struct build_zone *strat_get_best_zone(void);
//struct temple *strat_get_free_temple(void);
//
///* in strat_scan.c */
//struct scan_disc_result;
//void scanner_dump_state(void);
//int8_t strat_scan_get_checkpoint(uint8_t mode, int16_t *ckpt_rel_x,
//				 int16_t *ckpt_rel_y, int16_t *back_mm);
//uint8_t strat_scan_disc(int16_t angle, uint8_t mode,
//			struct scan_disc_result *result);
//uint8_t strat_goto_disc_angle(int16_t a_deg, int8_t level);
//int16_t strat_get_temple_angle(struct temple *temple);
//int16_t strat_temple_angle_to_scan_angle(int16_t temple_angle);
//uint8_t strat_build_on_opponent_temple(void);
//uint8_t strat_check_temple_and_build(void);

#endif
