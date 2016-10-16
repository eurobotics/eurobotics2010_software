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
 *  Revision : $Id: actuator.h,v 1.2 2009/04/24 19:30:41 zer0 Exp $
 *
 */

void dac_set_and_save(void *dac, int32_t val);

#define PWM_BALL_LID_R_OPEN		350
#define PWM_BALL_LID_R_CLOSE	700

#define PWM_BALL_LID_L_OPEN 	775
#define PWM_BALL_LID_L_CLOSE	375


void ball_lids_close(void);
void ball_lids_open(void);
