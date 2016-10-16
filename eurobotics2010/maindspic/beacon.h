
/*  
 *  Copyright EuRobotics Engineering (2010)
 *  Javier Baliñas Santos <balinas@gmail.com>
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

#ifndef _BEACON_H_
#define _BEACON_H_

#define BEACON_UART	1
#define LINE_BUFF_SIZE 	64
#define CMD_LINE_SIZE 	16

void beacon_cmd_wt11_local_reset(void);
void beacon_cmd_wt11_call(void);
void beacon_cmd_wt11_close(void);
void beacon_daemon(void * dummy);
void beacon_send_daemon(void * dummy);
void beacon_recv_daemon(void);
void beacon_init(void);

void beacon_cmd_color(void);
void beacon_cmd_opponent(void);
void beacon_cmd_beacon_on(void);
void beacon_cmd_beacon_off(void);

#endif
