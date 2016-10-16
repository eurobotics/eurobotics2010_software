/*  
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: uart_config.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/* Droids-corp 2004 - Zer0
 * config for uart module
 */

#ifndef UART_CONFIG_H
#define UART_CONFIG_H

/* compile uart0 fonctions, undefine it to pass compilation */
#define UART0_COMPILE  
#define UART0_ENABLED  1
#define UART0_INTERRUPT_ENABLED  1
#define UART0_BAUDRATE 57600
//#define UART0_BAUDRATE 1000000
#define UART0_USE_DOUBLE_SPEED 1
#define UART0_RX_FIFO_SIZE 32
#define UART0_TX_FIFO_SIZE 32
#define UART0_NBITS 8
#define UART0_PARITY UART_PARTITY_NONE
#define UART0_STOP_BIT UART_STOP_BITS_2

#define UART1_COMPILE  
#define UART1_ENABLED  1
#define UART1_INTERRUPT_ENABLED  1
#define UART1_BAUDRATE 57600
#define UART1_USE_DOUBLE_SPEED 1
#define UART1_RX_FIFO_SIZE 128
#define UART1_TX_FIFO_SIZE 128
#define UART1_NBITS 8
#define UART1_PARITY UART_PARTITY_NONE
#define UART1_STOP_BIT UART_STOP_BITS_1

#define UART3_COMPILE  
#define UART3_ENABLED  1
#define UART3_INTERRUPT_ENABLED  1
#define UART3_BAUDRATE 57600
#define UART3_USE_DOUBLE_SPEED 1
#define UART3_RX_FIFO_SIZE 128
#define UART3_TX_FIFO_SIZE 128
#define UART3_NBITS 8
#define UART3_PARITY UART_PARTITY_NONE
#define UART3_STOP_BIT UART_STOP_BITS_1

/* .... same for uart 1, 2, 3 ... */

#endif

