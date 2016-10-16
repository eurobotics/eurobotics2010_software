/**********************************************************************
* � 2005 Microchip Technology Inc.
*
* FileName:        i2cEmem.h
* Dependencies:    Other (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx//PIC24Hxxxx
* Compiler:        MPLAB� C30 v3.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (�Microchip�) licenses this software to you
* solely for use with Microchip dsPIC� digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED �AS IS.�  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Settu D.			07/09/06  First release of source file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*
* ADDITIONAL NOTES:
*
**********************************************************************/
#ifndef _I2C_MST_H
#define _I2C_MST_H 

#include <aversive.h>

#define MAX_RETRY	 1000

#define ONE_BYTE     		1
#define TWO_BYTE     		2
#define SUB_ADDRWIDTH   ONE_BYTE     

#define I2C_BUFF_SIZE 32

void i2c_mst_register_write_event(void (*event)(uint16_t));
void i2c_mst_register_read_event(void (*event)(uint8_t *, uint16_t));
void i2c_mst_write(uint16_t dev_addr, uint16_t sub_addr, uint8_t *buff, uint16_t size);
void i2c_mst_read(uint16_t dev_addr, uint16_t sub_addr, uint16_t size);
void i2c_mst_init(void);

void i2c_mst_wait_end(void);


#endif















































































































































































































































































































































































































































































































































































































































































































































































































