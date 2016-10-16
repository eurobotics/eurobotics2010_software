
/**********************************************************************
* © 2005 Microchip Technology Inc.
*
* FileName:        i2cEmem.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB® C30 v3.00 or higher
* Tested On:	   dsPIC33FJ256GP710
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip,s 
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP,S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Settu D.          07/09/06 	  First release of source file
* JBS				17/11/08	  Adaptacion firmware RAM.
**********************************************************************/
#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif

#include <stdlib.h>
#include <string.h>

#include "i2c_mst.h"

/* EEPROM DRIVER COMMAND DEFINITION */
#define I2C_IDLE 		 		 0  
#define I2C_WRITE        1      
#define I2C_READ         2    
#define I2C_ERR        0xFFFF

// I2C DATA OBJECT
typedef struct { 							
        uint8_t *buff;    
        uint16_t n;     	 
        uint16_t sub_addr;  	    
        uint16_t dev_addr;          
}I2C_DATA; 

// I2C DRIVER OBJECT
typedef struct { 
        unsigned int	cmd; 		
				I2C_DATA	*oData;	                      	
        void (*read_event) (uint8_t *buf, uint16_t size);
        void (*write_event) (uint16_t size);
        }I2C_DRV; 
    
#define I2C_DRV_DEFAULTS { 0,\
        (I2C_DATA *)0,\
        NULL,\
        NULL}  


uint8_t jDone=0;
I2C_DRV i2c_dev= I2C_DRV_DEFAULTS;                                  
I2C_DATA wData;
I2C_DATA rData;

uint8_t wBuff[I2C_BUFF_SIZE],rBuff[I2C_BUFF_SIZE];





void 
i2c_mst_register_write_event(void (*event)(uint16_t))
{
	uint8_t flags;
	IRQ_LOCK(flags);
	i2c_dev.write_event = event ;
	IRQ_UNLOCK(flags);
}

void 
i2c_mst_register_read_event(void (*event)(uint8_t *, uint16_t))
{
	uint8_t flags;
	IRQ_LOCK(flags);
	i2c_dev.read_event = event ;
	IRQ_UNLOCK(flags);
}


void i2c_mst_write(uint16_t dev_addr, uint16_t sub_addr, uint8_t *buff, uint16_t size)
{
		// Copy data
		memcpy(wBuff, buff, size);
		
		// Initialise I2C Data object for Write operation   
    wData.buff=wBuff;
    wData.n=size;
    wData.sub_addr=sub_addr; 
    wData.dev_addr=dev_addr;
		// Write Data
		i2c_dev.oData=&wData;
		i2c_dev.cmd = I2C_WRITE;	
			// Force interrupt
		IFS1bits.MI2C1IF = 1;
}

void i2c_mst_read(uint16_t dev_addr, uint16_t sub_addr, uint16_t size)
{
		// Initialise I2C Data object for Write operation   
    rData.buff=rBuff;
    rData.n=size;
    rData.sub_addr=sub_addr; 
    rData.dev_addr=dev_addr;
		// Write Data
		i2c_dev.oData=&rData;
		i2c_dev.cmd = I2C_READ;	
			// Force interrupt
		IFS1bits.MI2C1IF = 1;
}

void i2c_mst_init(void)
{   
  i2c_dev.cmd=0;
  i2c_dev.oData=0;

	I2C1CONbits.A10M=0;
	I2C1CONbits.SCLREL=1;
	I2C1CONbits.DISSLW = 0;
	I2C1BRG=0x188;

	I2C1ADD=0;
	I2C1MSK=0;

	I2C1RCV = 0x0000;
	I2C1TRN = 0x0000;

	I2C1CONbits.I2CEN=1;
	IEC1bits.MI2C1IE = 1;
  IFS1bits.MI2C1IF = 0;
}
                                                           
void i2c_mst_fsm(I2C_DRV *i2c_dev)
{

	static uint16_t state=0, cntr=0, rtrycntr=0;

  switch(state)
  {
    case 0: 
        if( (i2c_dev->cmd == I2C_WRITE)  || (i2c_dev->cmd == I2C_READ)  )
					state=1;

        break;

	/*==================================*/        
	/* Control/Address Phase			*/
	/*==================================*/
    case 1:
		// Start Condition
		I2C1CONbits.SEN=1;		
		state=state+1;
        break;


    case 2:
        // Start Byte with device select id
	    if(jDone==1)	{ 
			jDone=0;
          	state=state+1;
  			I2C1TRN=((i2c_dev->oData->dev_addr)<<1);
		}
        break;

    case 3:         
       	// Send address byte 1, if ack is received. Else Retry
        if(jDone==1)	{ 
			jDone=0;
  	
			if(I2C1STATbits.ACKSTAT==1) {		// Ack Not received, Retry

				if(rtrycntr < MAX_RETRY)
					state=18;
				else
					state=16;					// Flag error and exit

			} else {
 
				rtrycntr=0;

            	#if SUB_ADDRWIDTH==TWO_BYTE
            	I2C1TRN=((i2c_dev->oData->sub_addr)&0xFF00)>>8;
           		state=state+1;
            	#endif
               
            	#if SUB_ADDRWIDTH==ONE_BYTE
            	I2C1TRN=((i2c_dev->oData->sub_addr));
            	state=state+2;
            	#endif

			}
		}
		break;
    
    
    case 4:
		// Send address byte 2, if ack is received. Else Flag error and exit
      	if(jDone==1)	{ 
			jDone=0;

			if(I2C1STATbits.ACKSTAT==1) {		// Ack Not received, Flag error and exit
				state=16;

			} else {

            	#if SUB_ADDRWIDTH==TWO_BYTE
            	I2C1TRN=((i2c_dev->oData->sub_addr)&0x00FF);
            	#endif
            	state=state+1;
			}
		}
    	break;
    

    case 5:
		// Read or Write
      	if(jDone==1)	{ 
			jDone=0;

			if(I2C1STATbits.ACKSTAT==1) {		// Ack Not received, Flag error and exit
				state=16;

			} else {

        		if(i2c_dev->cmd == I2C_WRITE) 
					state=state+1;   
 
        		if(i2c_dev->cmd == I2C_READ) 
					state=8;
			}

		}
        break;

	/*==================================*/        
	/* Write Data Phase			        */
	/*==================================*/

    case 6:
		// Send data
        I2C1TRN=*(i2c_dev->oData->buff + cntr); 
        state=state+1;
        cntr=cntr+1;
        break;   

    case 7:  
		// Look for end of data or no Ack
      	if(jDone==1)	{ 
			jDone=0;
			state=state-1;

			if(I2C1STATbits.ACKSTAT==1) {		// Ack Not received, Flag error and exit
				state=16;
			} else {

        		if(cntr== i2c_dev->oData->n) 
					state=14;   				// Close the Frame
				}
		}
        break;

	/*==================================*/        
	/* Read Data Phase			        */
	/*==================================*/
    case 8:
		// Repeat Start
 		I2C1CONbits.RSEN=1;		
		state=state+1;
        break;

    case 9:
		// Re-send control byte with W/R=R
	    if(jDone==1)	{ 
			jDone=0;
          	state=state+1;
  			I2C1TRN=(0x0001)|((i2c_dev->oData->dev_addr)<<1);
		}
        break;

    case 10:    
		// Check, if control byte went ok
	    if(jDone==1)	{ 
			jDone=0;
          	state=state+1;

			if(I2C1STATbits.ACKSTAT==1) 		// Ack Not received, Flag error and exit
				state=16;
	
		}
        break;

    case 11:
      	// Receive Enable 
 		I2C1CONbits.RCEN=1;	
        state++;
        break;

    case 12:  
		// Receive data
	    if(jDone==1)	{ 
			jDone=0;
			state=state+1;

			*(i2c_dev->oData->buff+cntr)=I2C1RCV;
			cntr++;

        	if(cntr== i2c_dev->oData->n) {
				I2C1CONbits.ACKDT=1;		// No ACK		
			} else {
				I2C1CONbits.ACKDT=0;		// ACK
			}

			I2C1CONbits.ACKEN=1;	

		}
		break;

    case 13: 
	    if(jDone==1)	{ 
			jDone=0;
		  	if(cntr== i2c_dev->oData->n) 
            	state=state+1;
           	else
            	state=state-2;
         }          
         break;
         
	/*==================================*/        
	/* Stop Sequence			        */
	/*==================================*/
    case 14: 
      	I2C1CONbits.PEN=1;	
        state++;
        break;
           
    case 15: 
 	    if(jDone==1)	{ 
			jDone=0;
			state=0;
			
			/* Call funcion even */
			if(i2c_dev->cmd == I2C_READ)
				i2c_dev->read_event(i2c_dev->oData->buff, cntr);
			else if(i2c_dev->cmd == I2C_WRITE)
				i2c_dev->write_event(cntr);
			
			cntr=0;
			i2c_dev->cmd=0;
		}
      	break;

	/*==================================*/        
	/* Set Error     			        */
	/*==================================*/
    case 16:
       	I2C1CONbits.PEN=1;	
      	state++;
        break;
    
    case 17:
 	    if(jDone==1)	{ 
			jDone=0;
			state=0;
			rtrycntr=0;
			
			/* Call funcion even */
			if(i2c_dev->cmd == I2C_READ)
				i2c_dev->read_event(i2c_dev->oData->buff, cntr);
			else if(i2c_dev->cmd == I2C_WRITE)
				i2c_dev->write_event(cntr);
			
			cntr=0;
			i2c_dev->cmd=0xFFFF;
		}
      	break;

	/*==================================*/        
	/* Retry         			        */
	/*==================================*/
    case 18:
       	I2C1CONbits.PEN=1;	
      	state++;
		rtrycntr++;
        break;

    case 19:
 	    if(jDone==1)	{ 
			jDone=0;
			state=0;
			cntr=0;
		}
      	break;


 	}     

}

void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
{
		jDone=1;
    IFS1bits.MI2C1IF = 0;		//Clear the DMA0 Interrupt Flag;
    
    /* Call 3 times for avance in some states */
    //i2c_mst_fsm(&i2c_dev);
    //i2c_mst_fsm(&i2c_dev);
    //i2c_mst_fsm(&i2c_dev);
}


void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
{
	IFS1bits.SI2C1IF = 0;		//Clear the DMA0 Interrupt Flag
}

void i2c_mst_wait_end(void){
		while(i2c_dev.cmd!=I2C_IDLE){
			i2c_mst_fsm(&i2c_dev); 
		}
}
