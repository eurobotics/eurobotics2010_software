/*
*	Grupo de Robótica Educativa - Departamento de Electrónica - Universidad de Alcalá
*	---------------------------------------------------------------------------------
*
*	Acuerdo de Licencia Software:
*
*	(CC) Creative Commons, Reconocimiento - No Comercial - Compartir Igual 2.5 España
*
*	Usted es libre de:
*		+ copiar, distribuir y comunicar públicamente la obra
*		+ hacer obras derivadas
*
*	Bajo las condiciones siguientes:
*		+ Reconocimiento. Debe reconocer los créditos de la obra de la manera especificada
*		  por el autor o el licenciador (pero no de una manera que sugiera que tiene su apoyo o
*		  apoyan el uso que hace de su obra).
*		+ No comercial. No puede utilizar esta obra para fines comerciales
*		+ Compartir bajo la misma licencia. Si altera o transforma esta obra, o genera una obra
*		  derivada, sólo puede distribuir la obra generada bajo una licencia idéntica a ésta.
*
*		- Al reutilizar o distribuir la obra, tiene que dejar bien claro los términos de la licencia
*		  de esta obra.
*		- Alguna de estas condiciones puede no aplicarse si se obtiene el permiso del titular de
*		  los derechos de autor.
*		- Nada en esta licencia menoscaba o restringe los derechos morales del autor.
*
*******************************************************************************************************
*
*	Grupo de Robótica Educativa - Departamento de Electrónica - Universidad de Alcalá
*	---------------------------------------------------------------------------------
*
*	Software License Agreement:
*
*	(CC) Creative Commons, Attribution - NonCommercial - ShareAlike 2.5 Spain
*
*	You are free:
*		+ to Share — to copy, distribute and transmit the work
*		+ to Remix — to adapt the work
*
*	Under the following conditions:
*		+ Attribution. You must attribute the work in the manner specified by the author or
*		  licensor (but not in any way that suggests that they endorse you or your use of the work).
*		+ Noncommercial. You may not use this work for commercial purposes.
*		+ Share Alike. If you alter, transform, or build upon this work, you may distribute the
*		  resulting work only under the same or similar license to this one.
*
*		- For any reuse or distribution, you must make clear to others the license terms of this work.
*		  The best way to do this is with a link to this web page.
*		- Any of the above conditions can be waived if you get permission from the copyright holder.
*		- Nothing in this license impairs or restricts the author's moral rights.
*/

// Libreria para el control del I2C MAESTRO MEDIANTE PAQUETES

#ifndef I2C_MST_C
#define I2C_MST_C

#include "i2c_master.h"

// Defines

// Definicion de bits para configuracion

#define I2C_ON	                   0xFFFF	//!< I2C module enabled
#define I2C_OFF	                   0x7FFF	//!< I2C module disabled

#define I2C_IDLE_STOP              0xFFFF	//!< stop I2C module in Idle mode
#define I2C_IDLE_CON               0xDFFF	//!< continue I2C module in Idle mode

#define I2C_CLK_REL                0xFFFF	//!< release clock
#define I2C_CLK_HLD                0xEFFF	//!< hold clock

#define I2C_IPMI_EN                0xFFFF	//!< IPMI mode enabled
#define I2C_IPMI_DIS               0xF7FF	//!< IPMI mode not enabled

#define I2C_10BIT_ADD              0xFFFF	//!< I2CADD is 10-bit address
#define I2C_7BIT_ADD               0xFBFF	//!< I2CADD is 7-bit address

#define I2C_SLW_DIS                0xFFFF	//!< Disable Slew Rate Control for 100KHz
#define I2C_SLW_EN                 0xFDFF	//!< Enable Slew Rate Control for 400KHz

#define I2C_SM_EN                  0xFFFF	//!< Enable SM bus specification
#define I2C_SM_DIS                 0xFEFF	//!< Disable SM bus specification

#define I2C_GCALL_EN               0xFFFF	//!< Enable Interrupt when General call address is received.
#define I2C_GCALL_DIS              0xFF7F	//!< Disable General call address.

#define I2C_STR_EN                 0xFFFF	//!< Enable clock stretching
#define I2C_STR_DIS                0xFFBF	//!< disable clock stretching 

#define I2C_ACK                    0xFFDF	//!< Transmit 0 to send ACK as acknowledge
#define I2C_NACK                   0xFFFF	//!< Transmit 1 to send NACK as acknowledge

#define I2C_ACK_EN                 0xFFFF	//!< Initiate Acknowledge sequence
#define I2C_ACK_DIS                0xFFEF	//!< Acknowledge condition Idle

#define I2C_RCV_EN                 0xFFFF	//!< Enable receive mode
#define I2C_RCV_DIS                0xFFF7	//!< Receive sequence not in progress

#define I2C_STOP_EN                0xFFFF	//!< Initiate Stop sequence
#define I2C_STOP_DIS               0xFFFB	//!< Stop condition Idle

#define I2C_RESTART_EN             0xFFFF	//!< Initiate Restart sequence
#define I2C_RESTART_DIS            0xFFFD	//!< Start condition Idle

#define I2C_START_EN               0xFFFF	//!< Initiate Start sequence
#define I2C_START_DIS              0xFFFE	//!< Start condition Idle

////////////////////////////////////////////////////////////////////////////////////
// Variables internas (globales a 'i2c_master.c')

volatile unsigned char i2c_master_status;
volatile char i2c_error;
// unsigned char variable_basura; // Sin uso

// Buffers para guardar paquetes
volatile struct i2c_tpackage i2c_prior0_buffer[I2C_BUFFER_SIZE_PRIOR0];
volatile struct i2c_tpackage i2c_prior1_buffer[I2C_BUFFER_SIZE_PRIOR1];
volatile struct i2c_tpackage i2c_prior2_buffer[I2C_BUFFER_SIZE_PRIOR2];

// Punteros a los buffers
volatile unsigned char i2c_buffer_num_data[3]; // Número de datos pendientes en los tres búffers
volatile unsigned char i2c_prod_pos[3]; // Posiciones de los tres punteros productores del buffer circular
volatile unsigned char i2c_cons_prod[3]; // Posiciones de los tres punteros consumidores del buffer circular
volatile unsigned char i2c_buffer_size[3]={I2C_BUFFER_SIZE_PRIOR0,I2C_BUFFER_SIZE_PRIOR1,I2C_BUFFER_SIZE_PRIOR2}; // Array con los tamaños de los tres búffers.

//void i2c_recvevent(struct i2c_tpackage *i2c_pakage);
//void i2c_sendevent(struct i2c_tpackage *i2c_pakage);


// Interrupcion del I2C Master
void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
{	
	static char package_priority; // Para saber en la maquina con que buffer estamos trabajando

	_MI2C1IF = 0; // Borramos flag de interrupcion
	
	switch (i2c_master_status)
	{
		case 0:
		{
			if(i2c_buffer_num_data[I2C_PRIOR0] > 0) // Paquetes pendientes prioridad 0
			{	
				package_priority = 0;

				I2C1STATbits.IWCOL = 0;
				I2C1CONbits.SEN = 1;		// START  

				i2c_error = 0;

				if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_RW == 0)
				{
					i2c_master_status = 1; // ESCRIBIR
				
				}
				else if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_RW == 1)
				{
					i2c_master_status = 11; // ESCRIBIR i2c_sub_address Y LEER
				}
				else // == 2
				{
					i2c_master_status = 14; // LEER UNICAMENTE
				}
			}
			else if(i2c_buffer_num_data[I2C_PRIOR1] > 0) // Paquetes pendientes prioridad 1
			{	
				package_priority = 1;

				I2C1STATbits.IWCOL = 0;
				I2C1CONbits.SEN = 1;		// START  

				if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_RW == 0)
				{
					i2c_master_status = 1; // ESCRIBIR
				}
				else if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_RW == 1)
				{
					i2c_master_status = 11; // ESCRIBIR i2c_sub_address Y LEER
				}
				else // == 2
				{
					i2c_master_status = 14; // LEER UNICAMENTE
				}
			}
			else if(i2c_buffer_num_data[I2C_PRIOR2] > 0) // Paquetes pendientes prioridad 2
			{	
				package_priority = 2;

				I2C1STATbits.IWCOL = 0;
				I2C1CONbits.SEN = 1;		// START  

				if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_RW == 0)
				{
					i2c_master_status = 1; // ESCRIBIR
				}
				else if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_RW == 1)
				{
					i2c_master_status = 11; // ESCRIBIR i2c_sub_address Y LEER
				}
				else // == 2
				{
					i2c_master_status = 14; // LEER UNICAMENTE
				}
			}
			else
			{	
				_MI2C1IE = 0;	// Deshabilitamos interrupcion
				return;
			}
			break;
		}
		case 1:
		{
			if(I2C1CONbits.SEN == 0) // Ha finalizado correctamente el proceso de START
			{ 
				switch (package_priority)
				{
					case 0:
					{
						I2C1TRN = (i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
					case 1:
					{
						I2C1TRN = (i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
					case 2:
					{
						I2C1TRN = (i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
				}
				
				
				if(I2C1STATbits.IWCOL)        // write collision
    			{	
					I2C1STATbits.IWCOL = 0;
					I2C1CONbits.PEN = 1;	// STOP //

					i2c_error = 1;

					i2c_master_status = 5; // Saltamos al ultimo estado para terminar
	    			break;
				}
				else
				{
					i2c_master_status = 2;	// Escritura de Device-Address OK
					break;
				}
			}
			else
			{
				break;
			}
		
		}// case 1
		case 2:
		{
			if(I2C1STATbits.ACKSTAT == 1)	// NACK
			{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 2;

					i2c_master_status = 5; // Saltamos al ultimo estado para terminar
					break;
			} 
			else	// ACK
			{
							
				switch (package_priority)
				{
					case 0:
					{
						I2C1TRN = i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_sub_address; // Sub Address
						break;
					}
					case 1:
					{
						I2C1TRN = i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_sub_address; // Sub Address
						break;
					}
					case 2:
					{
						I2C1TRN = i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_sub_address; // Sub Address
						break;
					}
				}

					if(I2C1STATbits.IWCOL)        // write collision
    			{	
						I2C1STATbits.IWCOL = 0;
						I2C1CONbits.PEN = 1;	// STOP //

						i2c_error = 3;

						i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
	    			break;
					}
					else
					{
						i2c_master_status = 3;	// Escritura de Sub-Address OK
					}

					break;
			} 
		
		} // case 2
		case 3:
		{
			if(I2C1STATbits.ACKSTAT == 1)	// NACK 
			{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 4;

					i2c_master_status = 5;
					break;
			}
			else	// ACK de la SUB ADDRESS
			{

				i2c_master_status = 4;  // ESTADO POR DEFECTO: SEGUIR ESCRIBIENDO DATOS

				switch (package_priority)
				{
					case 0:
					{
						if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							I2C1CONbits.PEN = 1;	// STOP //
							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
						}
						else // Hay que escribir datos
						{
							I2C1TRN = *i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_ptr_data; // Data
						}

						break;
					}
					case 1:
					{
						if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							I2C1CONbits.PEN = 1;	// STOP //
							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
						}
						else // Hay que escribir datos
						{
							I2CTRN = *i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_ptr_data; // Data
						}

						break;
					}
					case 2:
					{
						if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							I2C1CONbits.PEN = 1;	// STOP //
							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
						}
						else // Hay que escribir datos
						{
							I2CTRN = *i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_ptr_data; // Data
						}
						break;
					}
				}

					if(I2C1STATbits.IWCOL)        // write collision
    				{	
						I2C1STATbits.IWCOL = 0;
						I2C1CONbits.PEN = 1;	// STOP //

						i2c_error = 5;

						i2c_master_status = 5;  
	    				break;
					}
				
					break;
				}
		
			} // case 3

		case 4:
		{
			if(I2C1STATbits.ACKSTAT == 1)	// NACK 
			{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 6;

					i2c_master_status = 5;
					break;
			}
			else	// ACK de la DATA
			{
				switch (package_priority)
				{
					case 0:
					{
						// HAY MAS DATOS?
						i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_num_data--;

						if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							*i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_sent_flag = 1; // Paquete enviado

							I2C1CONbits.PEN = 1;	// STOP //

							i2c_error = 7;

							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
				
							/* execute send event function */
							//i2c_sendevent((struct i2c_tpackage*)&i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_device_address);	

							break;
						}
						else	// HAY MAS DATOS
						{
							// Apuntamos al siguiente byte
							i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_ptr_data++;

							I2C1TRN = *i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_ptr_data; // Data
				
							if(I2C1STATbits.IWCOL)        // write collision
    						{	
								I2C1STATbits.IWCOL = 0;
								I2C1CONbits.PEN = 1;	// STOP //

								i2c_error = 8;

								i2c_master_status = 5;  
	    						break;
							} // if
							else
							{
								i2c_master_status = 4;  // COMPROBAMOS EL ACK del DATA
								break;
							} // else
						} // else

						break;
					} // case 0

					case 1:
					{
						// HAY MAS DATOS?
						i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_num_data--;

						if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							*i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_sent_flag = 1; // Paquete enviado

							I2C1CONbits.PEN = 1;	// STOP //

							i2c_error = 9;

							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
				
							break;
						}
						else	// HAY MAS DATOS
						{
							// Apuntamos al siguiente byte
							i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_ptr_data++;

							I2C1TRN = *i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_ptr_data; // Data
				
							if(I2C1STATbits.IWCOL)        // write collision
    						{	
								I2C1STATbits.IWCOL = 0;
								I2C1CONbits.PEN = 1;	// STOP //

								i2c_error = 10;

								i2c_master_status = 5;  
	    						break;
							} // if
							else
							{
								i2c_master_status = 4;  // COMPROBAMOS EL ACK del DATA
								break;
							} // else
						} // else

						break;
					} // case 1

					case 2:
					{
							// HAY MAS DATOS?
						i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_num_data--;

						if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_num_data == 0) //  NO HAY MAS DATOS
						{
							*i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_sent_flag = 1; // Paquete enviado

							I2C1CONbits.PEN = 1;	// STOP //

							i2c_error = 11;

							i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
				
							break;
						}
						else	// HAY MAS DATOS
						{
							// Apuntamos al siguiente byte
							i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_ptr_data++;

							I2C1TRN = *i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_ptr_data; // Data
				
							if(I2C1STATbits.IWCOL)        // write collision
    						{	
								I2C1STATbits.IWCOL = 0;
								I2C1CONbits.PEN = 1;	// STOP //

								i2c_error = 12;

								i2c_master_status = 5;  
	    						break;
							} // if
							else
							{
								i2c_master_status = 4;  // COMPROBAMOS EL ACK del DATA
								break;
							} // else
						} // else

						
						break;
					} // case 2

				} // switch (package_priority)
			} // else	// ACK de la DATA

			break;
						
		} // case 4


		case 5:
			{
				if(I2C1CONbits.PEN == 0)
				{
					i2c_master_status = 0;
					
					// si ha habido error de transmision, hay que registrarlo en el momento
					// del error,(en la matriz de errores): el dato del buffer se va a perder.


				switch (package_priority)
				{
					case 0:
					{
							i2c_buffer_num_data[I2C_PRIOR0]--;
							i2c_cons_prod[I2C_PRIOR0]++;
							i2c_cons_prod[I2C_PRIOR0] &= (I2C_BUFFER_SIZE_PRIOR0 - 1);

						break;
					}
					case 1:
					{
							i2c_buffer_num_data[I2C_PRIOR1]--;
							i2c_cons_prod[I2C_PRIOR1]++;
							i2c_cons_prod[I2C_PRIOR1] &= (I2C_BUFFER_SIZE_PRIOR1 - 1);

						break;
					}
					case 2:
					{
							i2c_buffer_num_data[I2C_PRIOR2]--;
							i2c_cons_prod[I2C_PRIOR2]++;
							i2c_cons_prod[I2C_PRIOR2] &= (I2C_BUFFER_SIZE_PRIOR2 - 1);

						break;
					}
				} // switch (package_priority)

					if(i2c_buffer_num_data[I2C_PRIOR0] == 0)
		 			{
						if(i2c_buffer_num_data[I2C_PRIOR1] == 0)
						{
							if(i2c_buffer_num_data[I2C_PRIOR2] == 0)
							{
								_MI2C1IE = 0; // El buffer se encuentra vacio
								break;
							}
							else
							{
									package_priority = 2;

									I2C1STATbits.IWCOL = 0;
									I2C1CONbits.SEN = 1;		// START  

									if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_RW == 0)
									{
										i2c_master_status = 1; // ESCRIBIR
									}
									else if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_RW == 1)
									{
										i2c_master_status = 11; // LEER
									} // else
									else // == 2
									{
										i2c_master_status = 14; // LEER UNICAMENTE
									}

									break;
							}
						}
						else
						{
							package_priority = 1;

							I2C1STATbits.IWCOL = 0;
							I2C1CONbits.SEN = 1;		// START  

							if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_RW == 0)
							{
								i2c_master_status = 1; // ESCRIBIR
							}
							else if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_RW == 1)
							{
								i2c_master_status = 11; // LEER
							} // else
							else // == 2
							{
								i2c_master_status = 14; // LEER UNICAMENTE
							}

							break;
						}
					}
					else	// Hay mas datos en el buffer: (i2c_buffer_num_data[I2C_PRIOR0] != 0)
					{	
						package_priority = 0;

						I2C1STATbits.IWCOL = 0;
						I2C1CONbits.SEN = 1;		// START  

						if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_RW == 0)
						{
							i2c_master_status = 1; // ESCRIBIR
						}
						else if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_RW == 1)
						{
							i2c_master_status = 11; // LEER
						} // else
						else // == 2
						{
							i2c_master_status = 14; // LEER UNICAMENTE
						}

						break;
					} // else

					break;
				}
				else
				{
					break;
				}
				
			} // case 5
		case 11:
		{
			if(I2C1CONbits.SEN == 0) // Ha finalizado correctamente el proceso de START
			{
				switch (package_priority)
				{
					case 0:
					{
						I2C1TRN = (i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
					case 1:
					{
						I2C1TRN = (i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
					case 2:
					{
						I2C1TRN = (i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_device_address << 1) & 0xFE; // Device Address & Escritura
						break;
					}
				}

				if(I2C1STATbits.IWCOL)        // write collision
    			{	
					I2C1STATbits.IWCOL = 0;
					I2C1CONbits.PEN = 1;	// STOP //

					i2c_error = 13;

					i2c_master_status = 5; // Saltamos al ultimo estado para terminar
	    			break;
				}
				else
				{
					i2c_master_status = 12;	// Escritura de Device-Address OK
					break;
				}
			}
			else
			{
				break;
			}
		
		}// case 11

	case 12:
		{
			if(I2C1STATbits.ACKSTAT == 1)	// NACK
			{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 14;

					i2c_master_status = 5; // Saltamos al ultimo estado para terminar
					break;
			} 
			else	// ACK
			{
									
				switch (package_priority)
				{
					case 0:
					{
						I2C1TRN = i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_sub_address; // Sub Address
						break;
					}
					case 1:
					{
						I2C1TRN = i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_sub_address; // Sub Address
						break;
					}
					case 2:
					{
						I2C1TRN = i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_sub_address; // Sub Address
						break;
					}
				}
				
					if(I2C1STATbits.IWCOL)        // write collision
    				{	
						I2C1STATbits.IWCOL = 0;
						I2C1CONbits.PEN = 1;	// STOP //

						i2c_error = 15;

						i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
	    				break;
					}
					else
					{
						i2c_master_status = 13;	// Escritura de Sub-Address OK
					}

					break;
			} // case 12
		
		}
		case 13:
		{
			if(I2C1STATbits.ACKSTAT == 1)	// NACK
			{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 16;

					i2c_master_status = 5;
					break;
			}
			else	// ACK
			{
					
					I2C1STATbits.IWCOL = 0;
					I2C1CONbits.RSEN = 1;		// RESTART  

					i2c_master_status = 14;

					break;
				}
		
			} // case 13
			case 14:
			{
					// Ha finalizado correctamente el proceso de RESTART (O START, en caso de SOLO LECTURA)
					if((I2C1CONbits.RSEN == 0)&&(I2C1CONbits.SEN == 0))
					{
						
						switch (package_priority)
						{
							case 0:
							{
								I2C1TRN = (i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_device_address << 1) | 0x01; // Device Address + Lectura
								break;
							}
							case 1:
							{
								I2C1TRN = (i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_device_address << 1) | 0x01; // Device Address + Lectura
								break;
							}
							case 2:
							{
								I2C1TRN = (i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_device_address << 1) | 0x01; // Device Address + Lectura
								break;
							}
						}

						if(I2C1STATbits.IWCOL)        // write collision
    					{	
							I2C1STATbits.IWCOL = 0;
							I2C1CONbits.PEN = 1;	// STOP //

							i2c_error = 17;

							i2c_master_status = 5; // Saltamos al ultimo estado para terminar
	    					break;
						}
						else
						{
							i2c_master_status = 15;	// Escritura de Device-Address OK
							break;
						}
					} // if(
				
			
				break;
			
				
			} // case 14

			case 15:
			{
				if(I2C1STATbits.ACKSTAT == 1)	// NACK
				{
					I2C1CONbits.PEN = 1;	// STOP

					i2c_error = 18;

					i2c_master_status = 5; // Saltamos al ultimo estado para terminar
					break;
				} 
				else	// ACK
				{
						// Recibimos dato

					_RCEN = 1; // Comenzar lectura
				
					i2c_master_status = 16;

					break;
				}
			} // case 15
			case 16:
			{
				if(I2C1CONbits.RCEN == 1)	
				{
					break;
				} 
				else	// Receive Complete
				{
						switch (package_priority)
						{
							case 0:
							{
								
								*i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_ptr_data = I2CRCV;

								// HAY MAS DATOS?
								i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_num_data--;

								if(i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_num_data == 0) //  NO HAY MAS DATOS
								{
									*i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_sent_flag = 1; // Paquete enviado

									_ACKDT = 1; 	// Enviamos NACK
									_ACKEN = 1; // Iniciamos NACK

									i2c_master_status = 17;

									i2c_error = 19;
									
									/* execute recv event function */
									//i2c_recvevent((struct i2c_tpackage*)&i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_device_address);	

									break;
								}
								else	// SI:HAY MAS DATOS
								{
									_ACKDT = 0; 	// Enviamos ACK
									_ACKEN = 1; // Iniciamos ACK

									// Apuntamos al siguiente byte
									i2c_prior0_buffer[i2c_cons_prod[I2C_PRIOR0]].i2c_ptr_data++;

									i2c_master_status = 18; // Esperamos a que finalice el ACK para recibir otro dato

									break;
								} // else	// SI:HAY MAS DATOS

								break;
							}
							case 1:
							{

								*i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_ptr_data = I2CRCV;

								// HAY MAS DATOS?
								i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_num_data--;

								if(i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_num_data == 0) //  NO HAY MAS DATOS
								{
	
									*i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_sent_flag = 1; // Paquete enviado

									_ACKDT = 1; 	// Enviamos NACK
									_ACKEN = 1; // Iniciamos NACK

									i2c_master_status = 17;

									i2c_error = 20;
										
									break;
								}
								else	// SI:HAY MAS DATOS
								{
									_ACKDT = 0; 	// Enviamos ACK
									_ACKEN = 1; // Iniciamos ACK

									// Apuntamos al siguiente byte
									i2c_prior1_buffer[i2c_cons_prod[I2C_PRIOR1]].i2c_ptr_data++;

									i2c_master_status = 18; // Esperamos a que finalice el ACK para recibir otro dato

									break;
								} // else	// SI:HAY MAS DATOS

								break;
							}
							case 2:
							{
								
								*i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_ptr_data = I2CRCV;

								// HAY MAS DATOS?
								i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_num_data--;

								if(i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_num_data == 0) //  NO HAY MAS DATOS
								{
									*i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_sent_flag = 1; // Paquete enviado

									_ACKDT = 1; 	// Enviamos NACK
									_ACKEN = 1; // Iniciamos NACK

									i2c_master_status = 17;

									i2c_error = 21;
									
									/* execute recv event function */
										
									break;
								}
								else	// SI:HAY MAS DATOS
								{
									_ACKDT = 0; 	// Enviamos ACK
									_ACKEN = 1; // Iniciamos ACK

									// Apuntamos al siguiente byte
									i2c_prior2_buffer[i2c_cons_prod[I2C_PRIOR2]].i2c_ptr_data++;

									i2c_master_status = 18; // Esperamos a que finalice el ACK para recibir otro dato

									break;
								} // else	// SI:HAY MAS DATOS

								break;
							}
						}

				} // else	// Receive Complete

				break;
			} // case 16

			case 17:
			{
				if(I2C1CONbits.ACKEN == 1)	
				{
					break;
				} 
				else	// NACK Complete
				{
					I2C1CONbits.PEN = 1;	// STOP //

					i2c_error = 22;

					i2c_master_status = 5;  // Saltamos al ultimo estado para terminar
	
					break;
				}
			} // case 17
			case 18:
			{
				if(I2C1CONbits.ACKEN == 1)	
				{
					break;
				} 
				else	// ACK Complete
				{
					_RCEN = 1; // Comenzar lectura
				
					i2c_master_status = 16;

					i2c_error = 23;

					break;
				}
			} // case 18
		
	}	// switch
	

}


void I2C_master_init(void)
{
	//I2CBRG = 47; // 400Khz
	I2C1BRG = 0x188;  // 100Khz / Fcy = 40 MHZ

	I2C1CON = (I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD
		& I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS & I2C_NACK
		& I2C_ACK_DIS & I2C_RCV_DIS & I2C_STOP_DIS & I2C_RESTART_DIS & I2C_START_DIS);

	// Inicializamos datos de los buffers

	i2c_buffer_num_data[I2C_PRIOR0] = 0;
	i2c_prod_pos[I2C_PRIOR0] = 0;
	i2c_cons_prod[I2C_PRIOR0] = 0;

	i2c_buffer_num_data[I2C_PRIOR1] = 0;
	i2c_prod_pos[I2C_PRIOR1] = 0;
	i2c_cons_prod[I2C_PRIOR1] = 0;

	i2c_buffer_num_data[I2C_PRIOR2] = 0;
	i2c_prod_pos[I2C_PRIOR2] = 0;
	i2c_cons_prod[I2C_PRIOR2] = 0;
	///

	i2c_master_status = 0; // inicializamos la maquina de estados

	_MI2C1IP = 6; // Prioridad I2C MASTER

	_MI2C1IF = 0;

	_MI2C1IE = 1; // Habilitamos interrupciones
}

////////////////////////////////////////////////////////////////////
//
//	COLOCAR PAQUETE de PRIORIDAD 0 EN BUFFER CIRCULAR
//
// Devuelve:
// 0: buffer lleno
// 1: insercion correcta del paquete
////////////////////////////////////////////////////////////////////
char I2C_send_package(volatile struct i2c_tpackage i2c_package, unsigned char i2c_send_priority)
{
	if (i2c_send_priority > 2)
		return 0;
	
	_MI2C1IE = 0; // desHabilitamos interrupciones
	
	if(i2c_buffer_num_data[i2c_send_priority] < i2c_buffer_size[i2c_send_priority]) // Hay lugar en el BUFFER
	{
		i2c_prior0_buffer[i2c_prod_pos[i2c_send_priority]] = i2c_package; 
	
		i2c_buffer_num_data[i2c_send_priority]++;

		i2c_prod_pos[i2c_send_priority]++;

		i2c_prod_pos[i2c_send_priority] &= (i2c_buffer_size[i2c_send_priority] - 1);

		if (i2c_send_priority == I2C_PRIOR0)
		{
			if(i2c_buffer_num_data[I2C_PRIOR0] == 1)
				if(i2c_buffer_num_data[I2C_PRIOR1] == 0)
					if(i2c_buffer_num_data[I2C_PRIOR2] == 0)
							_MI2C1IF = 1;	// si introducimos el primer dato, llamamos a la interrupcion
		}
		if (i2c_send_priority == I2C_PRIOR1)
		{
			if(i2c_buffer_num_data[I2C_PRIOR1] == 1)
				if(i2c_buffer_num_data[I2C_PRIOR0] == 0)
					if(i2c_buffer_num_data[I2C_PRIOR2] == 0)
						_MI2C1IF = 1;	// si introducimos el primer dato, llamamos a la interrupcion
		}
		if (i2c_send_priority == I2C_PRIOR2)
		{
			if(i2c_buffer_num_data[I2C_PRIOR2] == 1)
				if(i2c_buffer_num_data[I2C_PRIOR0] == 0)
					if(i2c_buffer_num_data[I2C_PRIOR1] == 0)
						_MI2C1IF = 1;	// si introducimos el primer dato, llamamos a la interrupcion
		}
					

		_MI2C1IE = 1;	// Habilitamos interrupcion

		return 1 ;
	}
	else	// Buffer lleno
	{
		_MI2C1IE = 1;	// Habilitamos interrupcion

		return 0;
	}	
}



void I2C_master_int_enable(unsigned char i2c_int_enable)
{
	if (i2c_int_enable)
		_MI2C1IE = 1;	// Habilitamos interrupcion
	else
		_MI2C1IE = 0; // desHabilitamos interrupciones
}







#endif
