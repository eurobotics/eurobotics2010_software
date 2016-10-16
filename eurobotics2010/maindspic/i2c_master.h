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

//! \brief Libreria para el control del I2C MAESTRO MEDIANTE PAQUETES (tarjetas miniAlcadsPIC, AlcadsPIC)

//! \file i2c_master.h
//! Esta librería inicializa y controla el I2C Maestro.
//! Los paquetes son insertados en un buffer FIFO y una maquina de estados se encarga de enviarlos segun su prioridad.
//! Una vez que el paquete es enviado o recibido, se avisa con un flag en una variable preasignada.
//! \version V1.3
//! \date 17 de Marzo de 2009
//! \author Marcelo Salazar Arcucci
//! \htmlonly <BR><BR> <BR>  <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/2.5/es/"> <img alt="Creative Commons License" style="border-width:0" src="http://i.creativecommons.org/l/by-nc-sa/2.5/es/88x31.png" /> </a> <br />Esta obra está bajo una <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/2.5/es/">licencia de Creative Commons</a>. \endhtmlonly

// Versiones
// V1.3		Código ordenado para una mayor facilidad de comprensión
// V1.2		Agregada funcionalidad para solo leer. Compatibilidad con dsPIC30F4012.
// V1.1		Modificaciones para la proteccion de variables
// V1.0		Version de la Campus Party 2006
// V1.1		Adaptacion para dspic33

#ifndef I2C_MST_H
#define I2C_MST_H

/*
** includes for dsPIC
*/
#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#endif 

//#if defined(__dsPIC30F4012__)
//#include <p30f4012.h>
//#elif defined(__dsPIC30F6010A__)
//#include <p30f6010A.h>
//#else
//#error -- Procesador NO soportado (de momento)
//#endif

// Defines

#define I2C_BUFFER_SIZE_PRIOR0 8 	//!< Tamaño del buffer FIFO, prioridad 0 (2^x)
#define I2C_BUFFER_SIZE_PRIOR1 1 	//!< Tamaño del buffer FIFO, prioridad 1 (2^x)
#define I2C_BUFFER_SIZE_PRIOR2 1 	//!< Tamaño del buffer FIFO, prioridad 2 (2^x)
#define I2C_PRIOR0 0	//!< Prioridad 0 de envío	
#define I2C_PRIOR1 1	//!< Prioridad 1 de envío	
#define I2C_PRIOR2 2	//!< Prioridad 2 de envío

#define I2C_WR		0
#define I2C_WR_RD	2
#define I2C_RD		1	

// ****************************************************************************
// Variables

/*! \struct i2c_tpackage
	\brief Estructura de paquetes de I2C
*/
struct i2c_tpackage  // Tipo de paquete
{
	volatile char i2c_device_address;	//!< Direccion de dispositivo
	volatile char i2c_sub_address;	//!< Subdireccion del dispositivo
	volatile char *i2c_ptr_data;	//!< Puntero a datos, donde escribir o leer.
	volatile char i2c_num_data;	//!< Numero de datos a leer o escribir
	volatile char i2c_RW;		//!< Operacion: '0' escritura, '1' escritura/lectura, '2' solo lectura
	volatile char * i2c_sent_flag;	//!< Puntero a variable donde escribir cuando se ha enviado o recibido el paquete: '1' enviado, '0' en cola
};

//! Inicializacion del I2C Master mediante paquetes
void I2C_master_init(void);

//! Envio de un paquete de I2C con prioridad 0

/*!
	\param i2c_package estructura con datos acerca de la comunicacion I2C a realizar
	\param i2c_send_priority indica la prioridad con la que queremos enviar el paquete

	\return '1' paquete insertado en el buffer correctamente, '0' buffer lleno: paquete no insertado
*/
char I2C_send_package(volatile struct i2c_tpackage i2c_package, unsigned char i2c_send_priority);

//! Habilita / Deshabilita la interrupcion I2C Master
/*!
	\param i2c_int_enable '1' habilita la interrupción, '0' deshabilita la interrupción
*/
void I2C_master_int_enable(unsigned char i2c_int_enable);



#endif
