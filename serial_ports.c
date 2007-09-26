/*******************************************************************************
*
*	TITLE:		serial_ports.c 
*
*	VERSION:	0.4 (Beta)                           
*
*	DATE:		10-Jan-2006
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	The source code in this file implements a fully buffered,
*				interrupt-driven serial port driver that can be used with
*				either or both on-board serial ports.
*
*				This source code will work with the Robovation (A/K/A EDU-RC) 
*				robot controller and the FIRST Robotics robot controller.
*
*				By default, serial port one will operate at 115200 baud, which
*				is compatible with InnovationFIRST's terminal program, and
*				serial port two will operate at 9600 baud. These values can be
*				easily changed by modifying the	serial port initialization 
*				functions mentioned below.
*
*               This file is best viewed with tabs set to four characters.
*
*				Nine things must be done before this software will work 
*				correctly:
*
*				  1a) FRC-RC: As this software is intended to replace IFI's
*				  serial port driver, the call to Serial_Driver_Initialize()
*				  in user_routines.c / User_Initialization() should be 
*				  removed or commented out.	In addition, all references to
*				  "user_Serialdrv.c" and "user_Serialdrv.h" must be removed 
*				  from the project and all project source files.
*
*				  1b) EDU-RC: As this software is intended to replace IFI's
*				  serial port driver, the call to Initialize_Serial_Comms()
*				  in user_routines.c / User_Initialization() should be 
*				  removed or commented out.	In addition, all references to
*				  "printf_lib.c" and "printf_lib.h" must be removed from 
*				  the project and all project source files.
*
*				  2) You must add the serial_ports.c/.h source files to
*				  your MPLAB project.
*
*				  3) A #include statement for the serial_ports.h header
*				  file must be included at the beginning of each source
*				  file that uses the serial ports. The statement should
*				  look like this: #include "serial_ports.h".
*
*				  4) If you intend to use the C18 output stream functions,
*				  A #include statement for the stdio.h header file must be 
*				  included at the beginning of each source file that calls
*				  any of these functions. The statement should look like 
*				  this: #include <serial_ports.h>.
*
*				  5) Init_Serial_Port_One() and/or Init_Serial_Port_Two()
*				  must be called from the User_Initialization() function 
*				  located in the user_routines.c source file.
*
*				  6) The interrupt handler(s) must be installed in the
*				  InterruptHandlerLow() function located in the 
*				  user_routines_fast.c source file. See the accompanying
*				  copy of user_routines_fast.c to see how this is done.
*
*				  7) Decide what functionality you need and comment out the
*				  #define ENABLE_SERIAL_PORT_xxx_yy entries in serial_ports.h
*				  as necessary. As an example, if you only need to send data
*				  using serial port one and would like to reclaim the resources
*				  used by serial port two and serial port one's receiver
*				  source code, the top of the serial_ports.h file would look
*				  like this:
*
*				  // comment out the next line to disable all serial port one
*				  // receive functionality
*				  // #define ENABLE_SERIAL_PORT_ONE_RX
*
*				  // comment out the next line to disable all serial port one
*				  // transmit functionality
*				  #define ENABLE_SERIAL_PORT_ONE_TX
*
*				  // comment out the next line to disable all serial port two
*				  // receive functionality
*				  // #define ENABLE_SERIAL_PORT_TWO_RX
*
*				  // comment out the next line to disable all serial port two
*				  // transmit functionality
*				  // #define ENABLE_SERIAL_PORT_TWO_TX
*
*				  By default, both serial ports and their respective receive
*				  and transmit sections are enabled.
*
*				  8) As the default output device for C18's output stream 
*				  functions is the null device, you'll presumably want to 
*				  change the value of stdout_serial_port to "SERIAL_PORT_ONE" 
*				  or "SERIAL_PORT_TWO" if you want to see printf()'s output.
*				  User_Initialization() is a good place to do this.
*
*				  9) To support terminal emulation software, \r\n should 
*				  be used instead of just \n in the printf() format string.
*
*
*				This serial port driver can send output stream data to either 
*				of the serial ports by setting the value of the global variable
*				stdout_serial_port before calling output stream	functions like
*				printf(). Setting the value	to "SERIAL_PORT_ONE" will send the 
*				output to serial port one. Likewise, setting the value to 
*				"SERIAL_PORT_TWO" will send the output to serial port two. 
*				Setting the value to "NUL" will send the output to the null 
*				device, meaning that the output is sent	nowhere. These values 
*				are #define'd in serial_ports.h. As an example,
*
*				  stdout_serial_port = SERIAL_PORT_ONE;
*				  printf("Kernighan");
*				  stdout_serial_port = NUL;
*				  printf("and");
*				  stdout_serial_port = SERIAL_PORT_TWO;
*				  printf("Ritchie");
*
*				will send the text "Kernighan" to the peripheral device attached
*				to serial port one, the text "Ritchie" to the device attached to 
*				serial port two and the text "and" won't be sent anywhere.
*				By default, output is sent to the null device, which is the only
*				output device guaranteed to be present.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright ©2004-2006 R. Kevin Watson. All rights are reserved.
*		
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2004  0.1  RKW - Original code.
*	28-Dec-2004  0.2  RKW - Using preprocessor directives, added the ability 
*	                  to enable/disable individual serial port receive and 
*	                  transmit code. Updated documentation.
*	05-Feb-2005  0.3  RKW - Added _user_putc() interface to C18's new output
*	                  stream capabilities. Updated documentation.
*	05-Jan-2006  0.4  RKW - Partial port to 18F8722. Updated documentation.
*	10-Jan-2006  0.4  RKW - Modified the #pragma interruptlow line to also
*	                  save the .tmpdata section.
*
*******************************************************************************/
#include <p18f8722.h>
#include <stdio.h>
#include "serial_ports.h"

// by default stdout stream output is sent to the null device, 
// which is the only device guaranteed to be present. 
unsigned char stdout_serial_port = NUL;

//
// Serial Port 1 Receive Variables:
//

#ifdef ENABLE_SERIAL_PORT_ONE_RX

volatile unsigned char Rx_1_Queue[RX_1_QUEUE_SIZE];	// serial port 1's receive circular queue

volatile unsigned char Rx_1_Queue_Full = FALSE;		// flag that indicates that serial port 1's
													// receive circular queue is full and cannot
													// accept any more data

unsigned char Rx_1_Queue_Empty = TRUE;				// flag that indicates that there is no more 
													// data present in serial port 1's receive
													// circular queue

unsigned char Rx_1_Queue_Read_Index = 0;			// read index into serial port 1's receive
													// circular queue
	
volatile unsigned char Rx_1_Queue_Write_Index = 0;	// write index into serial port 1's receive
													// circular queue
	
volatile unsigned char Rx_1_Queue_Byte_Count = 0;	// number of bytes in serial port 1's receive
													// circular queue

volatile unsigned char RX_1_Overrun_Errors = 0;		// number of overrun errors that have occurred
													// in serial port 1's receive circuitry since
													// the last reset

volatile unsigned char RX_1_Framing_Errors = 0;		// number of framing errors that have occurred
													// in serial port 1's receive circuitry since
													// the last reset
#endif

//
// Serial Port 1 Transmit Variables:
//

#ifdef ENABLE_SERIAL_PORT_ONE_TX

volatile unsigned char Tx_1_Queue[TX_1_QUEUE_SIZE];	// serial port 1's transmit circular queue

volatile unsigned char Tx_1_Queue_Full = FALSE;		// flag that indicates that serial port 1's
													// transmit circular queue is full and cannot
													// accept any more data

volatile unsigned char Tx_1_Queue_Empty = TRUE;		// flag that indicates that there is no more 
													// data to send in serial port 1's transmit
													// circular queue

volatile unsigned char Tx_1_Queue_Read_Index = 0;	// read index into serial port 1's transmit
													// circular queue

unsigned char Tx_1_Queue_Write_Index = 0;			// write index into serial port 1's transmit
													// circular queue

volatile unsigned char Tx_1_Queue_Byte_Count = 0;	// number of bytes in serial port 1's transmit
													// circular queue
#endif

//
// Serial Port 2 Receive Variables:
//

#ifdef ENABLE_SERIAL_PORT_TWO_RX

volatile unsigned char Rx_2_Queue[RX_2_QUEUE_SIZE];	// serial port 2's receive circular queue

volatile unsigned char Rx_2_Queue_Full = FALSE;		// flag that indicates that serial port 2's
													// receive circular queue is full and cannot
													// accept any more data

unsigned char Rx_2_Queue_Empty = TRUE;				// flag that indicates that there is no more 
													// data present in serial port 2's receive
													// circular queue
		
unsigned char Rx_2_Queue_Read_Index = 0;			// read index into serial port 2's receive
													// circular queue

volatile unsigned char Rx_2_Queue_Write_Index = 0;	// write index into serial port 2's receive
													// circular queue

volatile unsigned char Rx_2_Queue_Byte_Count = 0;	// number of bytes in serial port 2's receive
													// circular queue

volatile unsigned char RX_2_Overrun_Errors = 0;		// number of overrun errors that have occurred
													// in serial port 2's receive circuitry since
													// the last reset

volatile unsigned char RX_2_Framing_Errors = 0;		// number of framing errors that have occurred
													// in serial port 2's receive circuitry since
													// the last reset
#endif

//
// Serial Port 2 Transmit Variables:
//

#ifdef ENABLE_SERIAL_PORT_TWO_TX

volatile unsigned char Tx_2_Queue[TX_2_QUEUE_SIZE];	// serial port 2's transmit circular queue

volatile unsigned char Tx_2_Queue_Full = FALSE;		// flag that indicates that serial port 2's
													// transmit circular queue is full and cannot
													// accept any more data

volatile unsigned char Tx_2_Queue_Empty = TRUE;		// flag that indicates that there is no more 
													// data to send in serial port 2's transmit
													// circular queue

volatile unsigned char Tx_2_Queue_Read_Index = 0;	// read index into serial port 2's transmit
													// circular queue

unsigned char Tx_2_Queue_Write_Index = 0;			// write index into serial port 2's transmit
													// circular queue

volatile unsigned char Tx_2_Queue_Byte_Count = 0;	// number of bytes in serial port 2's transmit
													// circular queue
#endif

/*******************************************************************************
*
*	FUNCTION:		Init_Serial_Port_One()
*
*	PURPOSE:		Initializes serial port one for asynchronous operation
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		This function must be called before you try to use serial
*					port one.
*
*					By default, this serial port is set to 115200 baud with
*					the transmitter and receiver enabled. This is the rate
*					IFI's terminal program expects.
*
*					The	serial port's baud rate is programmed by entering
*					a value into the SPBRG1 register and possibly changing
*					the value of the BRGH bit. Several example values are 
*					included in the serial_ports.h file.
*
*					Numbers within brackets refer to the PIC18F8722	data
*					sheet page number where more information can be found.
*					This document can be found at microchip's website at
*					http://www.microchip.com or at the author's website at
*					http://www.kevin.org/frc
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_RX or ENABLE_SERIAL_PORT_ONE_TX
*					is #define'd in serial_ports.h
*
*******************************************************************************/
void Init_Serial_Port_One(void)
{
	// Start by initializing the serial port with code 
	// common to receive and transmit functions
	SPBRG1 = BAUD_115200;	// baud rate generator register [251]
							//
	TXSTA1bits.BRGH = 1;	// high baud rate select bit (asynchronous mode only) [248]
							//  0: low speed
							//  1: high speed
							//
	PIE1bits.RC1IE = 0;		// receive interrupt enable bit [127]
							//  0: disables received data interrupt
							//  1: enables received data interrupt
							//
	PIE1bits.TX1IE = 0;		// transmit interrupt enable bit [127]
							//  0: disables transmit register empty interrupt
							//  1: enables transmit register empty interrupt
							//
	TXSTA1bits.SYNC = 0;	// USART mode select bit [248]
							//  0: asynchronous mode
							//  1: synchronous mode
							//
	TXSTA1bits.CSRC = 0;	// clock source select bit (synchronous mode only) [248]
							//  0: Slave mode (clock generated by external source)
							//  1: Master mode (clock generated internally from BRG)

	// if receive functionality is to be included in the
	// software build, include code that is specific to
	// initializing the receiver
	#ifdef ENABLE_SERIAL_PORT_ONE_RX
							//
	TRISCbits.TRISC7 = 1;	// make sure the RC7/RX1/DT1 pin is configured as an input [140]
							//
	RCSTA1bits.RX9 = 0;		// 9-bit receive enable bit [249]
							//  0: 8-bit reception mode
							//  1: 9-bit reception mode
							//
	RCSTA1bits.ADDEN = 0;	// address detect enable bit (9-bit asynchronous mode only) [249]
							//  0: disables address detection
							//  1: enables address detection
							//
	RCSTA1bits.SREN = 1;	// single receive enable bit (master synchronous mode only) [249]
							//  0: disables single receive mode
							//  1: enables single receive mode
							//
	RCSTA1bits.CREN = 1;	// continuous receive mode enable bit [249]
							// asynchronous mode:
							//  0: disables receiver
							//  1: enable receiver
							// synchronous mode:
							//  0: disables continuous receive
							//  1: enables continuous receive until CREN is cleared
							//
	IPR1bits.RC1IP = 0;		// receive interrupt priority bit (must be 0 for IFI controllers) [130]
							//  0: low-priority
							//  1: high-priority
							//
	PIE1bits.RC1IE = 1;		// receive interrupt enable bit [127]
							//  0: disables received data interrupt
							//  1: enables received data interrupt
	#endif					//

	// if transmit functionality is to be included in the
	// software build, include code that is specific to
	// initializing the serial port transmitter
	#ifdef ENABLE_SERIAL_PORT_ONE_TX
							//
	stdout = _H_USER;		// use this driver for output stream functions
							//
	TRISCbits.TRISC6 = 0;	// make sure the RC6/TX1/CK1 pin is configured as an output [140]
							//
	TXSTA1bits.TX9 = 0;		// 9-bit transmit enable bit [248]
							//  0: 8-bit transmission mode
							//  1: 9-bit transmission mode
							//
	TXSTA1bits.SENDB = 0;	// send break character bit [248]
							//  0: Sync Break transmission completed
							//  1: Send Sync Break on next transmission
							//
	IPR1bits.TX1IP = 0;		// transmit interrupt priority bit (must be 0 for IFI controllers) [130]
							//  0: low-priority
							//  1: high-priority
							//
	PIE1bits.TX1IE = 1;		// transmit interrupt enable bit [127]
							//  0: disables transmit register empty interrupt
							//  1: enables transmit register empty interrupt
							//
	TXSTA1bits.TXEN = 1;  	// Enable transmitter [248]
							//  0: serial transmitter is disabled
							//  1: serial transmitter 
	#endif					//

	// finally, turn on the serial port
	RCSTA1bits.SPEN = 1;  	// Serial Port Enable [249]
							//  0: serial port is disabled
							//  1: serial port is enabled
}

/*******************************************************************************
*
*	FUNCTION:		Init_Serial_Port_Two()
*
*	PURPOSE:		Initializes serial port two for asynchronous operation
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		This function must be called before you try to use serial
*					port two.
*
*					By default, this serial port is set to 9600 baud with
*					the transmitter and receiver enabled. 
*
*					The	serial port's baud rate is programmed by entering
*					a value into the SPBRG2 register and possibly changing
*					the value of the BRGH bit. Several example values are 
*					included in the serial_ports.h file.
*
*					Numbers within brackets refer to the PIC18F8722	data
*					sheet page number where more information can be found.
*					This document can be found at microchip's website at
*					http://www.microchip.com or at the author's website at
*					http://www.kevin.org/frc
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_RX or ENABLE_SERIAL_PORT_TWO_TX
*					is #define'd in serial_ports.h
*
*******************************************************************************/
void Init_Serial_Port_Two(void)
{
	// Start by initializing the serial port with code 
	// common to receive and transmit functions
	SPBRG2 = BAUD_115200;	// baud rate generator register [251]
							//
	TXSTA2bits.BRGH = 1;	// high baud rate select bit (asynchronous mode only) [248]
							//  0: low speed
							//  1: high speed
							//
	PIE3bits.RC2IE = 0;		// receive interrupt enable bit [129]
							//  0: disables received data interrupt
							//  1: enables received data interrupt
							//
	PIE3bits.TX2IE = 0;		// transmit interrupt enable bit [129]
							//  0: disables transmit register empty interrupt
							//  1: enables transmit register empty interrupt
							//
	TXSTA2bits.SYNC = 0;	// USART mode select bit [248]
							//  0: asynchronous mode
							//  1: synchronous mode
							//
	TXSTA2bits.CSRC = 0;	// clock source select bit (synchronous mode only) [248]
							//  0: Slave mode (clock generated by external source)
							//  1: Master mode (clock generated internally from BRG)

	// if receive functionality is to be included in the
	// software build, include code that is specific to
	// initializing the receiver
	#ifdef ENABLE_SERIAL_PORT_TWO_RX
							//
	TRISGbits.TRISG2 = 1;	// make sure the RG2/RX2/DT2 pin is configured as an input [151]
							//
	RCSTA2bits.RX9 = 0;		// 9-bit receive enable bit [249]
							//  0: 8-bit reception mode
							//  1: 9-bit reception mode
							//
	RCSTA2bits.ADDEN = 0;	// address detect enable bit (9-bit asynchronous mode only) [249]
							//  0: disables address detection
							//  1: enables address detection
							//
	RCSTA2bits.SREN = 1;	// single receive enable bit (master synchronous mode only) [249]
							//  0: disables single receive mode
							//  1: enables single receive mode
							//
	RCSTA2bits.CREN = 1;	// continuous receive mode enable bit [249]
							// asynchronous mode:
							//  0: disables receiver
							//  1: enable receiver
							// synchronous mode:
							//  0: disables continuous receive
							//  1: enables continuous receive until CREN is cleared
							//
	IPR3bits.RC2IP = 0;		// receive interrupt priority bit (must be 0 for IFI controllers) [132]
							//  0: low-priority
							//  1: high-priority
							//
	PIE3bits.RC2IE = 1;		// receive interrupt enable bit [129]
							//  0: disables received data interrupt
							//  1: enables received data interrupt
	#endif					//

	// if transmit functionality is to be included in the
	// software build, include code that is specific to
	// initializing the serial port transmitter
	#ifdef ENABLE_SERIAL_PORT_TWO_TX
							//
	stdout = _H_USER;		// use this driver for output stream functions
							//
							//
	TRISGbits.TRISG1 = 0;	// make sure the RG1/TX2/CK2 pin is configured as an output [151]
							//
	TXSTA2bits.TX9 = 0;		// 9-bit transmit enable bit [248]
							//  0: 8-bit transmission mode
							//  1: 9-bit transmission mode
							//
	TXSTA2bits.SENDB = 0;	// send break character bit [248]
							//  0: Sync Break transmission completed
							//  1: Send Sync Break on next transmission
							//
	IPR3bits.TX2IP = 0;		// transmit interrupt priority bit (must be 0 for IFI controllers) [132]
							//  0: low-priority
							//  1: high-priority
							//
	PIE3bits.TX2IE = 1;		// transmit interrupt enable bit [129]
							//  0: disables transmit register empty interrupt
							//  1: enables transmit register empty interrupt
							//
	TXSTA2bits.TXEN = 1;  	// Enable transmitter [248]
							//  0: serial transmitter is disabled
							//  1: serial transmitter 
	#endif					//

	// finally, turn on the serial port
	RCSTA2bits.SPEN = 1;  	// Serial Port Enable [249]
							//  0: serial port is disabled
							//  1: serial port is enabled
}

/*******************************************************************************
*
*	FUNCTION:		Serial_Port_One_Byte_Count()
*
*	PURPOSE:		Returns the number of bytes in serial port 
*					ones's received data queue.		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		unsigned char
*
*	COMMENTS:		This function must be called to determine how much data,
*					if any, is present in serial port one's received data
*					queue. If the returned number is greater than zero, then
*					a call to Read_Serial_Port_One() can be made to retrieve
*					the next byte.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_RX is #define'd in serial_ports.h
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_ONE_RX
unsigned char Serial_Port_One_Byte_Count(void)
{
	unsigned char temp;

	// since we're about to use the Rx_1_Queue_Byte_Count variable,
	// which can also be modified in the interrupt service routine,
	// let's briefly disable the serial port interrupt to make sure 
	// that Rx_1_Queue_Byte_Count doesn't get altered while we're 
	// using it.
	PIE1bits.RC1IE = 0;

	// now we can get a local copy of the byte count without fear
	// that we'll get corrupted data
	temp = Rx_1_Queue_Byte_Count;

	// okay, we have a local copy of the byte count, so turn the 
	// serial port interrupt back on.
	PIE1bits.RC1IE = 1;

	// return the byte count
	return(temp);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Serial_Port_Two_Byte_Count()
*
*	PURPOSE:		Returns the number of bytes in serial port 
*					two's received data queue.		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		unsigned char
*
*	COMMENTS:		This function must be called to determine how much data,
*					if any, is present in serial port two's received data
*					queue. If the returned number is greater than zero, then
*					a call to Read_Serial_Port_Two() can be made to retrieve
*					the next byte.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_RX is #define'd in serial_ports.h
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_TWO_RX
unsigned char Serial_Port_Two_Byte_Count(void)
{
	unsigned char temp;

	// since we're about to use the Rx_1_Queue_Byte_Count variable,
	// which can also be modified in the interrupt service routine,
	// let's briefly disable the serial port interrupt to make sure 
	// that Rx_1_Queue_Byte_Count doesn't get altered while we're 
	// using it.
	PIE3bits.RC2IE = 0;

	// now we can get a local copy of the byte count without fear
	// that we'll get corrupted data
	temp = Rx_2_Queue_Byte_Count;

	// okay, we have a local copy of the byte count, so turn the 
	// serial port interrupt back on.
	PIE3bits.RC2IE = 1;

	// return the byte count
	return(temp);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Read_Serial_Port_One()
*
*	PURPOSE:		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		unsigned char
*
*	COMMENTS:		This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_RX is #define'd in serial_ports.h 		
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_ONE_RX
unsigned char Read_Serial_Port_One(void)
{
	unsigned char byte;

	if(Rx_1_Queue_Empty)
	{
		// error: no data to read
		return(0);
	}
	else
	{
		// get a byte from the circular queue and store it temporarily
		byte = Rx_1_Queue[Rx_1_Queue_Read_Index];

		// decrement the queue byte count
		Rx_1_Queue_Byte_Count--;

		// increment the read pointer
		Rx_1_Queue_Read_Index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Rx_1_Queue_Read_Index &= RX_1_QUEUE_INDEX_MASK;

		// since we're about to use the Rx_1_Queue_Write_Index variable, which can
		// also be modified in the interrupt service routine, let's briefly disable
		// the serial port interrupt to make sure that Rx_1_Queue_Write_Index doesn't 
		// get altered while we're using it.
		PIE1bits.RC1IE = 0;

		// is the circular queue now empty?
		if(Rx_1_Queue_Read_Index == Rx_1_Queue_Write_Index)
		{
			Rx_1_Queue_Empty = TRUE;
		}

		// okay, we're done using Rx_1_Queue_Write_Index, so turn the serial port
		// interrupt back on.
		PIE1bits.RC1IE = 1;

 		// Since we've just removed a byte to the queue, it can't possibly be full.
		// Again, this is quicker than using an if() statement every time
		Rx_1_Queue_Full = FALSE;

		// return the data
		return(byte);
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Read_Serial_Port_Two()
*
*	PURPOSE:		
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		unsigned char
*
*	COMMENTS:		This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_RX is #define'd in serial_ports.h 		
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_TWO_RX
unsigned char Read_Serial_Port_Two(void)
{
	unsigned char byte;

	if(Rx_2_Queue_Empty)
	{
		// error: no data to read
		return(0);
	}
	else
	{
		// get a byte from the circular queue and store it temporarily
		byte = Rx_2_Queue[Rx_2_Queue_Read_Index];

		// decrement the queue byte count
		Rx_2_Queue_Byte_Count--;

		// increment the read pointer
		Rx_2_Queue_Read_Index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Rx_2_Queue_Read_Index &= RX_2_QUEUE_INDEX_MASK;

		// since we're about to use the Rx_2_Queue_Write_Index variable, which can
		// also be modified in the interrupt service routine, let's briefly disable
		// the serial port interrupt to make sure that Rx_2_Queue_Write_Index doesn't 
		// get altered while we're using it.
		PIE3bits.RC2IE = 0;

		// is the circular queue now empty?
		if(Rx_2_Queue_Read_Index == Rx_2_Queue_Write_Index)
		{
			Rx_2_Queue_Empty = TRUE;
		}

		// okay, we're done using Rx_2_Queue_Write_Index, so turn the serial port
		// interrupt back on.
		PIE3bits.RC2IE = 1;

 		// Since we've just removed a byte to the queue, it can't possibly be full.
		// Again, this is quicker than using an if() statement every time
		Rx_2_Queue_Full = FALSE;

		// return the data
		return(byte);
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Write_Serial_Port_One()
*
*	PURPOSE:		Sends a byte of data using serial port one.
*
*	CALLED FROM:
*
*	PARAMETERS:		unsigned char
*
*	RETURNS:		nothing
*
*	COMMENTS:		If you don't initialize the serial port before calling
*					this function, the robot controller will stop functioning
*					and you'll get the much dreaded red-light-of-death. This 
*					is because the while() statement below is waiting for the
*					transmit circuitry to send another byte, but if the serial
*					port hasn't been configured, nothing will be transmitted
*					and we'll be stuck in the while() loop.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_TX is #define'd in serial_ports.h
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_ONE_TX
void Write_Serial_Port_One(unsigned char byte)
{
	// if the queue is full, wait here until space is available
	while(Tx_1_Queue_Full);

	// put the byte on the circular queue
	Tx_1_Queue[Tx_1_Queue_Write_Index] = byte;

	// increment the queue byte count
	Tx_1_Queue_Byte_Count++;

	// increment the write pointer
	Tx_1_Queue_Write_Index++;

	// If the index pointer overflowed, cut-off the high-order bit. Doing this
	// every time is quicker than checking for overflow every time with an if()
	// statement and only then occasionally setting it back to zero. For this 
	// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
	Tx_1_Queue_Write_Index &= TX_1_QUEUE_INDEX_MASK;

	// since we're about to use the Tx_1_Queue_Write_Index variable, which can
	// also be modified in the interrupt service routine, let's briefly disable
	// the serial port interrupt to make sure that Tx_1_Queue_Write_Index doesn't 
	// get altered while we're using it.
	PIE1bits.TX1IE = 0;

	// is the circular queue now full?
	if(Tx_1_Queue_Read_Index == Tx_1_Queue_Write_Index)
	{ 
		Tx_1_Queue_Full = TRUE;
	}

	// okay, we're done using Tx_Queue_Write_Index, so turn the serial port
	// interrupt back on.
	PIE1bits.TX1IE = 1;

	// Since we've just added a byte to the queue, it can't possibly be empty.
	// Again, this is quicker than using an if() statement every time
	Tx_1_Queue_Empty = FALSE;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Write_Serial_Port_Two()
*
*	PURPOSE:		Sends a byte of data using serial port two.
*
*	CALLED FROM:
*
*	PARAMETERS:		unsigned char
*
*	RETURNS:		nothing
*
*	COMMENTS:		If you don't initialize the serial port before calling
*					this function, the robot controller will stop functioning
*					and you'll get the much dreaded red-light-of-death. This 
*					is because the while() statement below is waiting for the
*					transmit circuitry to send another byte, but if the serial
*					port hasn't been configured, nothing will be transmitted
*					and we'll be stuck in the while() loop.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_TX is #define'd in serial_ports.h
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_TWO_TX
void Write_Serial_Port_Two(unsigned char byte)
{
	// if the queue is full, wait here until space is available
	while(Tx_2_Queue_Full);

	// put the byte on the circular queue
	Tx_2_Queue[Tx_2_Queue_Write_Index] = byte;

	// increment the queue byte count
	Tx_2_Queue_Byte_Count++;

	// increment the write pointer
	Tx_2_Queue_Write_Index++;

	// If the index pointer overflowed, cut-off the high-order bit. Doing this
	// every time is quicker than checking for overflow every time with an if()
	// statement and only then occasionally setting it back to zero. For this 
	// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
	Tx_2_Queue_Write_Index &= TX_2_QUEUE_INDEX_MASK;

	// since we're about to use the Tx_2_Queue_Write_Index variable, which can
	// also be modified in the interrupt service routine, let's briefly disable
	// the serial port interrupt to make sure that Tx_2_Queue_Write_Index doesn't 
	// get altered while we're using it.
	PIE3bits.TX2IE = 0;

	// is the circular queue now full?
	if(Tx_2_Queue_Read_Index == Tx_2_Queue_Write_Index)
	{ 
		Tx_2_Queue_Full = TRUE;
	}

	// okay, we're done using Tx_2_Queue_Write_Index, so turn the serial port
	// interrupt back on.
	PIE3bits.TX2IE = 1;

	// Since we've just added a byte to the queue, it can't possibly be empty.
	// Again, this is quicker than using an if() statement every time
	Tx_2_Queue_Empty = FALSE;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Rx_1_Int_Handler()
*
*	PURPOSE:		Serial port one new data interrupt handler.
*
*	CALLED FROM:	user_routines_fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		If the interrupt handler was installed correctly, this
*					function will be called every time a new byte of data
*					is received by serial port one.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_RX is #define'd in serial_ports.h		
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_ONE_RX
void Rx_1_Int_Handler(void)
{
	if(Rx_1_Queue_Full)
	{
		// just turn off the serial port interrupt if we can't store any more data.
		// the interrupt will be re-enabled within the Receive_Byte() function when
		// more data is read.
		PIE1bits.RC1IE = 0;
	}
	else
	{
		// put the byte on the circular queue
		Rx_1_Queue[Rx_1_Queue_Write_Index] = RCREG1;

		// if the interrupt handler was disabled while data was being received,
		// data may have backed-up in the receiver circuitry, causing an overrun
		// condition. So let's check the OERR bit to see if this has happened
		// and if it has, we'll need to reset the serial port receiver circuitry
		// to get data flowing again.
		if(RCSTA1bits.OERR)
		{
			// reset by turning off the receiver circuitry, then...
			RCSTA1bits.CREN = 0;
			
			// ...turn it back on
			RCSTA1bits.CREN = 1;

			// indicate that we've had an error
			RX_1_Overrun_Errors++;
		}

		// if incoming data gets misaligned and the receiver doesn't receive a
		// stop bit where it expects to detect it, the receiver circuitry will
		// set the FERR bit to indicate that it's received corrupted data. The
		// likely reason for this is an incorrectly set baud rate on either the
		// receiver or transmitter end.
		if(RCSTA1bits.FERR)
		{
			RX_1_Framing_Errors++;
		}

		// increment the queue byte count
		Rx_1_Queue_Byte_Count++;
	
		// increment the write pointer
		Rx_1_Queue_Write_Index++;
		
		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Rx_1_Queue_Write_Index &= RX_1_QUEUE_INDEX_MASK;
		
		// is the circular queue now full?
		if(Rx_1_Queue_Read_Index == Rx_1_Queue_Write_Index)
		{ 
			Rx_1_Queue_Full = TRUE;
		}
		
		// Since we've just added a byte to the queue, it can't possibly be empty.
		// Again, this is quicker than using an if() statement every time
		Rx_1_Queue_Empty = FALSE;
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Rx_2_Int_Handler()
*
*	PURPOSE:		Serial port two new data interrupt handler.		
*
*	CALLED FROM:	user_routines_fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		If the interrupt handler was installed correctly, this
*					function will be called every time a new byte of data
*					is received by serial port two.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_RX is #define'd in serial_ports.h 		
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_TWO_RX
void Rx_2_Int_Handler(void)
{
	if(Rx_2_Queue_Full)
	{
		// just turn off the serial port interrupt if we can't store any more data.
		// the interrupt will be re-enabled within the Receive_Byte() function when
		// more data is read.
		PIE3bits.RC2IE = 0;
	}
	else
	{
		// put the byte on the circular queue
		Rx_2_Queue[Rx_2_Queue_Write_Index] = RCREG2;

		// if the interrupt handler was disabled while data was being received,
		// data may have backed-up in the receiver circuitry, causing an overrun
		// condition. So let's check the OERR bit to see if this has happened
		// and if it has, we'll need to reset the serial port receiver circuitry
		// to get data flowing again.
		if(RCSTA2bits.OERR)
		{
			// reset by turning off the receiver circuitry, then...
			RCSTA2bits.CREN = 0;
			
			// ...turn it back on
			RCSTA2bits.CREN = 1;

			// indicate that we've had an error
			RX_2_Overrun_Errors++;
		}

		// if incoming data gets misaligned and the receiver doesn't receive a
		// stop bit where it expects to detect it, the receiver circuitry will
		// set the FERR bit to indicate that it's received corrupted data. The
		// likely reason for this is an incorrectly set baud rate on either the
		// receiver or transmitter end.
		if(RCSTA2bits.FERR)
		{
			RX_2_Framing_Errors++;
		}

		// increment the queue byte count
		Rx_2_Queue_Byte_Count++;
	
		// increment the write pointer
		Rx_2_Queue_Write_Index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Rx_2_Queue_Write_Index &= RX_2_QUEUE_INDEX_MASK;
		
		// is the circular queue now full?
		if(Rx_2_Queue_Read_Index == Rx_2_Queue_Write_Index)
		{ 
			Rx_2_Queue_Full = TRUE;
		}
		
		// Since we've just added a byte to the queue, it can't possibly be empty.
		// Again, this is quicker than using an if() statement every time
		Rx_2_Queue_Empty = FALSE;
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Tx_1_Int_Handler()
*
*	PURPOSE:		Serial port one empty transmit buffer interrupt handler.
*
*	CALLED FROM:	user_routines_fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		If the interrupt handler was installed correctly, this
*					function will be called every time serial port one is
*					ready to start sending a byte of data.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_ONE_TX is #define'd in serial_ports.h 		
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_ONE_TX
void Tx_1_Int_Handler(void)
{
	if(Tx_1_Queue_Empty)
	{
		// just turn off the serial port interrupt if we don't have data to send.
		// the interrupt will be re-enabled within the Send_Byte() function when
		// more data is sent.
		PIE1bits.TX1IE = 0;
	}
	else
	{
		// get a byte from the circular queue and send it to the USART
		TXREG1 = Tx_1_Queue[Tx_1_Queue_Read_Index];

		// decrement the queue byte count
		Tx_1_Queue_Byte_Count--;

		// increment the read pointer
		Tx_1_Queue_Read_Index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Tx_1_Queue_Read_Index &= TX_1_QUEUE_INDEX_MASK;

		// is the circular queue now empty?
		if(Tx_1_Queue_Read_Index == Tx_1_Queue_Write_Index)
		{
			Tx_1_Queue_Empty = TRUE;
		}

 		// Since we've just removed a byte from the queue, it can't possibly be full.
		// Again, this is quicker than using an if() statement every time
		Tx_1_Queue_Full = FALSE;
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Tx_2_Int_Handler()
*
*	PURPOSE:		Serial port two empty transmit buffer interrupt handler.		
*
*	CALLED FROM:	user_routines_fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		If the interrupt handler was installed correctly, this
*					function will be called every time serial port two is
*					ready to start sending a byte of data.
*
*					This function will not be included in the build unless
*					ENABLE_SERIAL_PORT_TWO_TX is #define'd in serial_ports.h 			
*
*******************************************************************************/
#ifdef ENABLE_SERIAL_PORT_TWO_TX
void Tx_2_Int_Handler(void)
{
	if(Tx_2_Queue_Empty)
	{
		// just turn off the serial port interrupt if we don't have data to send.
		// the interrupt will be re-enabled within the Send_Byte() function when
		// more data is sent.
		PIE3bits.TX2IE = 0;
	}
	else
	{
		// get a byte from the circular queue and send it to the USART
		TXREG2 = Tx_2_Queue[Tx_2_Queue_Read_Index];

		// decrement the queue byte count
		Tx_2_Queue_Byte_Count--;

		// increment the read pointer
		Tx_2_Queue_Read_Index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		Tx_2_Queue_Read_Index &= TX_2_QUEUE_INDEX_MASK;

		// is the circular queue now empty?
		if(Tx_2_Queue_Read_Index == Tx_2_Queue_Write_Index)
		{
			Tx_2_Queue_Empty = TRUE;
		}

 		// Since we've just removed a byte from the queue, it can't possibly be full.
		// Again, this is quicker than using an if() statement every time
		Tx_2_Queue_Full = FALSE;
	}
}
#endif

/*******************************************************************************
*
*	FUNCTION:		_user_putc()
*
*	PURPOSE:		putc() interface to C18 2.4 output stream functions		
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:			
*
*******************************************************************************/
void _user_putc(unsigned char byte)
{
	if(stdout_serial_port == NUL)
	{
		// send the data to the bit bucket
	}
	#ifdef ENABLE_SERIAL_PORT_ONE_TX
	else if(stdout_serial_port == SERIAL_PORT_ONE)
	{
		// send the data to serial port one
		Write_Serial_Port_One(byte);
	}
	#endif
	#ifdef ENABLE_SERIAL_PORT_TWO_TX
	else if(stdout_serial_port == SERIAL_PORT_TWO)
	{
		// send the data to serial port two
		Write_Serial_Port_Two(byte);
	}
	#endif
}
