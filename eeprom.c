/*******************************************************************************
*
*	TITLE:		eeprom.c 
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		10-Jan-2006
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2005-2006 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	08-Oct-2005  0.1  RKW Original
*	18-Oct-2005  0.2  RKW - Modified code to globally disable interrupts
*	                  only during the pre-write sequence.
*	10-Jan-2006  0.2  RKW - Verified code works with PIC18F8722.
*
*******************************************************************************/

//#include "ifi_picdefs.h"
#include <p18f8722.h>
#include "eeprom.h"

unsigned char eeprom_queue_count = 0;
unsigned char eeprom_queue_full = FALSE;
unsigned char eeprom_queue_empty = TRUE;
unsigned char eeprom_queue_read_index = 0;
unsigned char eeprom_queue_write_index = 0;
unsigned char eeprom_queue_data[EEPROM_QUEUE_SIZE];
unsigned int eeprom_queue_address[EEPROM_QUEUE_SIZE];

/*******************************************************************************
*
*	FUNCTION:		EEPROM_Read()
*
*	PURPOSE:		Reads a byte of data from EEPROM.
*
*	CALLED FROM:
*
*	PARAMETERS:		Unsigned int containing the address.
*
*	RETURNS:		Unsigned char containing the data.
*
*	COMMENTS:
*
*******************************************************************************/
unsigned char EEPROM_Read(unsigned int address)
{
	// EEPROM operation (as opposed to a flash memory operation)
    EECON1bits.EEPGD = 0; 

	// EEPROM Address
    EEADR = LOBYTE(address);
	EEADRH = HIBYTE(address);

    // execute the read
	EECON1bits.RD = 1;

	// return the data    
    return(EEDATA);
}    

/*******************************************************************************
*
*	FUNCTION:		EEPROM_Write()
*
*	PURPOSE:		Places address and data on the EEPROM write queue.
*
*	CALLED FROM:
*
*	PARAMETERS:		Unsigned int containing the address.
*					Unsigned char containing the data to write.
*
*	RETURNS:		Unsigned char containing 1 if successful, 0 if buffer full.
*
*	COMMENTS:
*
*******************************************************************************/
unsigned char EEPROM_Write(unsigned int address, unsigned char data)
{
	unsigned char return_value;

	// return error flag if the queue is full
	if(eeprom_queue_full == FALSE)
	{
		// put the byte and its address on their respective circular queues
		eeprom_queue_data[eeprom_queue_write_index] = data;
		eeprom_queue_address[eeprom_queue_write_index] = address;
	
		// increment the queue byte count
		eeprom_queue_count++;
	
		// increment the write pointer
		eeprom_queue_write_index++;
	
		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		eeprom_queue_write_index &= EEPROM_QUEUE_INDEX_MASK;
	
		// is the circular queue now full?
		if(eeprom_queue_read_index == eeprom_queue_write_index)
		{ 
			eeprom_queue_full = TRUE;
		}
	
		// Since we've just added a byte to the queue, it can't possibly be empty.
		// Again, this is quicker than using an if() statement every time
		eeprom_queue_empty = FALSE;

		return_value = 1;
	}
	else
	{
		return_value = 0;
	}

	return(return_value);
}

/*******************************************************************************
*
*	FUNCTION:		EEPROM_Write_Handler()
*
*	PURPOSE:		Writes data to EEPROM
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		nothing
*
*	COMMENTS:		If data is present in the EEPROM write queue when this
*					function is called, it will take about two milliseconds
*					to execute the EEPROM write.
*
*******************************************************************************/
void EEPROM_Write_Handler(void)
{
	unsigned char return_value;
    unsigned char temp_GIEH;
	unsigned char temp_GIEL;

	// check to see if we have data to write
	if(eeprom_queue_empty == FALSE)
	{
		// save the state of the interrupt enable bits
	    temp_GIEH = INTCONbits.GIEH;
		temp_GIEL = INTCONbits.GIEL;
	
		// EEPROM operation (as opposed to a flash memory operation)
	    EECON1bits.EEPGD = 0;
	
		// Only do a write
	    EECON1bits.FREE = 0;  
	
	    // make sure the EEPROM write done flag is reset
		PIR2bits.EEIF = 0;
	
		// set EEPROM address
	    EEADR = LOBYTE(eeprom_queue_address[eeprom_queue_read_index]);
		EEADRH = HIBYTE(eeprom_queue_address[eeprom_queue_read_index]);
	
		// set EEPROM data to write
	    EEDATA = eeprom_queue_data[eeprom_queue_read_index];

		// enable EEPROM writes
	    EECON1bits.WREN = 1;
	
	    // disable Interrupts
		INTCONbits.GIEH = 0;
		INTCONbits.GIEL = 0;
	    
	    // pre-write sequence
	    EECON2 = 0x55;
	    EECON2 = 0xAA;
	    
		// execute the write
		EECON1bits.WR = 1;

		// set GIEH to its original state
	    INTCONbits.GIEH = temp_GIEH;
	
		// set GIEL to its original state
	    INTCONbits.GIEL = temp_GIEL;

		// decrement the write queue count
		eeprom_queue_count--;

		// increment the read pointer
		eeprom_queue_read_index++;

		// If the index pointer overflowed, cut-off the high-order bit. Doing this
		// every time is quicker than checking for overflow every time with an if()
		// statement and only then occasionally setting it back to zero. For this 
		// to work, the queue size must be a power of 2 (e.g., 16,32,64,128...).
		eeprom_queue_read_index &= EEPROM_QUEUE_INDEX_MASK;

		// is the circular queue now empty?
		if(eeprom_queue_read_index == eeprom_queue_write_index)
		{
			eeprom_queue_empty = TRUE;
		}

 		// Since we've just removed a byte from the queue, it can't possibly be full.
		// Again, this is quicker than using an if() statement every time
		eeprom_queue_full = FALSE;
	
	    // wait for the write to complete
		while(PIR2bits.EEIF == 0);
	
		// clear the write completion interrupt flag
	    PIR2bits.EEIF = 0;
	
	    // disable EEPROM writes
		EECON1bits.WREN = 0;
	}
}

/*******************************************************************************
*
*	FUNCTION:		EEPROM_Queue_Free_Space()
*
*	PURPOSE:		Returns the number of free slots in the EEPROM queue.
*
*	CALLED FROM:
*
*	PARAMETERS:		none
*
*	RETURNS:		Unsigned char containing the number of free slots
*					in the EEPROM write buffer.
*
*	COMMENTS:		This function should be called to determine if enough
*					space is available before you call EEPROM_Write().
*
*******************************************************************************/
unsigned char EEPROM_Queue_Free_Space(void)
{
	return(EEPROM_QUEUE_SIZE - eeprom_queue_count);
}
