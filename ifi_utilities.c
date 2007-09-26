/*******************************************************************************
* FILE NAME: ifi_utilities.c
*
* DESCRIPTION:
*  This file contains some useful functions that you can call in your program.
*
* USAGE:
*  The user should NOT modify this file, so that if a new version is released
*  by Innovation First then it can be easily replaced.
*  The user should add their own functions to either user_routines.c or another
*  custom file.
*
*******************************************************************************/

#include <usart.h>
#include <spi.h>
#include <adc.h>
#include <capture.h>
#include <timers.h>
#include <string.h>
#include <pwm.h>
#include "delays.h"       /*defined locally*/
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"

int             ifi_packet_num1 = 0;
int             ifi_last_packet1 = 0;
unsigned char   *ptr;
unsigned char   ifi_count;
unsigned char   ifi_analog_channels;


/*******************************************************************************
* FUNCTION NAME: Wait4TXEmpty
* PURPOSE:       Wait for serial transmit buffer to be empty.
* CALLED FROM:   anywhere
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
/* Used when transmitting data serially.  It waits for each byte to finish.   */
void Wait4TXEmpty(void)
{
#ifndef _SIMULATOR
  while (!TXINTF)
  {
    continue;
  }
#endif
}


/*******************************************************************************
* FUNCTION NAME: PrintByte
* PURPOSE:       A simple way to print a byte of data to the serial port.
* CALLED FROM:   anywhere
* ARGUMENTS:     none
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     odata          unsigned char    I    byte of data to be transmitted
* RETURNS:       void
*******************************************************************************/
void PrintByte(unsigned char odata)
{
  Hex_output((unsigned char) odata);
  TXREG = 13;  /* a carriage return */
  Wait4TXEmpty();
}


/*******************************************************************************
* FUNCTION NAME: PrintWord
* PURPOSE:       A simple way to print a word of data to the serial port.
* CALLED FROM:   anywhere
* ARGUMENTS:     none
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     odata          unsigned int     I    word of data to be transmitted
* RETURNS:       void
*******************************************************************************/
void PrintWord(unsigned int odata)
{
  ptr = (unsigned char *) &odata;
  Hex_output(ptr[1]);
  Hex_output(ptr[0]);
  TXREG = 13;  /* add a carriage return */
  Wait4TXEmpty();
}

/*******************************************************************************
* FUNCTION NAME: DisplayBufr
* PURPOSE:       Print the entire transmit or receive buffer over the serial 
*                port for viewing in a terminal program on your PC.
* CALLED FROM:   anywhere
* ARGUMENTS:     
*     Argument       Type        IO   Description
*     --------       --------    --   -----------
*     *bufr          pointer     I    points to beginning of buffer to transmit
* RETURNS:       void
*******************************************************************************/
void DisplayBufr(unsigned char *bufr)
{   
  for (ifi_count=0;ifi_count<26;ifi_count++)
  {
    Hex_output((unsigned char) *bufr++);
  }
  TXREG = 13;  /* add a carriage return */
  Wait4TXEmpty();
}


/*******************************************************************************
* FUNCTION NAME: PacketNum_Check
* PURPOSE:       Print the packet number over the serial port if a packet gets
*                dropped.  Handy for seeing if you are dropping data.
* CALLED FROM:   anywhere
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void PacketNum_Check(void)
{
  /*    to print only the 10th (packet num) byte*/
  ptr = (unsigned char *) &rxdata.packet_num;
  ifi_packet_num1 = (int) rxdata.packet_num;
  if (ifi_packet_num1 != ifi_last_packet1)
  {
    if (statusflag.FIRST_TIME == 1)
    {
      statusflag.FIRST_TIME = 0;
    }
    else
    {
      Hex_output((unsigned char) ifi_last_packet1);
      Hex_output((unsigned char) ifi_packet_num1);
      TXREG = 13;
      Wait4TXEmpty();
      
      statusflag.FIRST_TIME = 0;
    }
    ifi_last_packet1 = ifi_packet_num1;
  }/*   if (ifi_packet_num1 != ifi_last_packet1)*/
  ifi_last_packet1++;
  if (ifi_last_packet1 > 255)
  { 
    ifi_last_packet1 = 0;
  }
}


/*******************************************************************************
* FUNCTION NAME: Initialize_Serial_Comms
* PURPOSE:       Opens the serial port 1 for communicating with your PC at
*                115k baud, 8 bits, no parity, one stop bit, and no flow control.
* CALLED FROM:   user_routines.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Initialize_Serial_Comms (void)
{
  OpenUSART(USART_TX_INT_OFF &
    USART_RX_INT_OFF &
    USART_ASYNCH_MODE &
    USART_EIGHT_BIT &
    USART_CONT_RX &
    USART_BRGH_HIGH,
    baud_115);   

  Delay1KTCYx( 50 ); /* Settling time */
}


/*******************************************************************************
* FUNCTION NAME: Set_Number_of_Analog_Channels
* PURPOSE:       Sets the variable used in Get_Analog_Value routine (below)
*                to the number of analog channels desired by the user.
* CALLED FROM:   user_routines.c initialization, typically
* ARGUMENTS:     
*      Argument            Type    IO   Description
*     -----------          -----   --   -----------
*     number_of_channels   alias   I    choose alias from ifi_aliases.h
* RETURNS:       void
*******************************************************************************/
void Set_Number_of_Analog_Channels (unsigned char number_of_channels)
{
  ifi_analog_channels = number_of_channels;
}


/*******************************************************************************
* FUNCTION NAME: Get_Analog_Value
* PURPOSE:       Reads the analog voltage on an A/D port and returns the
*                10-bit value read stored in an unsigned int.
* CALLED FROM:   user_routines.c, typically
* ARGUMENTS:     
*      Argument         Type        IO   Description
*     -----------   -------------   --   -----------
*     ADC_channel       alias       I    alias found in ifi_aliases.h
* RETURNS:       unsigned int
*******************************************************************************/
unsigned int Get_Analog_Value (unsigned char ADC_channel)
{
  unsigned int result;

#if defined(__18F8722)
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_0_TAD,
           ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,15);
#else
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ifi_analog_channels,
          ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS );
#endif
  Delay10TCYx( 10 );
  ConvertADC();
  while( BusyADC() );
  ReadADC();
  CloseADC();
  result = (int) ADRESH << 8 | ADRESL;
  return result;
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
