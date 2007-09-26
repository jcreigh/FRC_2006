/*******************************************************************************
* FILE NAME: main.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the main program loop.
*
* USAGE:
*  You should not need to modify this file.
*  Note the different loop speed for the two routines:
*     Process_Data_From_Master_uP
*     Process_Data_From_Local_IO
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"

tx_data_record txdata;          /* DO NOT CHANGE! */
rx_data_record rxdata;          /* DO NOT CHANGE! */
packed_struct statusflag;       /* DO NOT CHANGE! */

/*******************************************************************************
* FUNCTION NAME: main
* PURPOSE:       Main program function.
* CALLED FROM:   ifi_startup.c
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT DELETE THIS FUNCTION 
*******************************************************************************/
void main (void)
{
#ifdef UNCHANGEABLE_DEFINITION_AREA
  IFI_Initialization ();        /* DO NOT CHANGE! */
#endif

  User_Initialization();        /* You edit this in user_routines.c */

  statusflag.NEW_SPI_DATA = 0;  /* DO NOT CHANGE! */ 

  while (1)   /* This loop will repeat indefinitely. */
  {
#ifdef _SIMULATOR
    statusflag.NEW_SPI_DATA = 1;
#endif

    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {                                 /* I'm slow!  I only execute every 26.2ms because */
                                      /* that's how fast the Master uP gives me data. */
      Process_Data_From_Master_uP();  /* You edit this in user_routines.c */

      if (autonomous_mode)            /* DO NOT CHANGE! */
      {
        User_Autonomous_Code();        /* You edit this in user_routines_fast.c */
      }
    }
    Process_Data_From_Local_IO();     /* You edit this in user_routines_fast.c */
                                      /* I'm fast!  I execute during every loop.*/
  } /* while (1) */
}  /* END of Main */


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
