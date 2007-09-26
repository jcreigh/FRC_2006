/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
// #include "user_Serialdrv.h"


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow ()     
{
	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}



//  ***  IFI Code Starts Here***
//                              
//  unsigned char int_byte;       
//  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
//  { 
//    INTCON3bits.INT2IF = 0;
//  }
//  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
//  {
//    INTCON3bits.INT3IF = 0;
//  }
//  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
//  {
//    int_byte = PORTB;          /* You must read or write to PORTB */
//    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
//  }                                        /*     to clear the interrupt condition.  */
//  else
//  { 
//    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
//  }
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
      static int shooterPosition = 0;  // 0 is down, 1 is up
	static int shooterGoingDown = 0;
	static int shooterGoingUp = 0;
	static int shooterGoingUpCount = 0;
	static int shooterGoingDownCount = 0;
	
	static int readyToShoot = 0;
	static int letMyAimBeTrue = 0;
	static int inRange = 0;
	
	int shooterOverride = 0;
	
	static int loopCount = 0;
	static int autoCount = 0;
	
	int shooterButton = 0;
	static int buttonCount = 0;
	static int shooterRunning = 0;
	static int shooterLoop = 0;
	static int firstPush = 0;
	static int firstLetGo = 0;
	
	int initialDelay = 0;
	int firstDrive = 500;
	int firstTurn = 620;
	int secondDrive = 1300;
	int secondTurn = 0;
	int thirdDrive = 0;
  
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  pwm11 = 26;
  pwm12 = 254 - 26;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;
  
  

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */

        /* Add your own autonomous code here. */
    pwm05 = pwm06 = 254;

			
	if ((shooterButton == 1) & (shooterRunning == 0))
	{
		shooterRunning = 1;
		shooterLoop = 0;
		buttonCount = 0;
	}
	if (shooterRunning == 1)
		buttonCount++;
		
	if ((buttonCount < 30) & (shooterRunning == 1))
	{
		shooterOverride = 1;
	}
	else if ((buttonCount < 50) & (shooterRunning == 1))
	{
		shooterOverride = 0;
	}
	else if ((buttonCount >= 65) & (shooterRunning == 1))
	{
		relay2_fwd = 0;
		relay2_rev = 0;
		buttonCount = 0;
		shooterRunning = 0;
	}
		
	if (shooterOverride == 1)
	{
		firstLetGo = 1;
		if (firstPush == 1)
		{
			loopCount = 0;
			firstPush = 0;
		}
		
	if (loopCount < 45)
		{
			relay2_fwd = 0;
			relay2_rev = 1;
			loopCount++;
		}
		else
		{
			relay2_fwd = 0;
			relay2_rev = 0;
		}
	}
	else if (shooterOverride == 0)
	{
		if (firstLetGo == 1)
		{
			loopCount = 0;
			firstLetGo = 0;
		}
		loopCount++;
		firstPush = 1;
		if (loopCount < 50)
		{
			relay2_fwd = 1;
			relay2_rev = 0;
		}
		else
		{
			relay2_fwd = 0;
			relay2_rev = 0;
		}
		
	}
	
	

        Camera_Handler();
        
        letMyAimBeTrue = Servo_Track(pwm01, pwm03);
        
        if ((shooterGoingUp != 1) && (shooterGoingDown != 1) && (shooterPosition == 0) && (shooterOverride != 1) && (letMyAimBeTrue == 1))
			readyToShoot = 1;
	
		if ((letMyAimBeTrue > 13) && (letMyAimBeTrue < 40))
			inRange = 1;
		
		if ((letMyAimBeTrue != 1) && (readyToShoot == 1) )
		{
			readyToShoot = 0;			
		}
/*		
		// Motor Control
		// Go backward
		if (loopCount < 100)
		{
			pwm03 = pwm04 = 0;
			pwm01 = pwm03 = 254;
		}
		else if (loopCount == 100)
			pwm01 = pwm02 = pwm03 = pwm04 = 127;
			
		// Left turn
		else if (loopCount < 150)
		{
			pwm03 = pwm04 = 254;
			pwm01 = pwm02 = 254;	
		}
		// Go forward
		else if (loopCount == 150)
			pwm01 = pwm02 = pwm03 = pwm04 = 127;
			
		else if (loopCount < 240)
		{
			pwm01 = pwm02 = 254;
			pwm03 = pwm04 = 0;
		}
		// Stop
		else
			pwm01 = pwm02 = pwm03 = pwm04 = 127;
			
*/

	if (autoCount < initialDelay)
		pwm03 = pwm04 = pwm02 = pwm01 = 127;
		
	else if (autoCount < firstDrive)
	{
		pwm03 = pwm04 = 0;
		pwm01 = pwm02 = 254;
	}
	else if (autoCount < firstDrive + 5)
		pwm03 = pwm04 = pwm02 = pwm01 = 127;

	else if (autoCount < firstTurn)
	{
		pwm03 = pwm04 = 254;
		pwm01 = pwm02 = 254;
	}
	else if (autoCount < firstTurn + 5)
		pwm03 = pwm04 = pwm02 = pwm01 = 127;		
	else if (autoCount < secondDrive)
	{
		pwm03 = pwm04 = 0;
		pwm01 = pwm02 = 254;
	}
	else
		pwm03 = pwm04 = pwm01 = pwm02 = 127;
		
	if ((autoCount > 600) & (autoCount % 150 < 10))
		shooterButton = 1;
/*		

		if (loopCount > 200)
		{
			if (loopCount < 210)
				shooterOverride = 1;
			else if (loopCount < 300)
				shooterOverride = 0;	
			else if (loopCount < 330)
				shooterOverride = 1;
			else if (loopCount < 390)
				shooterOverride = 0;
				
					else if (loopCount < 430)
				shooterOverride = 1;
			else if (loopCount < 490)
				shooterOverride = 0;
				
					else if (loopCount < 530)
				shooterOverride = 1;
			else if (loopCount < 590)
				shooterOverride = 0;
				
			else if (loopCount < 630)
				shooterOverride = 1;
			else if (loopCount < 690)
				shooterOverride = 0;
		
			else if (loopCount < 730)
				shooterOverride = 1;
			else if (loopCount < 790)
				shooterOverride = 0;
				
			else if (loopCount < 830)
				shooterOverride = 1;
			else if (loopCount < 890)
				shooterOverride = 0;
			
			else if (loopCount < 930)
				shooterOverride = 1;
			else if (loopCount < 990)
				shooterOverride = 0;
				
			else if (loopCount < 1030)
				shooterOverride = 1;
			else if (loopCount < 1090)
				shooterOverride = 0;
				
*/			
			
			
		}

        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
		// ========================
		// HOOD ANGLE
		//=========================
		
	if (pwm10 < 10)
	{
		pwm11 = 30;
		pwm12 = 254 - pwm11;
	}
	else if (pwm10 < 15)
	{
		pwm11 = 39;
		pwm12 = 254 - pwm11;
	}
	else if (pwm10 < 23)
	{
		pwm11 = 32;
		pwm12 = 254 - pwm11;
	}
	else if (pwm10 < 55)
	{
		pwm11 = 26;
		pwm12 = 254 - pwm11;
	}	
	else if (pwm10 < 62)
	{
		pwm11 = 27;
		pwm12 = 254 - pwm11;
	}	
	else if (pwm10 < 65)
	{
		pwm11 = 30;
		pwm12 = 254 - pwm11;
	}	
	else 
	{
		pwm11 = 26;
		pwm12 = 254 - pwm11;
	}	
	
        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
        autoCount++;
    
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */
/*	if (p1_sw_top == 1)
	{
		pwm05 = pwm06 = 0; // Rotate Left (clockwise)
		printf("Override clockwise\n");
	}
	else if (p1_sw_aux1 == 1)
	{
		pwm05 = pwm06 = 254; // Rotate Right (counterclockwise)
		printf("Override counterclockwise\n");
	}
	*/
}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
