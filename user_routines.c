/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "camera_menu.h"
#include "tracking.h"
#include "tracking_menu.h"
#include "eeprom.h"
#include "terminal.h"
#include <math.h>


extern unsigned char aBreakerWasTripped;

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}




/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
 
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  pwm11 = 32 + 120;
  pwm12 = 254 - (32 + 120);


/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
 
  Init_Serial_Port_One();
  Init_Serial_Port_Two();


			
#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

  Putdata(&txdata);            /* DO NOT CHANGE! */

//  ***  IFI Code Starts Here***
//
//  Serial_Driver_Initialize();
//
//  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	static unsigned char count = 0;
	static unsigned char camera_menu_active = 0;
	static unsigned char tracking_menu_active = 0;
	unsigned char terminal_char;
	unsigned char returned_value;

	static int barf = 0;
	static int runBarf = 0;
	static int barfForward = 1;
	static int barfCount = 1;

	static int firstLock = 0;
	static int raise = 0;
	static int lower = 0;

	static int shooterPosition = 0;  // 0 is down, 1 is up
	static int shooterGoingDown = 0;
	static int shooterGoingUp = 0;
	static int shooterGoingUpCount = 0;
	static int shooterGoingDownCount = 0;
	
	static int readyToShoot = 0;
	static int letMyAimBeTrue = 0;
	static int inRange = 0;
	
	int shooterOverride = 0;
	int beltStop = 0;
	int emergencyStop = 0;
	int turretOverride = 0;
	int turretLeft = 0;
	int turretRight = 0;
	
	static int loopCount = 0;
	static int firstPush = 0;
	static int firstLetGo = 0;
	
	int shooterButton = 0;
	static int buttonCount = 0;
	static int shooterRunning = 0;
	static int shooterLoop = 0;
	
	

	// Start at 10
	const int hoodAngle[] = {96, 96, 96, 96, 32, 30, 28, 27, 26, 26,
							 26, 26, 26, 26, 26, 26, 26, 26, 32, 32,
							 27, 28, 28, 28, 29, 30, 30, 31, 31, 33, };


	
	Getdata(&rxdata);
	

	
	barf = p4_sw_aux1;
	shooterButton = p4_sw_aux1;
	emergencyStop = p4_sw_aux2;
	turretLeft = p4_sw_trig;
	turretRight = p4_sw_top;

	if ((shooterButton == 1) & (shooterRunning == 0))
	{
		shooterRunning = 1;
		shooterLoop = 0;
		buttonCount = 0;
	}
	if (shooterRunning == 1)
		buttonCount++;
		
	if ((buttonCount < 18) & (shooterRunning == 1))
	{
		shooterOverride = 1;
	}
	else if ((buttonCount < 38) & (shooterRunning == 1))
	{
		shooterOverride = 0;
	}
	else if ((buttonCount >= 38) & (shooterRunning == 1))
	{
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
		
	if (loopCount < 13)
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
		if (loopCount < 18)
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
	
	//pwm09 = 127;
	
	
	// send diagnostic information to the terminal, but don't 
	// overwrite the camera or tracking menu if it's active
	if(camera_menu_active == 0 && tracking_menu_active == 0)
	{
		Tracking_Info_Terminal();
	}

	// This function is responsable for camera initialization 
	// and camera serial data interpretation. Once the camera
	// is initialized and starts sending tracking data, this 
	// function will continuously update the global T_Packet_Data 
	// structure with the received tracking information.
	Camera_Handler();

	
	// This function reads data placed in the T_Packet_Data
	// structure by the Camera_Handler() function and if new
	// tracking data is available, attempts to keep the center
	// of the tracked object in the center of the camera's
	// image using two servos that drive a pan/tilt platform.
	// If the camera doesn't have the object within it's field 
	// of view, this function will execute a search algorithm 
	// in an attempt to find the object.

	if(tracking_menu_active == 0)
	{
		letMyAimBeTrue = Servo_Track(pwm01, pwm03);
	}
	
	if (letMyAimBeTrue == 300)
		pwm09 = 127;


	// this logic guarantees that only one of the menus can be
	// active at any giiven time
	if(camera_menu_active == 1)
	{
		// This function manages the camera menu functionality,
		// which is used to enter camera initialization and
		// color tracking parameters.
		camera_menu_active = Camera_Menu();
	}
	else if(tracking_menu_active == 1)
	{
		// This function manages the tracking menu functionality,
		// which is used to enter parameters that describe how
		// the pan and tilt servos will behave while in searching
		// and tracking modes.
		tracking_menu_active = Tracking_Menu();
	}
	else
	{
		// has the user sent any data via the terminal?
		terminal_char = Read_Terminal_Serial_Port();
		// check to see if any "hotkeys" have been pressed
		if(terminal_char == CM_SETUP_KEY)
		{
			camera_menu_active = 1;
		}
		else if(terminal_char == TM_SETUP_KEY)
		{
			tracking_menu_active = 1;
		}
	}

	// This funtion is used by the functions Camera_Menu() and
	// Tracking_Menu() to manage the writing of initialization
	// parameters to your robot controller's non-volatile
	// Electrically Erasable Programmable Read-Only Memory
	// (EEPROM)
	EEPROM_Write_Handler();
/*	
	// Check if it is first lock
		if ((letMyAimBeTrue == 1) && (firstLock == 0))
		{
			firstLock = 1;
			// Lower shooter
			relay2_rev = 0;
			relay2_fwd = 1;
			shooterMoving = 1;			
		}
*/		
	// If Camera is Aimed and shooter is not moving, Shoot
	if ((shooterGoingUp != 1) && (shooterGoingDown != 1) && (shooterPosition == 0) && (shooterOverride != 1) && (letMyAimBeTrue == 1))
		readyToShoot = 1;
	
	if ((letMyAimBeTrue > 13) && (letMyAimBeTrue < 40))
		inRange = 1;
		
	if ((letMyAimBeTrue != 1) && (readyToShoot == 1) && (inRange == 1))
	{
		/* AUTOMATIC SHOOTING
		// Start raising
		relay2_fwd = 0;
		relay2_rev = 1;
		
		shooterGoingUp = 1;
		readyToShoot = 0;
		*/
		
		// MANUAL SHOOTING
		Pwm1_green = 1;
		Pwm1_red = 0;
		Pwm2_green = 1;
		Pwm2_red = 0;
		
					
	}
	else if (letMyAimBeTrue == 1)
	{
		Pwm1_green = 1;
		Pwm1_red = 0;
		Pwm2_green = 0;
		Pwm2_red = 1;
	}
	else
	{
		Pwm1_green = 0;
		Pwm1_red = 1;
		Pwm2_green = 0;
		Pwm2_red = 1;
		
	}		
/*		
	// START BARFER CODE.  Connect barfer to relay1.  Set barf to true to begin barfing.  Drives motor forward for 1.25 seconds.  
	// Returns barfer position for 1.25 seconds
	// Sets bool barf to false.
	relay1_rev = 1;
	relay1_fwd = 0;
	
	if (runBarf == 0)
	{
		if (barf == 1)
			runBarf = 1;
	}
	if (runBarf == 1)
	{
		//printf("Barf Forward: ");
		if (barfCount <= 81) // Run 80 times
		{
				
			relay1_rev = 0;
			relay1_fwd = 1;

			barfCount++;
		}
			else
				runBarf = 0;
	}	
	
	printf("\n\nBarfer motor forward %d", relay1_fwd);
	printf("\nBarfer Backward: %d", relay1_rev);
	
	
	// END BARF CODE
*/

	// Barfer code
	
	relay1_fwd = 0;
	relay1_rev = 1;
	
	if (barf == 1)
	{
		relay1_fwd = 1;
		relay1_rev = 0;
	}
		
	// Ball Launcher

/*
	printf("\n\nForward %d", relay2_fwd);
	printf("\nReverse %d", relay2_rev);
	printf("\nShooter position: %d", shooterPosition);
	printf("\nShooter Going Up: %d", shooterGoingUp);
	printf("\nShooter Going Down: %d", shooterGoingDown);
	printf("\nShooter Going Up Count: %d", shooterGoingUpCount);
	printf("\nShooter Override: %d", shooterOverride);
*/	

		pwm05 = pwm06 = 254;
/*	
	printf("\n\noverride left: %d", p2_sw_aux2);
	printf("\noverride right: %d", p2_sw_top);
	printf("\shooter override: %d", p3_sw_aux2);
	printf("\nbarf: %d", p3_sw_aux1);
*/	

	// Emergency stop
	/*
	if (emergencyStop == 1)
	{
		 pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  		 pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
	}
	*/

	//-----------------------------------------------
	//	UPDATE HOOD ANGLE
	//------------------------------------------------
	
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
	

	
//------------------------
//  END HOOD CODE
//-------------------------	
	
	// if override button on OI is pushed, move in that direction overriding everything else
	if (turretLeft == 1)
	{
		pwm09 = 70; // Rotate Left (clockwise)
		//printf("Override clockwise\n");
	}
	else if (turretRight == 1)
	{
		pwm09 = 184; // Rotate Right (counterclockwise)
		//printf("Override counterclockwise\n");
	}
	/*
	//========================
	// DRIVE DIFFERENTIAL CODE
	//=========================
	// RIGHT TURN, TURRET LEFT
	else if ((pwm03 > 240) & (pwm01 > 240))
		pwm09 = 70;
	// LEFT TURN, TURRET RIGHT
	else if ((pwm03 < 15) & (pwm01 < 15))
		pwm03 = 184;
	*/
	if (emergencyStop == 1)
	{
		pwm09 = 127;
		pwm05 = pwm06 = 127;
		relay2_fwd = 0;
		relay2_rev = 0;
	}
	else
	{
		pwm11 = 26;
		pwm12 = 254 - 26;
	}
		


	

	

	Putdata(&txdata);



//  ***  IFI Code Starts Here***
//
//  static unsigned char i;

  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

  Default_Routine();  /* Optional.  See below. */
//
//  /* Add your own code here. (a printf will not be displayed when connected to the breaker panel unless a Y cable is used) */
//
//  printf("Port1 Y %3d, X %3d, Fire %d, Top %d\r",p1_y,p1_x,p1_sw_trig,p1_sw_top);  /* printf EXAMPLE */
//
//  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
//
//  /* Eample code to check if a breaker was ever tripped. */
//
//  if (aBreakerWasTripped)
//  {
//    for (i=1;i<29;i++)
//    {
//      if (Breaker_Tripped(i))
//        User_Byte1 = i;  /* Update the last breaker tripped on User_Byte1 (to demonstrate the use of a user byte) 
//                            Normally, you do something else if a breaker got tripped (ex: limit a PWM output)     */
//    }
//  }
//
//  Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
 	double outputX;
	double outputY;
 // Driving lookup tables

	   
 /*---------- Analog Inputs (Joysticks) to PWM Outputs-----------------------
  *--------------------------------------------------------------------------
  *   This maps the joystick axes to specific PWM outputs.
  */

/*
  pwm01 = p1_y;   
  pwm02 = p2_y;   
  pwm03 = p3_y;   
  pwm04 = p4_y;   
  pwm05 = p1_x;   
  pwm06 = p2_x;   
  pwm07 = p3_x;   
  pwm08 = p4_x;   
  pwm09 = p1_wheel;
  pwm10 = p2_wheel;   
  pwm11 = p3_wheel;   
  pwm12 = p4_wheel;   
  
*/

 /*---------- 1 Joystick Drive ----------------------------------------------
  *--------------------------------------------------------------------------
  *  This code mixes the Y and X axis on Port 1 to allow one joystick drive. 
  *  Joystick forward  = Robot forward
  *  Joystick backward = Robot backward
  *  Joystick right    = Robot rotates right
  *  Joystick left     = Robot rotates left
  *  Connect the right drive motors to PWM01 and/or PWM02 on the RC.
  *  Connect the left  drive motors to PWM03 and/or PWM04 on the RC.
  */  
/*
	int heavisideX = 0;
	int heavisideY = 0;
	int inputY;
	int inputX;
	double outputX;
	double outputY;
	
	inputY = p1_y;
	if ((127 - inputY) < 0) 
		heavisideY = 0;
	else
		heavisideY = 1;
		
	inputX = p1_x;
	if ((127 - inputX) < 0)
		heavisideX = 0;
	else
		heavisideX = 1;

	outputX = 127-42.5*(pow (1.010889286,-inputX+127)-1)+heavisideX*42.5*(pow (1.010889286,-inputX+127)-1)+heavisideX*42.5*(pow (1.010889286,inputX-127)-1);	
	outputY = 127-42.5*(pow (1.010889286,-inputY+127)-1)+heavisideY*42.5*(pow (1.010889286,-inputY+127)-1)+heavisideY*42.5*(pow (1.010889286,inputY-127)-1);
  	
  	pwm01 = pwm02 = Limit_Mix(2000 - outputY + outputX + 127); 
  	pwm03 = pwm04 = Limit_Mix(2000 + outputY + outputX - 127); 
*/
	p1_x = 127 - (p1_x - 127);
	p1_y = 127 - (p1_y - 127);
	outputX = (((((double)p1_x-127)*((double)p1_x - 127)*((double)p1_x - 127)) / 16129) + 127);
	outputY = (((((double)p1_y-127)*((double)p1_y - 127)*((double)p1_y - 127)) / 16129) + 127);

	pwm01 = pwm02 = Limit_Mix(2000 + (int)outputY + (int)outputX - 127);  // forward 0, backward 255, left 0 right 255 to turn left
    pwm03 = pwm04 = Limit_Mix(2000 - (int)outputY + (int)outputX + 127); 
/*    
    printf("Left Drive: %u\r\n", pwm01);
    printf("Right Drive: %u\r\n", pwm03);
  */  

    /*
    
    
  pwm13 = pwm14 = Limit_Mix(2000 + p1_y + p1_x - 127);
  pwm15 = pwm16 = Limit_Mix(2000 + p1_y - p1_x + 127)*/
  
/*	
	printf("Right: %d", pwm01);
	printf("\nLeft: %d", pwm03);
	printf("\n\n");
	printf("OutputX: %d", outputX);
	printf("\nOutputY: %d", outputY);
	printf("\np1_x: %d", p1_x);
	printf("\np1_y: %d", p1_y);
	printf("\n\n\n");
*/

/*


  pwm01 = pwm02 = Limit_Mix(2000 - p1_y + p1_x + 127); 
    pwm03 = pwm04 = Limit_Mix(2000 + p1_y + p1_x - 127);
*/
  /*
  printf("\n\nX: %d", p1_x); printf(" %d", outputX);
  printf("\nY: %d", p1_y); printf(" %d", outputY);
  */
 /*---------- Buttons to Relays----------------------------------------------
  *--------------------------------------------------------------------------
  *  This default code maps the joystick buttons to specific relay outputs.  
  *  Relays 1 and 2 use limit switches to stop the movement in one direction.
  *  The & used below is the C symbol for AND                                
  */
/*
  relay1_fwd = p1_sw_trig & rc_dig_in01;  // FWD only if switch1 is not closed. 
  relay1_rev = p1_sw_top  & rc_dig_in02;  // REV only if switch2 is not closed. 
  relay2_fwd = p2_sw_trig & rc_dig_in03;  // FWD only if switch3 is not closed. 
  relay2_rev = p2_sw_top  & rc_dig_in04;  // REV only if switch4 is not closed. 
  relay3_fwd = p3_sw_trig;
  relay3_rev = p3_sw_top;
  relay4_fwd = p4_sw_trig;
  relay4_rev = p4_sw_top;
  relay5_fwd = p1_sw_aux1;
  relay5_rev = p1_sw_aux2;
  relay6_fwd = p3_sw_aux1;
  relay6_rev = p3_sw_aux2;
  relay7_fwd = p4_sw_aux1;
  relay7_rev = p4_sw_aux2;
  relay8_fwd = !rc_dig_in18;  // Power pump only if pressure switch is off. 
  relay8_rev = 0;
*/
  
  /*---------- PWM outputs Limited by Limit Switches  ------------------------*/
  
  Limit_Switch_Max(rc_dig_in05, &pwm03);
  Limit_Switch_Min(rc_dig_in06, &pwm03);
  Limit_Switch_Max(rc_dig_in07, &pwm04);
  Limit_Switch_Min(rc_dig_in08, &pwm04);
  Limit_Switch_Max(rc_dig_in09, &pwm09);
  Limit_Switch_Min(rc_dig_in10, &pwm09);
  Limit_Switch_Max(rc_dig_in11, &pwm10);
  Limit_Switch_Min(rc_dig_in12, &pwm10);
  Limit_Switch_Max(rc_dig_in13, &pwm11);
  Limit_Switch_Min(rc_dig_in14, &pwm11);
  Limit_Switch_Max(rc_dig_in15, &pwm12);
  Limit_Switch_Min(rc_dig_in16, &pwm12);
  
 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */

  { /* Check position of Port 1 Joystick */

    
    ;
    
  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   
  
} /* END Default_Routine(); */

void shootTheJ(void)
{
	// Get cover angle from 
	
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
