/*******************************************************************************
*
*	TITLE:		tracking.c 
*
*	VERSION:	0.1 (Beta)                           
*
*	DATE:		26-Sep-2005
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This is the "bells and whistles" version of tracking.c
*
*				You are free to use this source code for any non-commercial
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
*	26-Sep-2005  0.1  RKW - Original code.
*
*******************************************************************************/
#include <stdio.h>
#include "ifi_default.h"
#include "ifi_aliases.h"
#include "eeprom.h"
#include "camera.h"
#include "tracking.h"

// This variable, when equal to one, indicates that the tracking 
// software has successfully initialized and should be running. 
// You can also force a re-initialization by setting this variable 
// to zero by calling the function Initialize_Tracking().
unsigned char tracking_initialized = 0;

// tracking configuration data structure
Tracking_Config_Data_Type Tracking_Config_Data;

/*******************************************************************************
*
*	FUNCTION:		Servo_Track()
*
*	PURPOSE:		This function reads data placed in the T_Packet_Data
*					structure by the Camera_Handler() function and if new
*					tracking data is available, attempts to keep the center
*					of the tracked object in the center of the camera's
*					image using two servos that drive a pan/tilt platform.
*					If the camera doesn't have the object within it's field 
*					of view, this function will execute a search algorithm 
*					in an attempt to find the object.		
*
*	CALLED FROM:	user_routines.c/Process_Data_From_Master_uP()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		This version of the tracking code uses a proportional
*					feedback controller to track the object.
*
*******************************************************************************/
int Servo_Track(int driveR, int driveL)
{
	static unsigned int old_camera_t_packets = 0;
	static unsigned char new_search = 1;
	static unsigned char loop_count = 0;
	int temp_pan_servo;
	int temp_tilt_servo;
	int servo_step;
	int pan_error;
	int tilt_error;
	int velocityToStart = 0;
	int turnVelocity = 1;
	int searchSpeed = 1;
	int letMyAimBeTrue = 0;
	int driveDifferential = 0;
	int driveDifferentialCorrection = 0;
	int driveDifferentialThreshold = 6;
	int position = 0;

	// Starts at 10
	const int hood_angle[] = {32, 32, 32, 32, 32, 30, 28, 27, 26, 26,
							 26, 26, 26, 26, 26, 26, 26, 26, 32, 32,
							 27, 28, 28, 28, 29, 30, 30, 31, 31, 33 };


	

	// calculate drive differential and change into pan speed
	// pwm01 is right motor, pwm03 is left motor
	// positive differential is counter-clockwise rotation
	// negative differential is clockwise rotation
	/*
	if ((driveR == driveL) && ((driveR > 200) || (driveR < 50)))
	{
		if (driveR > 200)
			driveDifferentialCorrection = 10;
		else if (driveR < 50)
			driveDifferentialCorrection = -10;
	}
	*/
	/*

	driveDifferential = driveR + driveL;
	if (driveDifferential > driveDifferentialThreshold)
		driveDifferentialCorrection = driveDifferential + 8;
	else if  (-1 * driveDifferential < driveDifferentialThreshold)
		driveDifferentialCorrection = driveDifferential - 8;
	else
		driveDifferential = 0;
	*/
	
	// if needed, (re)initialize the tracking code
	if(tracking_initialized == 0)
	{
		tracking_initialized = 1;
		Initialize_Tracking();
	}

	// Has a new camera t-packet arrived since we last checked?
	if(camera_t_packets != old_camera_t_packets)
	{

		old_camera_t_packets = camera_t_packets;

		// Does the camera have a tracking solution? If so,
		// do we need to move the servos to keep the center
		// of the tracked object centered within the image?
		// If not, we need to drop down below to start or
		// continue a search
		if(T_Packet_Data.my != 0)
		{
			// if we're tracking, reset the search
			// algorithm so that a new search pattern
			// will start should we lose tracking lock
			new_search = 1;

			////////////////////////////////
			//                            //
			//	x-axis/pan tracking code  //
			//                            //
			////////////////////////////////


			// save the current pan servo PWM value into a local
			// integer variable so that we can detect and correct 
			// underflow and overflow conditions before we update 
			// the pan servo PWM value with a new value
			temp_pan_servo = (int)PAN_SERVO;

			// calculate how many image pixels we're away from the
			// vertical center line.
			pan_error = (int)T_Packet_Data.mx - (int)Tracking_Config_Data.Pan_Target_Pixel;

			// Are we too far to the left or right of the vertical 
			// center line? If so, calculate how far we should step
			// the pan servo to reduce the error.
			if(pan_error > (int)Tracking_Config_Data.Pan_Allowable_Error)
			{
				// calculate how far we need to step the pan servo
				servo_step = pan_error / (int)Tracking_Config_Data.Pan_Gain;

				// Due to rounding error in the division calculation above,
				// the step may be calculated as zero, which will make it
				// impossible to converge on the target when x_error is
				// smaller than X_GAIN. To get around this problem, we just 
				// test for the zero case and set the step size to one. 
				if(servo_step == 0)
				{
					servo_step = 1;
				}
			}
			else if(pan_error < -1 * (int)Tracking_Config_Data.Pan_Allowable_Error)
			{
				// calculate how far we need to step the pan servo
				servo_step = pan_error / (int)Tracking_Config_Data.Pan_Gain;

				// Due to rounding error in the division calculation above,
				// the step may be calculated as zero, which will make it
				// impossible to converge on the target when x_error is
				// smaller than X_GAIN. To get around this problem, we just 
				// test for the zero case and set the step size to one. 
				if(servo_step == 0)
				{
					servo_step = -1;
				}
			}
			else
			{
				// if we've fallen through to here, it means that we're
				// neither too far to the left or too far to the right
				// of the vertical center line of the image and don't 
				// need to move the servo
				servo_step = 0;
				letMyAimBeTrue = 1;
			}
/*
			// add the step to the current servo position, taking into
			// account the direction set by the user in tracking.h
			temp_pan_servo += ((int)Tracking_Config_Data.Pan_Rotation_Sign * servo_step);

			// check the pan servo PWM value for under/overflow
			if(temp_pan_servo < (int)Tracking_Config_Data.Pan_Min_PWM)
			{
				temp_pan_servo = (int)Tracking_Config_Data.Pan_Min_PWM;
			}
			else if(temp_pan_servo > (int)Tracking_Config_Data.Pan_Max_PWM)
			{
				temp_pan_servo = (int)Tracking_Config_Data.Pan_Max_PWM;
			}

			// update pan servo PWM value
			PAN_SERVO = (unsigned char)temp_pan_servo;

*/

/*
			// Move the turret motor in the direction of the error
			if (servo_step > 0)
			{
				temp_pan_servo = (127 + ((int)Tracking_Config_Data.Pan_Rotation_Sign * turnVelocity)); 
			}
			else if (servo_step < 0)
			{
				temp_pan_servo = (127 - ((int)Tracking_Config_Data.Pan_Rotation_Sign * turnVelocity));
			}
			else
			{
				temp_pan_servo = 127;
				letMyAimBeTrue = 1;
			}
*/

			temp_pan_servo = 127 + (((int)Tracking_Config_Data.Pan_Rotation_Sign )* (velocityToStart + servo_step) + driveDifferential);
			
			PAN_SERVO = (unsigned char)temp_pan_servo;


			/////////////////////////////////
			//                             //
			//	y-axis/tilt tracking code  //
			//                             //
			/////////////////////////////////

			// save the current tilt servo PWM value into a local
			// integer variable so that we can detect and correct 
			// underflow and overflow conditions before we update 
			// the tilt servo PWM value with a new value
			temp_tilt_servo = (int)TILT_SERVO;
			//printf("TrackingY\n");

			// calculate how many image pixels we're away from the
			// horizontal center line.
			tilt_error = (int)T_Packet_Data.my - (int)Tracking_Config_Data.Tilt_Target_Pixel;

			// Are we too far above or below the horizontal center line?
			// If so, calculate how far we should step the tilt servo to 
			// reduce the error.
			if(tilt_error > (int)Tracking_Config_Data.Tilt_Allowable_Error)
			{
				// calculate how far we need to step the tilt servo
				servo_step = tilt_error / (int)Tracking_Config_Data.Tilt_Gain;

				// Due to rounding error in the division calculation above,
				// the step may be calculated as zero, which will make it
				// impossible to converge on the target when y_error is
				// smaller than Y_GAIN. To get around this problem, we just 
				// test for the zero case and set the step size to one. 
				if(servo_step == 0)
				{
					servo_step = 1;
				}
			}
			else if (tilt_error < -1 * (int)Tracking_Config_Data.Tilt_Allowable_Error)
			{
				// calculate how far we need to step the tilt servo
				servo_step = tilt_error / (int)Tracking_Config_Data.Tilt_Gain;

				// Due to rounding error in the division calculation above,
				// the step may be calculated as zero, which will make it
				// impossible to converge on the target when x_error is
				// smaller than X_GAIN. To get around this problem, we just 
				// test for the zero case and set the step size to one. 
				if(servo_step == 0)
				{
					servo_step = -1;
				}
			}
			else
			{
				// if we've fallen through to here, it means that we're
				// neither too far below or to far above the horizontal
				// center line of the image and don't need to move the
				// servo
				servo_step = 0;
			}

			// add the step to the current servo position, taking into
			// account the direction set by the user in tracking.h
			temp_tilt_servo += ((int)Tracking_Config_Data.Tilt_Rotation_Sign * servo_step);

			// check the tilt PWM value for under/overflow
			if(temp_tilt_servo < (int)Tracking_Config_Data.Tilt_Min_PWM)
			{
				temp_tilt_servo = (int)Tracking_Config_Data.Tilt_Min_PWM;
			}
			else if(temp_tilt_servo > (int)Tracking_Config_Data.Tilt_Max_PWM)
			{
				temp_tilt_servo = (int)Tracking_Config_Data.Tilt_Max_PWM;
			}
//			printf("Updating tilt PWM value in tracking\n");
			// update tilt servo PWM value
			TILT_SERVO = (unsigned char)temp_tilt_servo;
			/*
			// Update hood angle based on tilt pwm value
			if (((temp_tilt_servo - 30) < 40) && ((temp_tilt_servo - 30) > 13))
			{
				pwm11 = hood_angle[temp_tilt_servo - 40] + 120;
				pwm12 = 254 - hood_angle[temp_tilt_servo - 40] + 120;
			}
			else
			{
				pwm11 = 127;
				pwm12 = 127;
			}

			*/
/*
			//printf("\nCalculating Hood Position\n");
			if ((127 - temp_tilt_servo) < 0)
				position = 254 + (127 - temp_tilt_servo);
			else if ((127 - temp_tilt_servo) > 254)
				position = (127 - temp_tilt_servo) - 254;
			else
				position = (127 - temp_tilt_servo);
			
			if (((position - 30 - 66) < 40) && ((position - 30 - 66) > 13))
			{
					pwm11 = hood_angle[position - 66 - 10];
					pwm12 = 254 - hood_angle[position - 66 - 10];
			}
			else
				pwm11 = pwm12 = 127;
			
*/

		}
		else
		{
			///////////////////
			//               //
			//  search code  //
			//               //
			///////////////////
			letMyAimBeTrue = 300;
			// To provide a delay for the camera to lock onto the
			// target between position changes, we only step the camera
			// to a new position every SEARCH_DELAY times while we're 
			// in search mode. SEARCH_DELAY is #define'd in tracking.h
			loop_count++;

			if(loop_count > 12)
			{
				// reset the loop counter
				loop_count = 0;

				// If we're starting a new search, initialize the pan
				// and tilt servos to the search starting point.
				// Otherwise, just continue the search pattern from
				// where we left off. The variable new_search is reset
				// to one each time the tracking code (above) executes.
				if(new_search == 1)
				{
					new_search = 0;
					//temp_pan_servo = 0;
					temp_tilt_servo = Tracking_Config_Data.Tilt_Center_PWM;
										
				}
				else
				{
					if (loop_count <= 8)
					{
						//temp_pan_servo = 157;
						loop_count++;
					}
				
					else
					{
						temp_pan_servo = 127;
						loop_count++;
					}	
// calculate new servo position(s) based upon our
					// current servo position(s)
					temp_tilt_servo = (int)TILT_SERVO;
	
					// if the pan servo is at the end of its travel, 
					// send it back to the start and step the tilt
					// servo to its next position

		
						// if the tilt servo is at the end of its
						// travel, send it back to the start
						if(temp_tilt_servo >= (int)Tracking_Config_Data.Tilt_Max_PWM)
						{
							temp_tilt_servo = (int)Tracking_Config_Data.Tilt_Min_PWM;
						}
						else
						{
							// step the tilt servo to its next destination
							temp_tilt_servo += (int)Tracking_Config_Data.Tilt_Search_Step_Size;
			
							// make sure its new position isn't beyond
							// the maximum value set in tracking.h
							if(temp_tilt_servo >= (int)Tracking_Config_Data.Tilt_Max_PWM)
							{
								temp_tilt_servo = (int)Tracking_Config_Data.Tilt_Max_PWM;
						}	}
						
					

					
				}


					
				// update the pan and tilt servo PWM value
				pwm09 = 127;
				TILT_SERVO = (unsigned char)temp_tilt_servo;

			}


		}
	}
	if (letMyAimBeTrue == 1)
		return temp_tilt_servo;
	else if (letMyAimBeTrue == 300)
		return letMyAimBeTrue;
	else
		return 0;
}

/*******************************************************************************
*
*	FUNCTION:		Initialize_Tracking()
*
*	PURPOSE:		This function is responsable for initializing the
*					tracking software.
*
*	CALLED FROM:	Servo_Track(), above.
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:		Setting the second value that Get_Tracking_Configuration()
*					is called with (see below) to one will force the usage of
*					the default configuration values from tracking.h.
*
*******************************************************************************/
void Initialize_Tracking(void)
{
	unsigned char returned_value;

	// load tracking configuration structure
	returned_value = Get_Tracking_Configuration(TRACKING_CONFIG_EEPROM_ADDRESS, 0);

	// if debugging mode is on, report where the tracking configuration 
	// data came from (DEBUG() is a macro defined in camera.h)
	if(returned_value == TRACKING_EEPROM_USED)
	{
		DEBUG(("Tracking: Configuring with EEPROM data\r\n"));
	}
	else if (returned_value == TRACKING_EEPROM_CORRUPT)
	{
		DEBUG(("Tracking: EEPROM configuration corrupted; Using default parameters\r\n"));
	}
	else if(returned_value == TRACKING_NO_EEPROM)
	{
		DEBUG(("Tracking: No EEPROM configuration data found; Using default parameters\r\n"));
	}
	else if(returned_value == TRACKING_FORCE_DEFAULT)
	{
		DEBUG(("Tracking: force_default flag set; Using default parameters\r\n"));
	}	
}

/*******************************************************************************
*
*	FUNCTION:		Get_Tracking_Configuration()
*
*	PURPOSE:		Initializes the Tracking_Config_Data structure with data
*					stored in EEPROM. If valid data isn't present in EEPROM 
*					or the force_default flag is set, default values from 
*					tracking.h will be used.		
*
*	CALLED FROM:	Initialize_Tracking(), above
*
*	PARAMETERS:		Calling this function with a value greater than zero will
*					force it to load the configuration structure with the
*					default values found in tracking.h.
*
*	RETURNS:		TRACKING_EEPROM_USED if EEPROM configuration data was 
*					found and used.
*
*					TRACKING_EEPROM_CORRUPT if EEPROM configuration data was 
*					corrupt and default values from tracking.h were used.
*
*					TRACKING_NO_EEPROM if no EEPROM configuration data was
*					found and default values from tracking.h were used.
*
*					TRACKING_FORCE_DEFAULT if the force_default flag is set
*					and default values from camera.h were used.
*
*	COMMENTS:		The return values are defined in tracking.h.
*
*******************************************************************************/
unsigned char Get_Tracking_Configuration(unsigned int eeprom_address, unsigned char force_default)
{
	unsigned char i;
	unsigned char byte;
	unsigned int checksum;
	unsigned int return_value;

	if(force_default == 0)
	{
		checksum = 0;

		// blindly load the Tracking_Config_Data structure with EEPROM data
		// starting at "eeprom_address"
		for(i = 0; i < sizeof(Tracking_Config_Data); i++)
		{
			// read the EEPROM
			byte = EEPROM_Read(eeprom_address + (unsigned int)i);
	
			// this ugly code allows the tracking configuration data
			// structure to be addressed as an unsigned char array
			((unsigned char *)(&Tracking_Config_Data))[i] = byte;
	
			// add every byte, except the last, to the checksum
			if(i < sizeof(Tracking_Config_Data) - 1)
			{
				checksum += (unsigned int)byte;
			}
		}

		// okay, we've blindly loaded the Tracking_Config_Data structure
		// with the EEPROM data; lets make sure the identification bytes
		// are present
		if(Tracking_Config_Data.Letter_G == 'G' && Tracking_Config_Data.Letter_K == 'K')
		{
			// yep, they're present, so let's make sure the data isn't corrupted
			if(Tracking_Config_Data.Checksum == (unsigned char)checksum)
			{
				// data is good; we're done
				return_value = TRACKING_EEPROM_USED;
			}
			else
			{
				// data is corrupt; use default values from tracking.h
				return_value = TRACKING_EEPROM_CORRUPT;
			}
		}
		else
		{
			// no configuration structure found in EEPROM; use default values from tracking.h
			return_value = TRACKING_NO_EEPROM;
		}
	}
	else
	{
		// force the use of tracking.h default values
		return_value = TRACKING_FORCE_DEFAULT;
	}


	// use configuration data from tracking.h if we couldn't 
	// get valid configuration data from EEPROM or if the
	// force_default flag is set
	if(return_value != TRACKING_EEPROM_USED)
	{
		Tracking_Config_Data.Pan_Min_PWM = PAN_MIN_PWM_DEFAULT;
		Tracking_Config_Data.Pan_Center_PWM = PAN_CENTER_PWM_DEFAULT;
		Tracking_Config_Data.Pan_Max_PWM = PAN_MAX_PWM_DEFAULT;
		Tracking_Config_Data.Pan_Gain = PAN_GAIN_DEFAULT;
		Tracking_Config_Data.Pan_Allowable_Error = PAN_ALLOWABLE_ERROR_DEFAULT;
		Tracking_Config_Data.Pan_Rotation_Sign = PAN_ROTATION_SIGN_DEFAULT;
		Tracking_Config_Data.Pan_Search_Step_Size = PAN_SEARCH_STEP_SIZE_DEFAULT;
		Tracking_Config_Data.Pan_Target_Pixel = PAN_TARGET_PIXEL_DEFAULT;
		Tracking_Config_Data.Tilt_Min_PWM = TILT_MIN_PWM_DEFAULT;
		Tracking_Config_Data.Tilt_Center_PWM = TILT_CENTER_PWM_DEFAULT;
		Tracking_Config_Data.Tilt_Max_PWM = TILT_MAX_PWM_DEFAULT;
		Tracking_Config_Data.Tilt_Gain = TILT_GAIN_DEFAULT;
		Tracking_Config_Data.Tilt_Allowable_Error = TILT_ALLOWABLE_ERROR_DEFAULT;
		Tracking_Config_Data.Tilt_Rotation_Sign = TILT_ROTATION_SIGN_DEFAULT;
		Tracking_Config_Data.Tilt_Search_Step_Size = TILT_SEARCH_STEP_SIZE_DEFAULT;
		Tracking_Config_Data.Tilt_Target_Pixel = TILT_TARGET_PIXEL_DEFAULT;
		Tracking_Config_Data.Search_Delay = SEARCH_DELAY_DEFAULT;
	}
	return(return_value);
}
