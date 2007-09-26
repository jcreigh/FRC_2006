/*******************************************************************************
*
*	TITLE:		tracking.h 
*
*	VERSION:	0.1 (Beta)                           
*
*	DATE:		26-Sep-2005
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This is the "bells and whistles" version of tracking.h
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
#ifndef _TRACKING_H
#define _TRACKING_H

// By default, PWM output one is used for the pan servo.
// Change it to another value if you'd like to use PWM
// output one for another purpose.
#define PAN_SERVO pwm09

// By default, PWM output two is used for the tilt servo.
// Change it to another value if you'd like to use PWM
// output two for another purpose.
#define TILT_SERVO pwm10

// base address in EEPROM where Get_Tracking_Configuration()
// will look for valid tracking configuration data
#define TRACKING_CONFIG_EEPROM_ADDRESS 32

// This value defines how many "slow loops" to wait before
// sending the tracking servo(s) to their next destination
// while in search mode. This provides a small delay for the 
// camera to lock onto the target between position changes.
#define SEARCH_DELAY_DEFAULT 5

// These values define how quickly the camera will attempt 
// to track the object. If these are set too high, the camera 
// will take longer to settle, too low and the camera will
// overshoot the target and oscillate.
#define PAN_GAIN_DEFAULT 1
#define TILT_GAIN_DEFAULT 8

// If your camera suddenly moves away from the target once
// it finds it, you'll need to change the sign on one or
// both of these values.
#define PAN_ROTATION_SIGN_DEFAULT +1
#define TILT_ROTATION_SIGN_DEFAULT +1

// These two values define the image pixel that we're
// going to try to keep the tracked object on. By default
// the center of the image is used.
#define PAN_TARGET_PIXEL_DEFAULT 89
#define TILT_TARGET_PIXEL_DEFAULT 119

// These values define how much error, in pixels, is 
// allowable when trying to keep the center of the tracked
// object on the center pixel of the camera's imager. Too 
// high a value and your pointing accuracy will suffer, too
// low and your camera may oscillate because the servos 
// don't have enough pointing resolution to get the center 
// of the tracked object into the square/rectangle defined 
// by these values 
#define PAN_ALLOWABLE_ERROR_DEFAULT 6
#define TILT_ALLOWABLE_ERROR_DEFAULT 6

// These values define the lower and upper bound on the
// pan servo travel
#define PAN_MIN_PWM_DEFAULT 0   // -65 degrees
#define PAN_MAX_PWM_DEFAULT 248 // +65 degrees

// This value defines how far the pan servo will step
// each time while in search mode. This value was chosen
// to provide five evenly spaced stopping points (i.e., 0,
// 62, 124, 186 and 248) for the pan servo.
#define PAN_SEARCH_STEP_SIZE_DEFAULT 62

// These values define the PWM values of the pan and tilt
// servos when the camera is pointing directly ahead
// (i.e., when pan and tilt angles are zero).
#define PAN_CENTER_PWM_DEFAULT 124
#define TILT_CENTER_PWM_DEFAULT 114

// These values define the lower and upper bound on the
// tilt servo travel
#define TILT_MIN_PWM_DEFAULT 84  // -25 degrees
#define TILT_MAX_PWM_DEFAULT 144 // +25 degrees

// This value defines how far the tilt servo will step
// each time while in search mode. This value was chosen
// to provide three evenly spaced stopping points (i.e.,
// 94, 144 and 194) for the tilt servo.
#define TILT_SEARCH_STEP_SIZE_DEFAULT 18

// parameters for CMUcam2 with OV7620 camera module
#define IMAGE_WIDTH 159
#define IMAGE_HEIGHT 239

// Get_Tracking_Configuration() return values
#define TRACKING_EEPROM_USED 0
#define TRACKING_EEPROM_CORRUPT 1
#define TRACKING_NO_EEPROM 2
#define TRACKING_FORCE_DEFAULT 3

// this defines the tracking configuration data structure
// that is created in RAM and possibly EEPROM
typedef struct
{
	unsigned char Letter_G;				// first identification byte
	unsigned char Letter_K; 			// second identification byte
	unsigned char Pan_Min_PWM;			//
	unsigned char Pan_Center_PWM;		//
	unsigned char Pan_Max_PWM;			//
	unsigned char Pan_Gain;				//
	unsigned char Pan_Allowable_Error;	//
	         char Pan_Rotation_Sign;	//
	unsigned char Pan_Search_Step_Size;	//
	unsigned char Pan_Target_Pixel;		//
	unsigned char Tilt_Min_PWM;			//
	unsigned char Tilt_Center_PWM;		//
	unsigned char Tilt_Max_PWM;			//
	unsigned char Tilt_Gain;			//
	unsigned char Tilt_Allowable_Error;	//
	         char Tilt_Rotation_Sign;	//
	unsigned char Tilt_Search_Step_Size;//
	unsigned char Tilt_Target_Pixel;	//
	unsigned char Search_Delay;			//
	unsigned char Checksum;				// eight-bit structure checksum
}	Tracking_Config_Data_Type;

// global variables
extern Tracking_Config_Data_Type Tracking_Config_Data;

// function prototypes
int Servo_Track(int driveR, int driveL);
void Initialize_Tracking(void);
unsigned char Get_Tracking_Configuration(unsigned int, unsigned char);

#endif
