/*
 * Definitions.h
 *
 *  Created on: February 10, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_DEFINITIONS_H_
#define SRC_DEFINITIONS_H_

#define SHIFT_UP_SPEED 3100 				//Encoder speed(rate) limits for shifting gears
#define SHIFT_DOWN_SPEED 30250

#define L_MOTOR_PORT_1 1					//Left motor port 1
#define L_MOTOR_PORT_2 2					//Left motor port 2
#define R_MOTOR_PORT_1 3					//Right motor port 1
#define R_MOTOR_PORT_2 4					//Right motor port 2
#define C_MOTOR_PORT_1 5					//Climbing motor port 1
#define C_MOTOR_PORT_2 6					//Climbing motor port 2
#define T_MOTOR_PORT   7 //change					//Turret motor port, controls rotation
#define A_MOTOR_PORT   8					//Aim motor port, controls aiming hood, which controls the height of the shot
#define S_MOTOR_PORT   10					//Shoot motor port, controls the flywheels temp 10
#define I_MOTOR_PORT   9					//Intake motor port

#define L_ENCODER_CH_1 0					//Left encoder channel 1
#define L_ENCODER_CH_2 1					//Left encoder channel 2
#define R_ENCODER_CH_1 2					//Right encoder channel 1
#define R_ENCODER_CH_2 4					//Right encoder channel 2
#define T_ENCODER_CH_1 4					//Turret encoder channel 1
#define T_ENCODER_CH_2 5					//Turret encoder channel 2
#define A_ENCODER_CH_1 6					//Aim encoder channel 1
#define A_ENCODER_CH_2 7					//Aim encoder channel 2
//#define S_ENCODER_CH_1 8					//Shoot encoder channel 1
//#define S_ENCODER_CH_2 9					//Shoot encoder channel 2

#define SHIFT_PORT 0						//Shifting solenoid port
#define GEAR_PORT  1

#define L_JOYSTICK_PORT 0					//Left joystick port
#define R_JOYSTICK_PORT 1					//Right joystick port
#define T_JOYSTICK_PORT 2

#define OFFSET_PORT 9						//PixyCam camera port

//Default address of Pixy Camera I2C. You can change the address of the Pixy in Pixymon under setting-> Interface
#define PIXY_I2C_DEFAULT_ADDR           0x54

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L   //x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

#define BOILER_SIGNATURE			2

#define ANGLE_GAIN					0.7

/* STRING SHORTENING */
#define _0 		0b0000
#define _1 		0b0001
#define _2 		0b0010
#define _3 		0b0011
#define _4 		0b0100
#define _5 		0b0101
#define _6 		0b0110
#define _7 		0b0111
#define _8 		0b1000
#define _9 		0b1001
#define _dot 	0b1010
#define _neg	0b1011
#define _end	0b1100

#endif /* SRC_DEFINITIONS_H_ */
