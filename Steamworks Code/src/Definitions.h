/*
 * Definitions.h
 *
 *  Created on: February 10, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_DEFINITIONS_H_
#define SRC_DEFINITIONS_H_

#define SHIFT_UP_SPEED 3100 				//Encoder speed(rate) limits for shifting gears
#define SHIFT_DOWN_SPEED 25250

#define L_MOTOR_PORT_1 1					//Left motor port 1
#define L_MOTOR_PORT_2 2					//Left motor port 2
#define R_MOTOR_PORT_1 3					//Right motor port 1
#define R_MOTOR_PORT_2 4					//Right motor port 2
#define T_MOTOR_PORT   5					//Turret motor port, controls rotation
#define A_MOTOR_PORT   6					//Aim motor port, controls aiming hood, which controls the height of the shot
#define S_MOTOR_PORT   7					//Shoot motor port, controls the flywheels
#define I_MOTOR_PORT   8					//Intake motor port

#define L_ENCODER_CH_1 0					//Left encoder channel 1
#define L_ENCODER_CH_2 1					//Left encoder channel 2
#define R_ENCODER_CH_1 2					//Right encoder channel 1
#define R_ENCODER_CH_2 3					//Right encoder channel 2
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

#endif /* SRC_DEFINITIONS_H_ */
