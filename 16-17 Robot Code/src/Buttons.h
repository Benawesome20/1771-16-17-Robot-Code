/*
 * Buttons.h
 *
 *  Created on: Mar 31, 2016
 *      Author: Ryan McHale
 */

#ifndef SRC_BUTTONS_H_
#define SRC_BUTTONS_H_


#define LS(x) l_stick.GetRawButton(x)	// Left Joystick Button
#define L1 LS(1)
#define L8 LS(8)	// Intake Rollers
#define L9 LS(9)	// Shoot Rollers

#define RS(x) r_stick.GetRawButton(x)	// Right Joystick Button
#define R11 RS(11)	// Carmera Tracking

#define AS(x) a_stick.GetRawButton(x)	// Sabrina's Joystick Button
#define A1 AS(1)    // Climb (when A2)
#define A2 AS(2)    // Preclimb
#define A3 AS(3)	// WinchPos Port

#define A6 AS(6)	// WinchPos Climber
#define A7 AS(7)	// WinchPos ClimbDown

#define A8 AS(8)	// Shoot Rollers
#define A9 AS(9)	// ArmPos Low_Goal
#define A10 AS(10)	// ArmPos PowerOff
#define A11 AS(11)	// PickUp
#define A12 AS(12)	// ArmPos Shoot


#endif /* SRC_BUTTONS_H_ */
