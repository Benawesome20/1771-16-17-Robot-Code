/**
 * Enums.h
 *
 * Created on: Apr 11, 2016
 *     Auther: Ryan McHale
 */
 
#ifndef SRC_ENUMS_H_
#define SRC_ENUMS_H_
 
enum {
	Nothing,
	LowBar,
	RockWall,
	PortAutoMode,
	LowBarShoot,
	FastRockWall
} Robot::AutoMode;

typedef enum {
	Floor,
	Shoot,
	Climb,
	Traverse,
	Traverse_Pickup,
	Camera_Setpoint,
	Slider_Pos,
	Pre_Climb,
	Portculus,
	Low_Goal
} Robot::ArmPosition;

typedef enum {
	Port,
	Zero,
	Climber,
	ClimbDown,
	Snap
} Robot::WinchPosition;

#endif // SRC_ENUMS_H_