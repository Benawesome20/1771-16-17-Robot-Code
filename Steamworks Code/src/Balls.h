/*
 * Balls.h
 *
 *  Created on: January 21, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_BALLS_H_
#define SRC_BALLS_H_

#include <WPILib.h>
#include <CANTalon.h>
#include "Turret.h"
#include "Intake.h"

class Balls {
	Turret turret;
	Intake intake;

public:
	Balls()
	{
	}

};
#endif /* SRC_BALLS_H_ */
