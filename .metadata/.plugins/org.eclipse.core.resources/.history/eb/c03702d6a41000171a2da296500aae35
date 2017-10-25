/*
 * Climber.h
 *
 *  Created on: Mar 9, 2017
 *      Author: Justin Desimpliciis
 */

#ifndef SRC_CLIMBER_H_
#define SRC_CLIMBER_H_

#include <WPILib.h>
#include <CANTalon.h>

#include "Definitions.h"
#include "Transmission.h"

class Climber {
	CANTalon climb_motor;
	double startingDist;

public:
	Climber(int climb_port):
		climb_motor(climb_port)
	{
		startingDist = 0;
		climb_motor.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	}

	void ClimbUp(int distance)
	{
		if (climb_motor.GetEncPosition() < distance )
		{
			climb_motor.Set(.5);
		}
		else
		{
			climb_motor.Set(0);
		}

		//^^ I added the else statement, but I'm not sure if it's necessary
	}

	void ManualClimb(double rate)
	{
		climb_motor.Set(rate);
	}

	double GetEncPosition()
	{
		return climb_motor.GetEncPosition();
	}

	void ClearEncoder()
	{
		climb_motor.SetEncPosition(0);
	}
};


#endif /* SRC_CLIMBER_H_ */
