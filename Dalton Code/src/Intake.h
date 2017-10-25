/*
 * Intake.h
 *
 *  Created on: January 21, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <WPILib.h>
#include <CANTalon.h>

class Intake {
	CANTalon i_motor;
public:
	Intake(int imotor_port):
		i_motor(imotor_port)
	{
	}

	void Set(double rate)
	{
		i_motor.Set(rate);
	}

};
#endif /* SRC_INTAKE_H_ */
