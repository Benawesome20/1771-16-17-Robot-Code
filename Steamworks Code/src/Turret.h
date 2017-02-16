/*
 * Turret.h
 *
 *  Created on: January 21, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_TURRET_H_
#define SRC_TURRET_H_

#include <WPILib.h>
#include <CANTalon.h>

#include "Definitions.h"
#include "Transmission.h"
#include "Pixy.h"

class Turret {
	Transmission t_motor;
	Transmission aim_motor;
	Transmission shoot_motor;
public:
	Turret(int tmotor_port, int amotor_port, int smotor_port, int tencoder_1,
		   int tencoder_2, int aencoder_1, int aencoder_2, int sencoder_1, int sencoder_2):
		t_motor(tmotor_port, tencoder_1, tencoder_2),
		aim_motor(amotor_port, aencoder_1, aencoder_2),
		shoot_motor(smotor_port, sencoder_1, sencoder_2)
	{
	}

	void Aim()
	{
	}

	int GetRotation()
	{
		return (int)t_motor.GetDistance() % 360; //TEST PLS
	}

	double GetHeight()
	{
		return 0;
	}

	void SetRotation()
	{
	}

	void SetHeight()
	{
	}
};

#endif /* SRC_TURRET_H_ */
