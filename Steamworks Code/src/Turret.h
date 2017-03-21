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
	CANTalon shoot_motor;
public:
	Turret(int tmotor_port, int amotor_port, int smotor_port, int tencoder_1,
		   int tencoder_2, int aencoder_1, int aencoder_2):
		t_motor(tmotor_port, tencoder_1, tencoder_2),
		aim_motor(amotor_port, aencoder_1, aencoder_2),
		shoot_motor(smotor_port)
	{
		t_motor.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
	}

	void Aim(){

	}

	int GetAvgRotation()
	{
		return (int)t_motor.GetDistance() % 360; //TEST PLS
	}

	double GetRealRotation()
	{
		return t_motor.GetDistance();
	}

	double GetHeight()
	{
		return aim_motor.GetDistance();
	}

	void SetRotation(double rate)
	{
		t_motor.Set(rate);
	}

	void SetHeight(double rate)
	{
		aim_motor.Set(rate);
	}

	void Fire()
	{
		shoot_motor.Set(1);
	}

	void StopFire()
	{
		shoot_motor.Set(0);
	}

	void StopAll()
	{
		t_motor.Set(0);
		aim_motor.Set(0);
		shoot_motor.Set(0);
	}
};

#endif /* SRC_TURRET_H_ */
