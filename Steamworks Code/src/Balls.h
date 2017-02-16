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
public:
	Turret turret;
	Intake intake;

	Balls(int tmotor_port, int amotor_port, int smotor_port, int imotor_port, int tencoder_1,
		  int tencoder_2, int aencoder_1, int aencoder_2, int sencoder_1, int sencoder_2):
		turret(tmotor_port, amotor_port, smotor_port, tencoder_1, tencoder_2, aencoder_1, aencoder_2, sencoder_1, sencoder_2),
		intake(imotor_port)
	{
	}

};
#endif /* SRC_BALLS_H_ */
