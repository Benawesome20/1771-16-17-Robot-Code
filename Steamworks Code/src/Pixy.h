/*
 * Pixy.h
 *
 *  Created on: February 8, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_PIXY_H_
#define SRC_PIXY_H_

#include <WPILib.h>
#include <CANTalon.h>

class Pixy {
	AnalogInput offset;

public:
	Pixy(int offsetCh):
		offset(offsetCh)
	{
	}

	/* Purpose: Returns the offset of the tracked target from (-1.0,1.0), with 0 being the center
	 *
	 * Method : Checks if the PixyCam sees a target
	 * 			subtracts half the total voltage to center 0 at the center of the PixyCam's field of view
	 * 			divides by half the total voltage to make the right and left ends 1 and -1 */
	double GetOffset()
	{
		if (offset.GetVoltage())
			return ((double)offset.GetVoltage() - (3.3/2.0)) / (3.3/2.0);
		else
			return 0.0;
	}

	/* Returns the raw offset data in voltage form, (0V, 3.3V) */
	double GetRawOffset()
	{
		return offset.GetVoltage();
	}

};
#endif /* SRC_PIXY_H_ */
