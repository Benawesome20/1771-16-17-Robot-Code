#include <WPILib.h>
#include "NavX.h"
#include "Vector3.hpp"

#ifndef INPUT_H_P_P
#define INPUT_H_P_P


struct INPUT{
	frc::BuiltInAccelerometer acc;
	NavX gyro;

	Vector3 FormattedAccl();

	double GetH();
	double GetF();
	double GetS();

	double GetX();
	double GetY();
	double GetZ();
};

#endif //INPUT_H_P_P
