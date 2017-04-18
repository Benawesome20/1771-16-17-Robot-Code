#include "CanTalonSRX.h"

#ifndef WHEEL_H_P_P
#define WHEEL_H_P_P
class Wheel {
	CanTalonSRX motor[2];
public:
	void Set(double speed);
	double operator=(double d);
};
#endif //WHEEL_H_P_P