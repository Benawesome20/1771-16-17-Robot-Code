#include <WPILib.h>
#include "Vector3.hpp"
#include "Input.hpp"

#ifndef POSITION_H_P_P
#define POSITION_H_P_P

class Position{
	double dt, rot, sinr, cosr;
	Point pos;
	
	Vector3 vel;
	Vector3 acc_v;

	struct INPUT input;

	Timer dt_t;
	
public:
	Position(Point start);
	
	void Reset();
	void Update();
};

#endif //POSITION_H_P_P
