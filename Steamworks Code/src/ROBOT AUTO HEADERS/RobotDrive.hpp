#include "Wheel.hpp"
#include "Point.hpp"
#include "Position.hpp"
#include "Vector3.hpp"

#ifndef ROBOTDRIVE_H_P_P
#define ROBOTDRIVE_H_P_P

class RobotDrive {
	Wheel wheel[4];

	Vector3 pre_dir;
	Position pos;
	Point pre_pos;
	
	double v2dc(double theta, bool r);
	
public:
	void Move(Vector3 dir, double r);
	Point GetVelocity();
};

#endif //ROBOTDRIVE_H_P_P