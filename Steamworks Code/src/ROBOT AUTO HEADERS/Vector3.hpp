#include "Point.hpp";

#ifndef VECTOR3_H_P_P
#define VECTOR3_H_P_P
class Vector3 : public Point {
public:
	explicit Vector3(const Vector3&);
	Vector3(double x, double y, double theta);
};
	

#endif //VECTOR3_H_P_P
