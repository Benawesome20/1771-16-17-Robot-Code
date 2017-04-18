#include <WPILib.h>
#include "NavX.h"

#ifndef INPUT_H_P_P
#define INPUT_H_P_P

class Input {
public:
	NavX navx;
	BuiltInAccerometer acc;
	
	double GetH();
	double GetF();
	double GetS();
	
	Vector3 Get();
	
	double GetX();
	double GetY();
	double GetZ();
};

#endif //INPUT_H_P_P