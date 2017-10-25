#ifndef POINT_H_P_P
#define POINT_H_P_P

class Point {
	
	public:
		typedef enum{
			Polar,
			Cart
		}CoordinateType;
	
		double GetX();
		double GetY();
		double GetZ();
		double GetR();
		double GetTheta();
		
		Point operator=(Point&);
		Point operator+(Point&);
		Point operator+=(Point&);
		
		explicit Point(const Point&);
		Point(double a, double b, double c, CoordinateType ct = Cart);
		
	protected:
		friend Point;
		double x,y,z,r,t;
		void UpdatePolar();
		void UpdateCartesian();
};

#endif // POINT_H_P_P
