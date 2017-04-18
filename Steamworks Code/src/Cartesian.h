struct Vector2 {
  public:
    Vector2(double dx, double dz){
    	this->dx = dx;
    	this->dz = dz;
    }
    double dx, dz;
};
struct Point {
  public:
    typedef enum{Polar, Cart}CoordinateType;

    explicit Point(Point* p){
    	*this = *p;
    }

    Point(double a, double b, CoordinateType ct = Cart){
    	this->x = x;
    	this->z = z;

      if(coord_T == Polar) {
        r = a;
        t = b;
        UpdateCartesian();
      }
      else if(coord_T == Cart) {
        x = a;
        z = b;
        UpdatePolar();
      }
    }

    double GetX() {
      return x;
    }

    double GetZ() {
      return z;
    }

    double GetR() {
      return r;
    }

    double GetTheta() {
      return t;
    }

    Point operator=(Point p) {
      x = p.x;
      z = p.z;
      r = p.r;
      t = p.t;
      return *this;
    }

    Point operator+(Point p) {
      return Point(
          p.x + this->x,
          p.z + this->z
      );
    }

    Point operator+=(Point p) {
      *this = *(new Point(
          p.x + this->x,
          p.z + this->z
      ));
      return *this;
    }
  protected:
    friend Point;
    CoordinateType coord_T;
    double x, z, r, t;

    void UpdatePolar() {
      r = sqrt(x*x + z*z);
      t = atan(z/x);
    }

    void UpdateCartesian() {
      x = r * cos(t);
      z = r * sin(t);
    }
};

/*struct Vector2 : public Point;*/
