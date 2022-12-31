#include <vector>

using std::vector;

struct Point {
  double x, y;
};

struct Rectangle {
  Point UL, LR;
};

struct Point3D {
  double x, y, z;
};

struct Cube {
  Rectangle bottom;
  double bottom_z, height;
};

using Plane = vector<Rectangle>;

using Path = vector<Point>;