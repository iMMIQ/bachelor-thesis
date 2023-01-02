#include <vector>

using std::vector;

struct Point3D {
  double x{0}, y{0}, z{0};
};

struct Line {
  // (x - p1.x) / (p2.x - p1.x) = (y - p1.y) / (p2.y - p1.y) =
  // (z - p1.z) / (p2.z - p1.z)
  Point3D p1, p2;
};

struct Rectangle {
  Point3D UL, LR;
};

struct Cube {
  Rectangle bottom;
  double bottom_z, height;
};

using Plane = vector<Rectangle>;

using Path = vector<Point3D>;