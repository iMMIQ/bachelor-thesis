#include <istream>
#include <ostream>
#include <vector>

using std::istream;
using std::ostream;
using std::vector;

struct Point {
  double x{0}, y{0};
};

struct Point3D {
  double x{0}, y{0}, z{0};

  Point3D() = default;
  Point3D(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  };
  Point3D(double x, Point p) {
    this->x = x;
    this->y = p.x;
    this->z = p.y;
  }

  Point3D operator+(const Point3D &p) { return {x + p.x, y + p.y, z + p.z}; }
  Point3D operator-(const Point3D &p) { return {x - p.x, y - p.y, z - p.z}; }

  friend istream &operator>>(istream &is, Point3D &p);
  friend ostream &operator<<(ostream &os, const Point3D &p);
};

inline istream &operator>>(istream &is, Point3D &p) {
  is >> p.x >> p.y >> p.z;
  return is;
}

inline ostream &operator<<(ostream &os, const Point3D &p) {
  os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return os;
}

struct Line {
  // (x - p1.x) / (p2.x - p1.x) = (y - p1.y) / (p2.y - p1.y) =
  // (z - p1.z) / (p2.z - p1.z)
  Point3D p1, p2;
};

struct Rectangle {
  Point3D LL; // 左下
  Point3D UR; // 右上
  Point3D LR; // 右下
};

/** Unused
struct Cube {
  Rectangle bottom;
  double bottom_z, height;
};
**/

using Plane = vector<Rectangle>;

using Path = vector<Point3D>;