#include <istream>
#include <ostream>
#include <vector>

using std::istream;
using std::ostream;
using std::vector;

struct Point {
  double x{0}, y{0};

  friend ostream &operator<<(ostream &os, const Point &p);
};

inline ostream &operator<<(ostream &os, const Point &p) {
  os << "(" << p.x << ", " << p.y << ")";
  return os;
}

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

  Point3D operator+(const Point3D &p) const {
    return {x + p.x, y + p.y, z + p.z};
  }
  Point3D operator-(const Point3D &p) const {
    return {x - p.x, y - p.y, z - p.z};
  }

  Point3D operator*(const double a) const { return {a * x, a * y, a * z}; }

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
  Point LL; // 左下
  Point UR; // 右上
};

struct Rectangle3D {
  Point3D LL; // 左下
  Point3D UR; // 右上
  Point3D LR; // 右下
};

using Plane = vector<Rectangle3D>;

using Path = vector<Point>;
using Path3D = vector<Point3D>;
