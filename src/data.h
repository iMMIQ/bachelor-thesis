#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/segment.hpp>

using std::vector;

namespace bg = boost::geometry;

struct Point {
  double x, y;

  explicit Point(double x = 0, double y = 0) : x(x), y(y) {}

  auto operator+(const Point &other) const -> Point {
    return Point(x + other.x, y + other.y);
  }

  auto operator-(const Point &other) const -> Point {
    return Point(x - other.x, y - other.y);
  }

  auto operator*(double scalar) const -> Point {
    return Point(x * scalar, y * scalar);
  }

  friend auto operator>>(std::istream &in, Point &p) -> std::istream & {
    in >> p.x >> p.y;
    return in;
  }

  friend auto operator<<(std::ostream &out, const Point &p) -> std::ostream & {
    out << "(" << p.x << ", " << p.y << ")";
    return out;
  }
};

namespace boost::geometry::traits {
template <> struct tag<Point> {
  using type = point_tag;
};
template <> struct coordinate_type<Point> {
  using type = double;
};
template <> struct coordinate_system<Point> {
  using type = cs::cartesian;
};
template <> struct dimension<Point> : boost::mpl::int_<2> {};
template <> struct access<Point, 0> {
  static auto get(Point const &p) -> double { return p.x; }
  static void set(Point &p, double const &value) { p.x = value; }
};
template <> struct access<Point, 1> {
  static auto get(Point const &p) -> double { return p.y; }
  static void set(Point &p, double const &value) { p.y = value; }
};
} // namespace boost::geometry::traits

struct Point3D {
  double x, y, z;

  explicit Point3D(double x = 0, double y = 0, double z = 0)
      : x(x), y(y), z(z) {}

  auto operator+(const Point3D &other) const -> Point3D {
    return Point3D(x + other.x, y + other.y, z + other.z);
  }

  auto operator-(const Point3D &other) const -> Point3D {
    return Point3D(x - other.x, y - other.y, z - other.z);
  }

  auto operator*(double scalar) const -> Point3D {
    return Point3D(x * scalar, y * scalar, z * scalar);
  }

  friend auto operator>>(std::istream &in, Point3D &p) -> std::istream & {
    in >> p.x >> p.y >> p.z;
    return in;
  }

  friend auto operator<<(std::ostream &out, const Point3D &p)
      -> std::ostream & {
    out << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    return out;
  }
};

namespace boost::geometry::traits {
template <> struct tag<Point3D> {
  using type = point_tag;
};
template <> struct coordinate_type<Point3D> {
  using type = double;
};
template <> struct coordinate_system<Point3D> {
  using type = cs::cartesian;
};
template <> struct dimension<Point3D> : boost::mpl::int_<3> {};
template <> struct access<Point3D, 0> {
  static auto get(Point3D const &p) -> double { return p.x; }
  static void set(Point3D &p, double const &value) { p.x = value; }
};
template <> struct access<Point3D, 1> {
  static auto get(Point3D const &p) -> double { return p.y; }
  static void set(Point3D &p, double const &value) { p.y = value; }
};
template <> struct access<Point3D, 2> {
  static auto get(Point3D const &p) -> double { return p.z; }
  static void set(Point3D &p, double const &value) { p.z = value; }
};
} // namespace boost::geometry::traits

using Line3D = bg::model::segment<Point3D>;

struct Rectangle {
  Point LL; // 左下
  Point UR; // 右上
};

struct Rectangle3D {
  Point3D LL; // 左下
  Point3D UR; // 右上
  Point3D LR; // 右下
};

using Rectangle3DList = vector<Rectangle3D>;
using Plane = Rectangle3DList;

using Path = vector<Point>;
using Path3D = vector<Point3D>;
