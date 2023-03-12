#ifndef WIRING_INPUT_H
#define WIRING_INPUT_H

#include "data.h"

inline namespace Input {
struct Triangle {
  Point3D A, B, C;

  explicit Triangle(Point3D a, Point3D b, Point3D c) : A(a), B(b), C(c) {}
};

auto read_stl(const std::string &file) -> vector<Triangle>;
} // namespace Input

#endif // WIRING_INPUT_H
