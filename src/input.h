#ifndef WIRING_INPUT_H
#define WIRING_INPUT_H

#include "data.h"

inline namespace Input {
struct Triangle {
  Point3D A, B, C;

  explicit Triangle(Point3D a, Point3D b, Point3D c) : A(a), B(b), C(c) {}
};

auto read_model(const std::string &file) -> vector<Triangle>;

auto transform_tri2rect3D(const vector<Triangle> &triangles) -> Rectangle3DList;
} // namespace Input

#endif // WIRING_INPUT_H
