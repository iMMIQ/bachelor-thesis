#ifndef WIRING_FIND_PATH_H
#define WIRING_FIND_PATH_H

#include "solve.h"

inline namespace FindPath {
struct DijkstraNode {
  int id;
  double dis;

  auto operator<(const DijkstraNode &rhs) const -> bool {
    return dis > rhs.dis;
  }
};

struct PointWithRectangleIndex {
  Point3D p;
  int r{};
};

auto isParallel(const Line3D &l1, const Line3D &l2) -> bool;
auto arePointsCollinear(const Point3D &p1, const Point3D &p2, const Point3D &p3,
                        const Point3D &p4) -> bool;
auto calcOverlapLine(const Line3D &l1, const Line3D &l2) -> Line3D;
auto calcOverlapRectangle(const Rectangle3D &r1, const Rectangle3D &r2)
    -> Line3D;
auto Dijkstra(const vector<PointWithRectangleIndex> &pris,
              const Rectangle3DList &list) -> vector<int>;
auto rand_0_to_1() -> double;
auto lerp_point3D(const Point3D &a, const Point3D &b, double t) -> Point3D;
auto calc_path(const Rectangle3DList &list,
               const vector<vector<int>> &rectangles, const Point3D &start,
               const Point3D &end, const vector<Line3D> &lines,
               const vector<double> &input) -> std::pair<Path3D, double>;
auto find_rectangle_index(const Rectangle3DList &list, const Point3D &start,
                           const Point3D &end) -> vector<int>;
auto simulated_annealing(const Rectangle3DList &list,
                         const vector<vector<int>> &rectangles,
                         const Point3D &start, const Point3D &end,
                         const vector<Line3D> &lines, int iterations,
                         double initial_temp, double cooling_rate)
    -> vector<double>;
} // namespace FindPath

auto find_path(const Rectangle3DList &list, const Point3D &start,
               const Point3D &end) -> std::pair<Path3D, double>;

#endif // WIRING_FIND_PATH_H
