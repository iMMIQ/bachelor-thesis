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

auto Dijkstra(const vector<PointWithRectangleIndex> &pris,
              const vector<vector<bool>> &edge) -> vector<int>;
auto rand_0_to_1() -> double;
auto lerp_point3D(const Point3D &a, const Point3D &b, double t) -> Point3D;
auto calc_path(const Rectangle3DList &list,
               const vector<vector<int>> &rectangles, const Point3D &start,
               const Point3D &end, const vector<Line3D> &lines,
               const vector<double> &input) -> std::pair<Path3D, double>;
auto find_rectangle_index(const Rectangle3DList &list, const Point3D &start,
                          const Point3D &end) -> vector<int>;
auto simulated_annealing(Rectangle3DList &list,
                         const vector<vector<int>> &rectangles,
                         const Point3D &start, const Point3D &end,
                         const vector<Line3D> &lines, int iterations,
                         double initial_temp, double cooling_rate)
    -> vector<double>;
} // namespace FindPath

auto find_path(Rectangle3DList &list, const Point3D &start, const Point3D &end)
    -> std::pair<Path3D, double>;

#endif // WIRING_FIND_PATH_H
