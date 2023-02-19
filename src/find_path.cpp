#include <bits/ranges_algo.h>
#include <map>
#include <queue>
#include <random>

#include <boost/geometry/algorithms/detail/equals/interface.hpp>

#include "basic.h"

using Basic::cross;
using Basic::dot;

using bg::distance;

namespace {
struct DijkstraNode {
  int id;
  double dis;

  bool operator<(const DijkstraNode &rhs) const { return dis > rhs.dis; }
};

struct PointWithRectangleIndex {
  Point3D p;
  int r;
};

auto isParallel(const Line3D &l1, const Line3D &l2) -> bool {
  auto dir1 = l1.second - l1.first;
  auto dir2 = l2.second - l2.first;
  auto c = cross(dir1, dir2);
  return bg::equals(c, Point3D());
}

auto arePointsCollinear(const Point3D &p1, const Point3D &p2, const Point3D &p3,
                        const Point3D &p4) -> bool {
  return bg::equals(cross(p2 - p1, p3 - p1), Point3D()) &&
         bg::equals(cross(p2 - p1, p4 - p1), Point3D());
}

auto calcOverlapLine(const Line3D &l1, const Line3D &l2) -> Line3D {
  if (!arePointsCollinear(l1.first, l1.second, l2.first, l2.second) ||
      bg::equals(l1.first, l1.second) || bg::equals(l2.first, l2.second)) {
    return {};
  }

  vector<Point3D> overlaps_points;
  for (const auto &p : {l1.first, l1.second, l2.first, l2.second}) {
    auto k = (p.x - l1.first.x) / (l1.second.x - l1.first.x);
    if (std::isnan(k)) {
      k = (p.y - l1.first.y) / (l1.second.y - l1.first.y);
    }
    if (std::isnan(k)) {
      k = (p.z - l1.first.z) / (l1.second.z - l1.first.z);
    }
    if (k > -Basic::EPS && k < 1 + Basic::EPS) {
      k = (p.x - l2.first.x) / (l2.second.x - l2.first.x);
      if (std::isnan(k)) {
        k = (p.y - l2.first.y) / (l2.second.y - l2.first.y);
      }
      if (std::isnan(k)) {
        k = (p.z - l2.first.z) / (l2.second.z - l2.first.z);
      }
      if (k > -Basic::EPS && k < 1 + Basic::EPS) {
        overlaps_points.emplace_back(p);
      }
    }
  }

  if (overlaps_points.size() > 1) {
    return {overlaps_points[0], overlaps_points[1]};
  } else {
    return {};
  }
}

auto isRectangleOverlap(const Rectangle3D &r1, const Rectangle3D &r2) -> bool {
  std::array<Point3D, 4> points1{r1.LL, r1.LR, r1.UR, r1.LL + r1.UR - r1.LR},
      points2{r2.LL, r2.LR, r2.UR, r2.LL + r2.UR - r2.LR};

  auto isEdgeOverlap = [](const Point3D &p1, const Point3D &q1,
                          const Point3D &p2, const Point3D &q2) {
    auto line = calcOverlapLine(Line3D(p1, q1), Line3D(p2, q2));
    return distance(line.first, line.second) > Basic::EPS;
  };

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (isEdgeOverlap(points1[i], points1[(i + 1) % 4], points2[j],
                        points2[(j + 1) % 4])) {
        return true;
      }
    }
  }
  return false;
}

auto Dijkstra(const vector<PointWithRectangleIndex> &pris,
              const Rectangle3DList &list) {
  constexpr auto INF = std::numeric_limits<double>::max();
  const int START = pris.size() - 2, END = pris.size() - 1;

  vector<bool> vis(pris.size(), false);
  vector<double> dis(vis.size(), INF);
  vector<int> prev(vis.size(), -1);
  vector<int> path;
  std::priority_queue<DijkstraNode> pq;

  dis[START] = 0;
  pq.push({START, 0});

  while (!pq.empty()) {
    const auto id = pq.top().id;
    pq.pop();
    if (vis[id]) {
      continue;
    }
    vis[id] = true;

    for (int i = 0; i < pris.size(); ++i) {
      if (pris[i].r == pris[id].r ||
          isRectangleOverlap(list[pris[i].r], list[pris[id].r])) {
        if (const auto tmp = distance(pris[i].p, pris[id].p);
            dis[i] > dis[id] + tmp) {
          dis[i] = dis[id] + tmp;
          prev[i] = id;
          pq.push({i, dis[i]});
        }
      }
    }
  }

  int cur = prev[END];
  while (cur != START && cur != -1) {
    path.push_back(cur);
    cur = prev[cur];
  }
  std::reverse(path.begin(), path.end());
  return path;
}
} // namespace

inline auto find_path(const Rectangle3DList &list, const Point3D &start,
                      const Point3D &end) {
  constexpr auto block_size = 1.0;
  std::mt19937_64 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{0.0, 1.0};
  vector<PointWithRectangleIndex> points;
  std::map<int, int> pointToRectangleIndex;

  for (int i = 0; i < list.size(); ++i) {
    const auto &r = list[i];
    for (int j = 0;
         j < distance(r.LL, r.LR) * distance(r.LR, r.UR) / block_size; ++j) {
      const auto x = dist(rng), y = dist(rng);
      const auto point = (r.LL - r.LR) * x + (r.UR - r.LR) * y + r.LR;
      points.push_back({point, i});
      pointToRectangleIndex[points.size() - 1] = i;
    }
  }

  const int start_rectangle =
      std::ranges::find_if(list.begin(), list.end(),
                           [&start](const auto &r) {
                             return Basic::isPointInsideRectangle3D(start, r);
                           }) -
      list.begin();

  const int end_rectangle =
      std::ranges::find_if(list.begin(), list.end(),
                           [&end](const auto &r) {
                             return Basic::isPointInsideRectangle3D(end, r);
                           }) -
      list.begin();

  points.push_back({start, start_rectangle});
  points.push_back({end, end_rectangle});

  const auto path = Dijkstra(points, list);
  vector<int> indexs{start_rectangle};
  for (const auto i : path) {
    indexs.push_back(pointToRectangleIndex[i]);
  }
  indexs.push_back(end_rectangle);

  std::pair<Point3D, Point3D> last_line;
  int last_rect_index = -1;
  for (auto i : indexs) {
  }
  return indexs;
}