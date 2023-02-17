#include <bits/ranges_algo.h>
#include <map>
#include <queue>
#include <random>

#include "basic.h"

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

auto isEdgeOverlap(Point3D p1, Point3D q1, Point3D p2, Point3D q2) -> bool {
  const auto d1 = q1 - p1;
  const auto d2 = q2 - p2;

  auto n = Basic::cross(d1, d2);
  auto dist = Basic::dot((p2 - p1), n) / Basic::dot(d1, n);

  if (const auto cross = Basic::cross(d1, d2);
      std::hypot(cross.x, cross.y, cross.z) < Basic::EPS) {
    if (Basic::dot((p2 - p1), d1) < 0 || Basic::dot((p2 - q1), d1) > 0 ||
        Basic::dot((q2 - p1), d1) < 0 || Basic::dot((q2 - q1), d1) > 0 ||
        Basic::dot((p1 - p2), d2) < 0 || Basic::dot((p1 - q2), d2) > 0 ||
        Basic::dot((q1 - p2), d2) < 0 || Basic::dot((q1 - q2), d2) > 0) {
      return false;
    }
    return true;
  }
  return false;
}

auto isRectangleOverlap(const Rectangle3D &r1, const Rectangle3D &r2) -> bool {
  std::array<Point3D, 4> points1{r1.LL, r1.LR, r1.UR, r1.LL + r1.UR - r1.LR},
      points2{r2.LL, r2.LR, r2.UR, r2.LL + r2.UR - r2.LR};

  for (int i = 0; i < 4; ++i) {
    if (isEdgeOverlap(points1[i], points1[(i + 1) % 4], points2[i],
                      points2[(i + 1) % 4])) {
      return true;
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
      if (isRectangleOverlap(list[pris[i].r], list[pris[id].r])) {
        if (const auto tmp = Basic::distance(pris[i].p, pris[id].p);
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
}; // namespace

inline auto find_path(const Rectangle3DList &list, const Point3D &start,
                      const Point3D &end) {
  constexpr auto block_size = 1.0;
  std::mt19937_64 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist{0.0, 1.0};
  vector<PointWithRectangleIndex> points;
  std::map<int, int> pointToRectangleIndex;

  for (int i = 0; i < list.size(); ++i) {
    const auto &r = list[i];
    for (int j = 0; j < Basic::distance(r.LL, r.LR) *
                            Basic::distance(r.LR, r.UR) / block_size;
         ++j) {
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
  return indexs;
}