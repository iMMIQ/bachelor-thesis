#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <iostream>

#include "input.h"
#include "solve.h"

inline namespace Input {
auto read_model(const std::string &file) -> vector<Triangle> {
  Assimp::Importer importer;

  const auto *scene = importer.ReadFile(file, aiProcess_Triangulate);

  vector<Triangle> result;

  for (int i = 0; i < scene->mNumMeshes; ++i) {
    const auto *mesh = scene->mMeshes[i];

    for (int j = 0; j < mesh->mNumFaces; ++j) {
      const auto &face = mesh->mFaces[j];

      const auto t = Triangle(Point3D(mesh->mVertices[face.mIndices[0]].x,
                                      mesh->mVertices[face.mIndices[0]].y,
                                      mesh->mVertices[face.mIndices[0]].z),
                              Point3D(mesh->mVertices[face.mIndices[1]].x,
                                      mesh->mVertices[face.mIndices[1]].y,
                                      mesh->mVertices[face.mIndices[1]].z),
                              Point3D(mesh->mVertices[face.mIndices[2]].x,
                                      mesh->mVertices[face.mIndices[2]].y,
                                      mesh->mVertices[face.mIndices[2]].z));

      result.push_back(t);
    }
  }

  return result;
}

auto isPerpendicular(const Line3D &a, const Line3D &b) -> bool {
  return std::abs(Solve::dot(a.first - a.second, b.first - b.second)) < EPS;
}

auto isRectangle(const Triangle &a, const Triangle &b) -> bool {
  const auto va = {a.A, a.B, a.C};
  const auto vb = {b.A, b.B, b.C};
  Point3D same_point1, same_point2;
  int same_count = 0;
  for (const auto &p : va) {
    for (const auto &q : vb) {
      if (p == q) {
        switch (same_count) {
        case 0:
          same_point1 = p;
          break;
        case 1:
          same_point2 = p;
          break;
        default:
          return false;
        }
        same_count++;
        break;
      }
    }
  }
  if (same_count != 2) {
    return false;
  }
  Point3D a_point, b_point;
  for (const auto &p : va) {
    if (p != same_point1 && p != same_point2) {
      a_point = p;
      break;
    }
  }
  for (const auto &p : vb) {
    if (p != same_point1 && p != same_point2) {
      b_point = p;
      break;
    }
  }
  if (!isPerpendicular(Line3D(same_point1, a_point),
                       Line3D(same_point2, a_point)) ||
      !isPerpendicular(Line3D(same_point1, b_point),
                       Line3D(same_point2, b_point))) {
    return false;
  }
  return Solve::areCoplanar(same_point1, same_point2, a_point, b_point);
}

auto transform_tri2rect3D(const vector<Triangle> &triangles)
    -> Rectangle3DList {
  Rectangle3DList res;
  auto n = triangles.size();
  vector<bool> vis(n, false);
  for (int i = 0; i < n; ++i) {
    if (vis[i]) {
      continue;
    }
    for (int j = i + 1; j < n; ++j) {
      if (vis[j]) {
        continue;
      }
      if (isRectangle(triangles[i], triangles[j])) {
        if (isPerpendicular(Line3D(triangles[i].B, triangles[i].A),
                            Line3D(triangles[i].C, triangles[i].A))) {
          res.emplace_back(triangles[i].B, triangles[i].C, triangles[i].A);
        } else if (isPerpendicular(Line3D(triangles[i].A, triangles[i].B),
                                   Line3D(triangles[i].C, triangles[i].B))) {
          res.emplace_back(triangles[i].A, triangles[i].C, triangles[i].B);
        } else {
          res.emplace_back(triangles[i].A, triangles[i].B, triangles[i].C);
        }
        vis[i] = vis[j] = true;
        break;
      }
    }
  }

  auto mid_point = [](const Point3D &a, const Point3D &b) {
    return Point3D((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
  };

  auto calc_triangle_area = [](const Triangle &t) {
    auto a = distance(t.A, t.B);
    auto b = distance(t.B, t.C);
    auto c = distance(t.C, t.A);
    auto s = (a + b + c) / 2;
    return sqrt(s * (s - a) * (s - b) * (s - c));
  };

  constexpr auto MIN_AREA = 0.1;
  for (int i = 0; i < n; ++i) {
    if (!vis[i]) {
      std::queue<Triangle> q;
      q.push(triangles[i]);
      while (!q.empty()) {
        auto t = q.front();
        q.pop();
        if (calc_triangle_area(t) > MIN_AREA) {
          auto LR = mid_point(t.A, t.B);
          auto LL = mid_point(t.A, t.C);
          auto UR = Solve::point_projection(LR, Line3D(t.B, t.C));
          auto UL = Solve::point_projection(LL, Line3D(t.B, t.C));
          res.emplace_back(LL, UR, LR);
          q.emplace(t.A, LR, LL);
          q.emplace(t.B, UR, LR);
          q.emplace(t.C, UL, LL);
        }
      }
    }
  }

  auto calc_2line_dis = [](const Line3D &l1, const Line3D &l2) -> double {
    auto p = Solve::point_projection(l1.first, l2);
    if (Solve::is_point_in_segment(p, l2)) {
      return distance(p, l1.first);
    }
    p = Solve::point_projection(l1.second, l2);
    if (Solve::is_point_in_segment(p, l2)) {
      return distance(p, l1.second);
    }
    p = Solve::point_projection(l2.first, l1);
    if (Solve::is_point_in_segment(p, l1)) {
      return distance(p, l2.first);
    }
    p = Solve::point_projection(l2.second, l1);
    if (Solve::is_point_in_segment(p, l1)) {
      return distance(p, l2.second);
    }
    return std::numeric_limits<double>::max();
  };

  auto calc_rectangle_area = [](const Rectangle3D &r) {
    return distance(r.LL, r.LR) * distance(r.UR, r.LR);
  };

  for (auto &r1 : res) {
    for (auto &r2 : res) {
      if (&r1 != &r2) {
        std::array<Point3D, 4> ps1{r1.LL, r1.LR, r1.UR, r1.LL + r1.UR - r1.LR};
        std::array<Point3D, 4> ps2{r2.LL, r2.LR, r2.UR, r2.LL + r2.UR - r2.LR};
        std::array<Line3D, 4> ls1{
            Line3D(ps1[0], ps1[1]), Line3D(ps1[1], ps1[2]),
            Line3D(ps1[2], ps1[3]), Line3D(ps1[3], ps1[0])};
        std::array<Line3D, 4> ls2{
            Line3D(ps2[0], ps2[1]), Line3D(ps2[1], ps2[2]),
            Line3D(ps2[2], ps2[3]), Line3D(ps2[3], ps2[0])};
        for (auto &l1 : ls1) {
          for (auto &l2 : ls2) {
            if (Solve::isParallel(l1, l2) && calc_2line_dis(l1, l2) < 0.1) {
              if (calc_rectangle_area(r1) < calc_rectangle_area(r2)) {
                vector<Point3D> last;
                for (auto &i : ps1) {
                  if (i != l1.first && i != l1.second) {
                    last.push_back(i);
                  }
                }
                Rectangle3D tmp;
                tmp.UR = Solve::point_projection(last[0], l2);
                tmp.LR = last[0];
                tmp.LL = last[1];
                r1 = tmp;
              } else {
                vector<Point3D> last;
                for (auto &i : ps2) {
                  if (i != l2.first && i != l2.second) {
                    last.push_back(i);
                  }
                }
                Rectangle3D tmp;
                tmp.UR = Solve::point_projection(last[0], l1);
                tmp.LR = last[0];
                tmp.LL = last[1];
                r2 = tmp;
              }
              break;
            }
          }
        }
      }
    }
  }
  return res;
}
} // namespace Input
