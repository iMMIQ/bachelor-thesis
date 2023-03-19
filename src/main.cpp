#include <iostream>

#include "find_path.h"
#include "input.h"

using std::cin;
using std::cout;
using std::endl;

auto main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) -> int {
  //  int n = 0;
  //  cin >> n;
  //  Plane plane(n, Rectangle3D());
  //  for (int i = 0; i < n; ++i) {
  //    cin >> plane[i].LL >> plane[i].UR >> plane[i].LR;
  //  }
  auto plane = Input::transform_tri2rect3D(
      Input::read_model("../test/Cube_3d_printing_sample.stl"));
  Point3D start(-55, 60, 0);
  Point3D end(-55, 40, 20);
  //  cin >> start >> end;
  auto ans = find_path(plane, start, end);
  cout << "path:\n";
  for (const auto &i : ans.first) {
    cout << i << endl;
  }
  cout << "distance: " << ans.second << endl;
  return 0;
}