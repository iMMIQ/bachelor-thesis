#include <iostream>

#include "find_path.h"
#include "input.h"

using std::cin;
using std::cout;
using std::endl;

auto main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) -> int {
  auto plane =
      Input::transform_tri2rect3D(Input::read_model("../test/ball.stl"));
  Point3D start = plane.front().LL;
  Point3D end = plane.back().LL;
  //  cin >> start >> end;
  auto ans = find_path(plane, start, end);
  cout << "path:\n";
  for (const auto &i : ans.first) {
    cout << i << endl;
  }
  cout << "distance: " << ans.second << endl;
  return 0;
}