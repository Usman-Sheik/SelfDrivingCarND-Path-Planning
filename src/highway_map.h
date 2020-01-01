#ifndef PATH_PLANNING_HIGHWAY_MAP_H
#define PATH_PLANNING_HIGHWAY_MAP_H

#include <string>
#include <vector>

#include "cartesian_point.h"
#include "i_map.h"
#include "spline.h"

namespace sdc {
namespace highway_driving {

class HighwayMap : public IMap {
public:
  explicit HighwayMap(const std::string &map_data, const double max_s = 0.);

  const std::vector<std::vector<double>> &get_waypoints() const {
    return waypoints_;
  }

  const double get_wrap_point() const override { return max_s_; }

private:
  void load_data(const std::string &map_data);

  static constexpr int kXWaypointIndice{0};
  static constexpr int kYWaypointIndice{1};
  static constexpr int kSWaypointIndice{2};
  static constexpr int kDxWaypointIndice{3};
  static constexpr int kDyWaypointIndice{4};

  std::vector<std::vector<double>> waypoints_{};
  std::vector<::tk::spline> splines_{};
  double max_s_{0.};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_HIGHWAY_MAP_H
