#ifndef PATH_PLANNING_ROAD_CONFIGURATION_H
#define PATH_PLANNING_ROAD_CONFIGURATION_H

#include <vector>

namespace sdc {
namespace highway_driving {

struct RoadConfiguration {
  int num_lanes{0};
  double lane_width{0.};
  double speed_limit{0.};
  std::vector<int> lane_ids{};
};

inline int get_lane_number(const double d,
                           const RoadConfiguration &road_configuration) {
  return static_cast<int>(d / road_configuration.lane_width);
}

inline double get_lane_center(int lane,
                              const RoadConfiguration &road_configuration) {
  return lane * road_configuration.lane_width +
         road_configuration.lane_width / 2.;
}

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_ROAD_CONFIGURATION_H
