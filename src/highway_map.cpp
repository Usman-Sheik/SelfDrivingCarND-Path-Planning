#include "highway_map.h"
#include "logger.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace sdc {
namespace highway_driving {

HighwayMap::HighwayMap(const std::string &map_data, const double max_s)
    : waypoints_{5}, splines_{5}, max_s_{max_s} {
  load_data(map_data);
}

void HighwayMap::load_data(const std::string &map_data) {

  std::ifstream in_map_(map_data.c_str(), std::ifstream::in);
  std::string line{};

  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x{}, y{}, s{}, d_x{}, d_y{};
    iss >> x >> y >> s >> d_x >> d_y;
    waypoints_[kXWaypointIndice].push_back(x);
    waypoints_[kYWaypointIndice].push_back(y);
    waypoints_[kSWaypointIndice].push_back(s);
    waypoints_[kDxWaypointIndice].push_back(d_x);
    waypoints_[kDyWaypointIndice].push_back(d_y);
  }

  splines_[kXWaypointIndice].set_points(waypoints_[kSWaypointIndice],
                                        waypoints_[kXWaypointIndice]);
  splines_[kYWaypointIndice].set_points(waypoints_[kSWaypointIndice],
                                        waypoints_[kYWaypointIndice]);
  splines_[kDxWaypointIndice].set_points(waypoints_[kSWaypointIndice],
                                         waypoints_[kDxWaypointIndice]);
  splines_[kDyWaypointIndice].set_points(waypoints_[kSWaypointIndice],
                                         waypoints_[kDyWaypointIndice]);
}

} // namespace highway_driving
} // namespace sdc
