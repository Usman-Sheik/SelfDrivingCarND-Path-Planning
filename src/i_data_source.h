#ifndef PATH_PLANNING_I_DATA_SOURCE_H
#define PATH_PLANNING_I_DATA_SOURCE_H

#include <map>
#include <vector>

#include "object.h"
#include "road_configuration.h"

namespace sdc {
namespace highway_driving {

class IEnvironmentData {
public:
  virtual const RoadConfiguration &get_road_configuration() const = 0;
  virtual const std::map<int, std::vector<Object>> &get_object_list() const = 0;
  virtual const std::vector<std::vector<double>> &get_waypoints() const = 0;
  virtual const double get_wrap_point() const = 0;
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_I_DATA_SOURCE_H
