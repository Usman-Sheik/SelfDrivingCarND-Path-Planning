#ifndef PATH_PLANNING_I_MAP_H
#define PATH_PLANNING_I_MAP_H

#include <vector>

namespace sdc {
namespace highway_driving {
class IMap {
public:
  virtual const std::vector<std::vector<double>> &get_waypoints() const = 0;
  virtual const double get_wrap_point() const = 0;
};
} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_I_MAP_H
