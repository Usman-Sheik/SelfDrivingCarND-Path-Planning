#ifndef PATH_PLANNING_ROAD_H
#define PATH_PLANNING_ROAD_H

#include <map>
#include <string>
#include <vector>

#include "i_data_source.h"
#include "i_map.h"
#include "object.h"
#include "road_configuration.h"
#include "vehicle.h"

namespace sdc {
namespace highway_driving {

class Road : public IEnvironmentData {
public:
  explicit Road(const IMap &map, const RoadConfiguration &road_config);
  ~Road() = default;

  void update_environment(const std::vector<Object> &object_list);
  void
  update_ego_vehicle(const FrenetPoint &frenet, const CartesianPoint &cartesian,
                     double speed,
                     const std::vector<std::vector<double>> &prev_trajectory);

  std::vector<std::vector<double>> get_ego_trajectory();

  const RoadConfiguration &get_road_configuration() const override {
    return road_configuration_;
  }

  const std::map<int, std::vector<Object>> &get_object_list() const override {
    return object_map_;
  };

  const std::vector<std::vector<double>> &get_waypoints() const override {
    return map_.get_waypoints();
  };

  const double get_wrap_point() const override { return map_.get_wrap_point(); }

  void step();

private:
  void process_objects(std::vector<Object> object_list);
  void clear_object_list();
  void debug_object_list();

  const IMap &map_;
  RoadConfiguration road_configuration_{};
  Vehicle ego_vehicle_;
  std::map<int, std::vector<Object>> object_map_{};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_ROAD_H
