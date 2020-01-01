#include "road.h"
#include "logger.h"
#include <algorithm>

namespace sdc {
namespace highway_driving {

Road::Road(const IMap &map, const RoadConfiguration &road_config)
    : map_{map}, road_configuration_{road_config},
      ego_vehicle_{FrenetPoint{}, CartesianPoint{}, 0, 0.} {

  Logger::get_logger() << "Road Configuration => No. of lanes: "
                       << road_config.num_lanes
                       << " lane_width: " << road_config.lane_width
                       << " speed_limit: " << road_config.speed_limit
                       << " Lane Ids: ";
  for (const auto &id : road_config.lane_ids) {
    Logger::get_logger() << id << " ";
  }

  Logger::get_logger() << "\n";
}

void Road::update_environment(const std::vector<Object> &object_list) {
  process_objects(object_list);
}

void Road::process_objects(std::vector<Object> object_list) {

  clear_object_list();

  for (const auto &object : object_list) {

    const int lane_id{
        get_lane_number(object.frenet_point.d, road_configuration_)};

    bool lane_is_not_valid{
        road_configuration_.lane_ids.end() ==
        std::find_if(road_configuration_.lane_ids.begin(),
                     road_configuration_.lane_ids.end(),
                     [&lane_id](const auto &id) { return (id == lane_id); })};

    if (!lane_is_not_valid) {

      const auto iterator = object_map_.find(lane_id);

      if (iterator != object_map_.end()) {
        auto &objects{iterator->second};
        objects.emplace_back(object);
      } else {
        object_map_[lane_id] = {object};
      }
    }
  }

  debug_object_list();
}

void Road::debug_object_list() {

  for (const auto &iterator : object_map_) {
    const auto &lane_id{iterator.first};
    const auto &objects{iterator.second};

    Logger::get_logger() << "Lane \"" << lane_id << "\" has \""
                         << objects.size() << "\" objects \n";

    for (const auto &object : objects) {
      Logger::get_logger() << "object id: " << object.id
                           << " x: " << object.cartesian_point.x
                           << " y: " << object.cartesian_point.y
                           << " theta: " << object.cartesian_point.theta
                           << " s: " << object.frenet_point.s
                           << " d: " << object.frenet_point.d
                           << " speed: " << object.speed << "\n";
    }
  }
}

void Road::clear_object_list() { object_map_.clear(); }

void Road::update_ego_vehicle(
    const FrenetPoint &frenet, const CartesianPoint &cartesian, double speed,
    const std::vector<std::vector<double>> &prev_trajectory) {
  ego_vehicle_.UpdateState(cartesian, frenet, speed, prev_trajectory, *this);
}

void Road::step() { ego_vehicle_.choose_next_state(*this); }

std::vector<std::vector<double>> Road::get_ego_trajectory() {
  return ego_vehicle_.get_trajectory(*this);
}

} // namespace highway_driving
} // namespace sdc
