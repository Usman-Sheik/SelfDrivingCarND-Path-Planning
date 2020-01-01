#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "cost.h"
#include "logger.h"
#include "vehicle.h"

namespace sdc {
namespace highway_driving {

Vehicle::Vehicle(const FrenetPoint &frenet,
                 const CartesianPoint &cartesian_point, int lane, double speed)
    : frenet_point_{frenet},
      cartesian_point_{cartesian_point}, lane_{lane}, speed_{speed} {}

void Vehicle::UpdateState(
    const CartesianPoint &cartesian_point, const FrenetPoint &frenet_point,
    const double speed, const std::vector<std::vector<double>> &prev_trajectory,
    const IEnvironmentData &data) {

  const auto &road_configuration{data.get_road_configuration()};

  speed_limit_ = road_configuration.speed_limit;
  cartesian_point_ = cartesian_point;
  frenet_point_ = frenet_point;
  lane_ = static_cast<int>(frenet_point_.d / road_configuration.lane_width);
  speed_ = speed;
  trajectory_.update_previous_path(prev_trajectory);

  Logger::get_logger() << "New Cartesian coordinates => x: "
                       << cartesian_point.x << " y: " << cartesian_point.y
                       << " theta: " << cartesian_point.theta << "\n";
  Logger::get_logger() << "New Frenet coordinates => s: " << frenet_point.s
                       << " d: " << frenet_point.d << "\n";
  Logger::get_logger() << "Lane number: " << lane_
                       << " New speed limit: " << speed_limit_
                       << " current speed of ego vehicle: " << speed_ << "\n";
}

bool Vehicle::update_object_from_behind(int lane,
                                        sdc::highway_driving::Object object,
                                        double new_distance) {

  const auto &existing_object_from_behind{
      nearby_objects_[lane].object_from_behind};

  const double existing_distance{existing_object_from_behind.frenet_point.s -
                                 frenet_point_.s};

  if (std::fabs(existing_distance) > new_distance) {
    nearby_objects_[lane].object_from_behind = object;
    Logger::get_logger() << "Lane \"" << lane << "\" has a new object \""
                         << object.id << "\" from behind at: " << new_distance
                         << " meters"
                         << "\n";
  }
}

bool Vehicle::update_object_in_front(int lane,
                                     sdc::highway_driving::Object object,
                                     double new_distance) {

  const auto &existing_object_in_front{nearby_objects_[lane].object_in_front};

  const double existing_distance{existing_object_in_front.frenet_point.s -
                                 frenet_point_.s};

  if (std::fabs(existing_distance) > new_distance) {
    nearby_objects_[lane].object_in_front = object;
    Logger::get_logger() << "Lane \"" << lane << "\" has a new object \""
                         << object.id << "\" in front at: " << new_distance
                         << " meters"
                         << "\n";
  }
}

void Vehicle::reset_nearby_objects(int num_lanes) {
  nearby_objects_.clear();
  nearby_objects_.resize(num_lanes);
}

void Vehicle::update_nearby_objects(
    const sdc::highway_driving::IEnvironmentData &data) {

  const auto &road_configuration{data.get_road_configuration()};
  const std::map<int, std::vector<Object>> &object_map{data.get_object_list()};
  std::vector<Object> objects{};

  reset_nearby_objects(road_configuration.num_lanes);

  for (const auto &itr : object_map) {
    for (const auto &object : itr.second) {
      objects.emplace_back(object);
    }
  }

  for (const auto &object_itr : object_map) {
    const auto &lane_id{object_itr.first};
    const auto &object_list{object_itr.second};

    for (const auto &object : object_list) {
      const auto s_distance{object.frenet_point.s - frenet_point_.s};

      if (s_distance < 0.) {
        update_object_from_behind(lane_id, object, std::fabs(s_distance));
      } else {
        update_object_in_front(lane_id, object, s_distance);
      }
    }
  }
}

ManeuverType Vehicle::get_maneuver_type_from_state(const std::string &state) {
  if (!state.compare("INIT")) {
    return ManeuverType::INIT;
  } else if (!state.compare("KL")) {
    return ManeuverType::KEEP_LANE;
  } else if (!state.compare("PLCL")) {
    return ManeuverType::PREPARE_LANE_CHANGE_LEFT;
  } else if (!state.compare("LCL")) {
    return ManeuverType::LANE_CHANGE_LEFT;
  } else if (!state.compare("PLCR")) {
    return ManeuverType::PREPARE_LANE_CHANGE_RIGHT;
  } else if (!state.compare("LCR")) {
    return ManeuverType::LANE_CHANGE_RIGHT;
  }
}

Maneuver Vehicle::prepare_maneuvers(const std::string &state,
                                    const RoadConfiguration &road_config) {
  Maneuver maneuver{};

  maneuver.maneuver_type = get_maneuver_type_from_state(state);
  if (!state.compare("INIT")) {
    target_speed_ += 0.4;
    maneuver.speed = target_speed_;
    maneuver.lane_id = lane_;
  } else if (!state.compare("KL")) {
    maneuver.speed = road_config.speed_limit;
    maneuver.lane_id = lane_;
  } else {
    maneuver.speed = road_config.speed_limit;
    maneuver.lane_id = lane_ + lane_direction_[state];
  }

  return maneuver;
}

std::optional<Object> Vehicle::get_object_from_behind(int lane_id) const {
  if (nearby_objects_[lane_id].object_from_behind.id != kInvalidObjectId) {
    return nearby_objects_[lane_id].object_from_behind;
  }
  return {};
}

std::optional<Object> Vehicle::get_object_in_front(int lane_id) const {
  if (nearby_objects_[lane_id].object_in_front.id != kInvalidObjectId) {
    return nearby_objects_[lane_id].object_in_front;
  }
  return {};
}

std::optional<double> Vehicle::get_space_in_front(int lane_id) const {
  if (nearby_objects_[lane_id].object_in_front.id != kInvalidObjectId) {
    return nearby_objects_[lane_id].object_in_front.frenet_point.s -
           frenet_point_.s;
  }
  return {};
}

std::optional<double> Vehicle::get_space_from_behind(int lane_id) const {
  if (nearby_objects_[lane_id].object_from_behind.id != kInvalidObjectId) {
    return frenet_point_.s -
           nearby_objects_[lane_id].object_from_behind.frenet_point.s;
  }
  return {};
}

double Vehicle::calculate_safe_speed(int lane) {

  const auto object_in_front{get_object_in_front(lane)};
  double space_in_front{
      object_in_front.has_value() ? get_space_in_front(lane).value() : 1000.};
  double speed_in_front{nearby_objects_[lane].object_in_front.id !=
                                kInvalidObjectId
                            ? nearby_objects_[lane].object_in_front.speed
                            : speed_limit_};
  double safety_distance{std::min(get_safety_distance(), speed_in_front * 1.5)};
  double max_speed{speed_limit_ - 0.2};
  double speed{0.0};

  if (space_in_front > safety_distance) {
    speed = max_speed;
  } else if (space_in_front > safety_distance - safety_distance / 3) {
    speed = speed_in_front;
  } else {
    speed = speed_in_front - 3.0;
  }

  Logger::get_logger() << "space in front: " << space_in_front
                       << " speed in front: " << speed_in_front
                       << " safety distance: " << safety_distance
                       << " max speed: " << max_speed
                       << " Chosen safe speed: " << speed << "\n";

  return speed;
}

void Vehicle::choose_next_state(const IEnvironmentData &data) {

  update_nearby_objects(data);

  const std::vector<std::string> states{successor_states(data)};
  double min_cost{std::numeric_limits<double>::infinity()};
  std::string next_state{};
  Maneuver chosen_maneuver{};
  std::string c_state{current_state_};

  for (const auto &state : states) {

    auto maneuver{prepare_maneuvers(state, data.get_road_configuration())};
    Cost cost{maneuver};
    auto calculated_cost{cost.calculate_cost(*this)};

    if (calculated_cost < min_cost) {
      min_cost = calculated_cost;
      next_state = state;
      chosen_maneuver = maneuver;
    }
  }

  current_state_ = next_state;
  chosen_maneuver_ = chosen_maneuver;

  adjusr_target_speed();

  Logger::get_logger() << "Current state: " << c_state
                       << " next state: " << current_state_ << " maneuver: "
                       << chosen_maneuver_.maneuver_type_to_string()
                       << " lane id: " << chosen_maneuver_.lane_id
                       << " speed: " << chosen_maneuver_.speed << "\n";
}

std::vector<std::string>
Vehicle::successor_states(const IEnvironmentData &data) {

  const RoadConfiguration &road_config{data.get_road_configuration()};
  std::vector<std::string> states{};

  states.emplace_back("KL");

  if (!current_state_.compare("INIT")) {
    states.emplace_back("INIT");
  } else if (!current_state_.compare("KL")) {
    if (lane_ != 0) {
      states.emplace_back("PLCL");
    }
    if (lane_ != road_config.num_lanes - 1) {
      states.emplace_back("PLCR");
    }
  } else if (!current_state_.compare("PLCR")) {
    if (lane_ != road_config.num_lanes - 1) {
      states.emplace_back("PLCR");
      states.emplace_back("LCR");
    }
  } else if (!current_state_.compare("PLCL")) {
    if (lane_ != 0) {
      states.emplace_back("PLCL");
      states.emplace_back("LCL");
    }
  }

  return states;
}

void Vehicle::adjusr_target_speed() {
  if (current_state_.compare("INIT")) {
    chosen_maneuver_.speed = calculate_safe_speed(chosen_maneuver_.lane_id);
  }
}

std::vector<std::vector<double>>
Vehicle::get_trajectory(const IEnvironmentData &env_data) {
  return trajectory_.generate_trajectory(env_data, *this);
}

} // namespace highway_driving
} // namespace sdc
