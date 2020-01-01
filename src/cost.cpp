#include <algorithm>
#include <cmath>
#include <vector>

#include "cost.h"
#include "logger.h"

namespace sdc {
namespace highway_driving {

static constexpr double kCollisionCost{10e6};
static constexpr double kDangerCost{10e5};
static constexpr double kComfortCost{10e4};
static constexpr double kEfficienyCost{10e2};

double Cost::calculate_cost(const IVehicleData &vehicle_data) {

  double cost{0.0};

  cost += calculate_start_cost(vehicle_data);
  cost += calculate_keep_lane_cost(vehicle_data);
  cost += calculate_lane_speed_cost(vehicle_data);
  cost += calculate_free_lane_cost(vehicle_data);
  cost += calculate_collisions_cost(vehicle_data);

  Logger::get_logger() << "Maneuver \"" << maneuver_.maneuver_type_to_string()
                       << "\" has total cost of \"" << cost << "\""
                       << "\n";

  return cost;
}

double Cost::calculate_start_cost(const IVehicleData &data) const {

  const double current_vehicle_speed{data.get_speed()};
  double cost{0.};

  if (maneuver_.speed <= 10.0) {
    if (current_vehicle_speed <= 10.0) {
      cost = -kComfortCost;
    } else {
      cost = 5 * kComfortCost;
    }
  }

  Logger::get_logger() << "Maneuver \"" << maneuver_.maneuver_type_to_string()
                       << "\" has start cost of \"" << cost << "\""
                       << "\n";
  return cost;
}

double Cost::calculate_keep_lane_cost(const IVehicleData &data) const {
  double cost{0.};

  if (data.get_current_lane() == maneuver_.lane_id) {
    cost = -kEfficienyCost * 10;
  }

  Logger::get_logger() << "Maneuver \"" << maneuver_.maneuver_type_to_string()
                       << "\" has keep lane cost of \"" << cost << "\""
                       << "\n";

  return cost;
}

double Cost::calculate_lane_speed_cost(const IVehicleData &data) const {
  double cost{0.};
  const auto object_in_front{data.get_object_in_front(maneuver_.lane_id)};
  const auto space_in_front{
      object_in_front.has_value()
          ? data.get_space_in_front(maneuver_.lane_id).value()
          : 1000};
  const auto max_speed{data.get_max_speed()};

  if (object_in_front.has_value() && space_in_front < 75.0) {
    cost = 10 * (max_speed - object_in_front.value().speed) * kEfficienyCost;
  }

  Logger::get_logger() << "Maneuver \"" << maneuver_.maneuver_type_to_string()
                       << "\" has lane speed cost of \"" << cost << "\""
                       << "\n";

  return cost;
}

double Cost::calculate_free_lane_cost(const IVehicleData &data) const {
  const auto object_in_front{data.get_object_in_front(maneuver_.lane_id)};
  const auto space_in_front{
      object_in_front.has_value()
          ? data.get_space_in_front(maneuver_.lane_id).value()
          : 1000};
  double cost{-1 * std::min(75.0, space_in_front) * kEfficienyCost};
  return cost;
}

double Cost::calculate_collisions_cost(const IVehicleData &data) const {
  double cost{0.};
  double safety_distance{data.get_safety_distance()};
  const auto object_in_front{data.get_object_in_front(maneuver_.lane_id)};
  const auto object_from_behind{data.get_object_from_behind(maneuver_.lane_id)};
  const auto space_in_front{
      object_in_front.has_value()
          ? data.get_space_in_front(maneuver_.lane_id).value()
          : 1000};
  const auto space_from_behind{
      object_from_behind.has_value()
          ? data.get_space_from_behind(maneuver_.lane_id).value()
          : 10e6};

  std::exper

      if (space_in_front < 10.) {
    cost += kCollisionCost;
  }
  else if (space_in_front < safety_distance * .25) {
    cost += kDangerCost;
  }

  if (space_from_behind < 10.) {
    cost += kCollisionCost;
  }

  Logger::get_logger() << "Maneuver \"" << maneuver_.maneuver_type_to_string()
                       << "\" has collisions cost of \"" << cost << "\""
                       << "\n";

  return cost;
}

} // namespace highway_driving
} // namespace sdc
