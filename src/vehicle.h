#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "cartesian_point.h"
#include "frenet_point.h"
#include "i_data_source.h"
#include "i_vehicle_data.h"
#include "object.h"
#include "trajectory.h"

namespace sdc {
namespace highway_driving {

class Vehicle : public IVehicleData {

public:
  explicit Vehicle(const FrenetPoint &frenet,
                   const CartesianPoint &cartesian_point, int lane,
                   double speed);

  ~Vehicle() = default;

  void UpdateState(const CartesianPoint &cartesian_point,
                   const FrenetPoint &frenet_point, const double speed,
                   const std::vector<std::vector<double>> &prev_trajectory,
                   const IEnvironmentData &data);

  void choose_next_state(const IEnvironmentData &data);

  std::optional<Object> get_object_from_behind(int lane_id) const override;
  std::optional<Object> get_object_in_front(int lane_id) const override;
  double get_safety_distance() const override { return speed_limit_ * 1.5; };
  std::optional<double> get_space_in_front(int lane_id) const override;
  std::optional<double> get_space_from_behind(int lane_id) const override;
  double get_speed() const override { return speed_; }
  int get_current_lane() const override { return lane_; };
  double get_max_speed() const override { return speed_limit_; }
  FrenetPoint get_frenet_coordinates() const override { return frenet_point_; }
  CartesianPoint get_cartesian_coordinates() const override {
    return cartesian_point_;
  };
  Maneuver get_chosen_maneuver() const override { return chosen_maneuver_; };
  std::string get_chosen_state() const override { return current_state_; }

  std::vector<std::string> successor_states(const IEnvironmentData &data);
  void adjusr_target_speed();

  std::vector<std::vector<double>>
  get_trajectory(const IEnvironmentData &env_data);

private:
  bool update_object_from_behind(int lane, Object object, double new_distance);
  bool update_object_in_front(int lane, Object object, double new_distance);
  void update_nearby_objects(const IEnvironmentData &data);
  Maneuver prepare_maneuvers(const std::string &state,
                             const RoadConfiguration &road_config);
  void reset_nearby_objects(int num_lanes);
  ManeuverType get_maneuver_type_from_state(const std::string &state);
  double calculate_safe_speed(int lane);

  struct NearbyObjects {
    Object object_from_behind{};
    Object object_in_front{};
  };

  std::map<std::string, int> lane_direction_ = {
      {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  int id_{-1};
  std::string current_state_{"INIT"};
  FrenetPoint frenet_point_{};
  CartesianPoint cartesian_point_{};
  int lane_{0};
  double speed_{0.};
  double speed_limit_{0.};
  double target_speed_{0.};
  Maneuver chosen_maneuver_{};

  std::vector<NearbyObjects> nearby_objects_{};
  Trajectory trajectory_{};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_VEHICLE_H
