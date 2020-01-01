#ifndef PATH_PLANNING_I_VEHICLE_DATA_H
#define PATH_PLANNING_I_VEHICLE_DATA_H

#include <string>

#include "maneuver.h"
#include "object.h"
#include <optional>

namespace sdc {
namespace highway_driving {

class IVehicleData {
public:
  virtual std::optional<Object> get_object_from_behind(int lane_id) const = 0;
  virtual std::optional<Object> get_object_in_front(int lane_id) const = 0;
  virtual std::optional<double> get_space_in_front(int lane_id) const = 0;
  virtual std::optional<double> get_space_from_behind(int lane_id) const = 0;
  virtual double get_speed() const = 0;
  virtual double get_max_speed() const = 0;
  virtual double get_safety_distance() const = 0;
  virtual int get_current_lane() const = 0;
  virtual FrenetPoint get_frenet_coordinates() const = 0;
  virtual CartesianPoint get_cartesian_coordinates() const = 0;
  virtual Maneuver get_chosen_maneuver() const = 0;
  virtual std::string get_chosen_state() const = 0;
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_I_VEHICLE_DATA_H
