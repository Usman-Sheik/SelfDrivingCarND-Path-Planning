#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include "i_vehicle_data.h"
#include "maneuver.h"
#include "vehicle.h"

namespace sdc {
namespace highway_driving {

class Cost {

public:
  Cost(const Maneuver &maneuver) : maneuver_{maneuver} {}
  ~Cost() = default;

  double calculate_cost(const IVehicleData &vehicle_data);

private:
  double calculate_start_cost(const IVehicleData &data) const;
  double calculate_keep_lane_cost(const IVehicleData &data) const;
  double calculate_lane_speed_cost(const IVehicleData &data) const;
  double calculate_free_lane_cost(const IVehicleData &data) const;
  double calculate_collisions_cost(const IVehicleData &data) const;

  Maneuver maneuver_{};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_COST_H
