#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

#include "i_data_source.h"
#include "i_vehicle_data.h"

namespace sdc {
namespace highway_driving {

class Trajectory {

public:
  Trajectory() : current_trajectory_(2) {}
  ~Trajectory() = default;

  void
  update_previous_path(const std::vector<std::vector<double>> &prev_trajectory);
  std::vector<std::vector<double>>
  generate_trajectory(const IEnvironmentData &env_data,
                      const IVehicleData &vehicle_data);

private:
  std::vector<std::vector<double>> previous_trajectory_{};
  std::vector<std::vector<double>> current_trajectory_{};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_TRAJECTORY_H
