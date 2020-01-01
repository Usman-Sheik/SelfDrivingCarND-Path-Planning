#include <algorithm>
#include <cmath>
#include <unistd.h>

#include "helpers.h"
#include "logger.h"
#include "spline.h"
#include "trajectory.h"

namespace sdc {
namespace highway_driving {

namespace {
double get_acceleration(double max_acc, double speed, double target_speed) {
  if (std::fabs(speed) < 1)
    return max_acc;
  else
    return max_acc - max_acc * log(speed) / log(target_speed);
}
} // namespace

void Trajectory::update_previous_path(
    const std::vector<std::vector<double>> &prev_trajectory) {
  previous_trajectory_ = prev_trajectory;
}

std::vector<std::vector<double>>
Trajectory::generate_trajectory(const IEnvironmentData &env_data,
                                const IVehicleData &vehicle_data) {

  constexpr int X{0};
  constexpr int Y{1};
  const auto frenet{vehicle_data.get_frenet_coordinates()};
  const auto cartesian{vehicle_data.get_cartesian_coordinates()};
  const auto maneuver{vehicle_data.get_chosen_maneuver()};
  const auto &waypoints{env_data.get_waypoints()};
  const auto speed(vehicle_data.get_speed());
  const auto state{vehicle_data.get_chosen_state()};
  const double wrap_point{env_data.get_wrap_point()};

  std::vector<std::vector<double>> next_path{2};

  std::vector<double> ref{};
  int path_size = previous_trajectory_[X].size();
  int reuse_size = std::min(10, path_size);

  int t_size = current_trajectory_[X].size();
  int first_reuse = std::max(0, t_size - path_size);
  for (int i = first_reuse; i < t_size && i < first_reuse + reuse_size; i++) {
    next_path[X].push_back(this->current_trajectory_[X][i]);
    next_path[Y].push_back(this->current_trajectory_[Y][i]);
  }

  vector<vector<double>> spts(2);

  if (path_size < 2) {
    ref.push_back(cartesian.x);
    ref.push_back(cartesian.y);
    ref.push_back(deg2rad(cartesian.theta));
    spts[X].push_back(0.0);
    spts[Y].push_back(0.0);
  } else {
    ref.push_back(next_path[X][reuse_size - 1]);
    ref.push_back(next_path[Y][reuse_size - 1]);

    double x2 = next_path[X][reuse_size - 2];
    double y2 = next_path[Y][reuse_size - 2];
    ref.push_back(atan2(ref[Y] - y2, ref[X] - x2));

    vector<double> prev = global2car({x2, y2}, ref);
    spts[X].push_back(prev[X]);
    spts[Y].push_back(prev[Y]);
    spts[X].push_back(0.0);
    spts[Y].push_back(0.0);
  }

  double center{
      get_lane_center(maneuver.lane_id, env_data.get_road_configuration())};

  double s{wrap_point != 0. ? fmod(frenet.s, wrap_point) : frenet.s};
  vector<double> next0{
      getXY(s + 40, center, waypoints[2], waypoints[0], waypoints[1])};
  next0 = global2car(next0, ref);
  spts[X].push_back(next0[X]);
  spts[Y].push_back(next0[Y]);

  vector<double> next1{
      getXY(s + 80, center, waypoints[2], waypoints[0], waypoints[1])};
  next1 = global2car(next1, ref);
  spts[X].push_back(next1[X]);
  spts[Y].push_back(next1[Y]);

  tk::spline sp{};
  sp.set_points(spts[X], spts[Y]);

  double target_x{40.0};
  double target_y{sp(target_x)};
  double target_dist{sqrt(target_x * target_x + target_y * target_y)};
  double max_acc{5.0};
  double prev_x{0.0};
  double prev_y{0.0};
  double prev_speed{
      (path_size < 2)
          ? speed
          : sqrt(spts[X][0] * spts[X][0] + spts[Y][0] * spts[Y][0]) / 0.02};
  double acc{0.0};

  for (int i = 0; i < 50 - reuse_size; i++) {
    double x{}, y{};

    if (state.compare("INIT") != 0) {
      acc = get_acceleration(max_acc, prev_speed, maneuver.speed);
      if (prev_speed > maneuver.speed && acc > 0.0) {
        acc = 0.0;
        prev_speed = maneuver.speed;
      }
      x = prev_x + prev_speed * .02 + .5 * .004 * acc;
      y = sp(x);
      prev_speed =
          sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y)) / .02;
    } else {
      double N = target_dist / (.02 * maneuver.speed);
      x = prev_x + target_x / N;
      y = sp(x);
    }

    prev_x = x;
    prev_y = y;

    std::vector<double> pt{car2global({x, y}, ref)};
    next_path[X].push_back(pt[X]);
    next_path[Y].push_back(pt[Y]);
  }

  current_trajectory_[X].clear();
  current_trajectory_[Y].clear();
  current_trajectory_[X] = next_path[X];
  current_trajectory_[Y] = next_path[Y];

  return next_path;
}

} // namespace highway_driving
} // namespace sdc
