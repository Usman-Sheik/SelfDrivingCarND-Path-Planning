#ifndef PATH_PLANNING_OBJECT_H
#define PATH_PLANNING_OBJECT_H

#include <cmath>
#include <stdint.h>
#include <vector>

#include "cartesian_point.h"
#include "frenet_point.h"

namespace sdc {
namespace highway_driving {

static constexpr int kInvalidObjectId{-1};

enum class ObjectType : uint8_t { UNKNOWN = 0, VEHICLE };

struct Object {
  int id{kInvalidObjectId};
  ObjectType object_type{ObjectType::UNKNOWN};
  FrenetPoint frenet_point{};
  CartesianPoint cartesian_point{};
  double speed{0.};
};

inline std::vector<Object> create_objects_from_sensor_input(
    std::vector<std::vector<double>> sensor_input) {

  std::vector<Object> object_list{};

  for (const auto &object : sensor_input) {

    const int object_id{static_cast<int>(object[0])};
    const double x{object[1]};
    const double y{object[2]};
    const double vx{object[3]};
    const double vy{object[4]};
    const double s{object[5]};
    const double d{object[6]};

    object_list.emplace_back(Object{object_id, ObjectType::VEHICLE,
                                    FrenetPoint{s, d}, CartesianPoint{x, y, 0.},
                                    double{std::sqrt(vx * vx + vy * vy)}});
  }

  return object_list;
}
} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_OBJECT_H
