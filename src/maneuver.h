#ifndef PATH_PLANNING_MANEUVER_H
#define PATH_PLANNING_MANEUVER_H

#include <iostream>
#include <string>

namespace sdc {
namespace highway_driving {

enum ManeuverType : std::uint8_t {
  INIT = 0,
  KEEP_LANE,
  PREPARE_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREPARE_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

struct Maneuver {

  int lane_id{0};
  double speed{0.};
  ManeuverType maneuver_type{ManeuverType::INIT};

  std::string maneuver_type_to_string() const {
    switch (maneuver_type) {
    case ManeuverType::INIT:
      return "INIT";
    case ManeuverType::KEEP_LANE:
      return "KL;";
    case ManeuverType::PREPARE_LANE_CHANGE_LEFT:
      return "PLCL";
    case ManeuverType::LANE_CHANGE_LEFT:
      return "LCL";
    case ManeuverType::PREPARE_LANE_CHANGE_RIGHT:
      return "PLCR";
    case ManeuverType::LANE_CHANGE_RIGHT:
      return "LCR";
    }
  }
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_MANEUVER_H
