#ifndef PATH_PLANNING_FRENET_POINT_H
#define PATH_PLANNING_FRENET_POINT_H

namespace sdc {
namespace highway_driving {

struct FrenetPoint {
  double s{0.};
  double d{0.};
};

// inline double get_d_center(const int lane) {
//  return lane * kLaneWidth + kLaneWidth / 2.0;
//}

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_FRENET_POINT_H
