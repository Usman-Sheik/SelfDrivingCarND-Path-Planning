#ifndef PATH_PLANNING_CARTESIAN_POINT_H
#define PATH_PLANNING_CARTESIAN_POINT_H

namespace sdc {
namespace highway_driving {

struct CartesianPoint {
  double x{0.};
  double y{0.};
  double theta{0.};
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_CARTESIAN_POINT_H
