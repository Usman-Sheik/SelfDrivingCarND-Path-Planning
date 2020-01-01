#ifndef PATH_PLANNING_LOGGER_H
#define PATH_PLANNING_LOGGER_H

#include <iostream>

namespace sdc {
namespace highway_driving {

static constexpr auto kEnableDebug{1};

class Logger {

public:
  static Logger &get_logger() {
    static Logger logger;
    return logger;
  }

  template <typename T> Logger &operator<<(const T &val) {
    if (kEnableDebug) {
      std::cout << val;
    }
    return *this;
  }

private:
  Logger() = default;
};

} // namespace highway_driving
} // namespace sdc

#endif // PATH_PLANNING_LOGGER_H
