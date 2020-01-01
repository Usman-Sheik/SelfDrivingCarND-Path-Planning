#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "highway_map.h"
#include "json.hpp"
#include "object.h"
#include "road.h"
#include "road_configuration.h"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using namespace sdc::highway_driving;

static constexpr int kNumLanes{3};
static const std::vector<int> kLaneIndices{0, 1, 2};
static constexpr double kLaneWidth{4.};
static constexpr double kSpeedLimit{mph2mps(50.)};

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  std::string map_file{"../data/highway_map.csv"};
  // The max s value before wrapping around the track back to 0
  double max_s{6945.554};

  HighwayMap highway_map{map_file, max_s};

  Road road{highway_map, RoadConfiguration{kNumLanes, kLaneWidth, kSpeedLimit,
                                           kLaneIndices}};

  h.onMessage([&highway_map, &road](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                    size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          std::vector<std::vector<double>> prev_path{previous_path_x,
                                                     previous_path_y};

          road.update_environment(
              create_objects_from_sensor_input(sensor_fusion));
          road.update_ego_vehicle(FrenetPoint{car_s, car_d},
                                  CartesianPoint{car_x, car_y, car_yaw},
                                  car_speed, prev_path);
          road.step();

          vector<vector<double>> trajectory{road.get_ego_trajectory()};

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          msgJson["next_x"] = trajectory[0];
          msgJson["next_y"] = trajectory[1];

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
