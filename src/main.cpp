#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Initialize out ego vehicle
  Vehicle ego_vehicle;
  ego_vehicle.set_map(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                      map_waypoints_dx, map_waypoints_dy);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy,
               &ego_vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data,
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
          double car_speed = j[1]["speed"];  // Unit is MPH
          printf("curr xy: (%f, %f), speed: %f\n", car_x, car_y,
                 mph2ms(car_speed));

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          std::vector<std::vector<double>> prev_path;
          for (int i = 0; i < previous_path_x.size(); ++i) {
            std::vector<double> point;
            point.push_back(previous_path_x[i]);
            point.push_back(previous_path_y[i]);
            prev_path.push_back(point);
          }
          std::cout << "--------------------" << std::endl;
          std::cout << "prev_path size: " << prev_path.size() << std::endl;
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Vehicle> traffics;
          for (auto each : sensor_fusion) {
            // The max speed is 50 MPH = 22.352 meter/second.
            // I'll predict the vehicle's trajectory in 3 seconds.
            // 3 * 22.352 = 67.056, So I set MAX_DISTANCE = 68.0.
            const double MAX_DISTANCE = 68.0;
            int id = each[0];
            double x = each[1];
            double y = each[2];
            double x_vel = each[3];
            double y_vel = each[4];
            double s = each[5];
            double d = each[6];
            int lane = get_lane_from_d(d);
            double dis = distance(car_x, car_y, x, y);
            if (lane >= 0 && dis <= MAX_DISTANCE) {
              Vehicle vehicle(id, x, y, x_vel, y_vel, s, d, lane);
              vehicle.set_map(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                              map_waypoints_dx, map_waypoints_dy);
              traffics.push_back(vehicle);
            }
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          ego_vehicle.update_state(car_x, car_y, car_s, car_d, deg2rad(car_yaw),
                                   mph2ms(car_speed), prev_path.size());
          ego_vehicle.get_trajectory(next_x_vals, next_y_vals, prev_path,
                                     end_path_s, end_path_d, traffics);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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