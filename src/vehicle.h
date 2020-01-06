#ifndef VEHICLE_H
#define VEHICLE_H

// #include <deque>
#include <list>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// #include "json.hpp"

/**
 * Current Idea:
 * 1. Model based prediction
 * 2. Behavior planning: State machine
 */

class Vehicle {
  // Define limitation conditions
  static const double MAX_VEL;   // Unit: 22.0 m/s
  static const double MIN_VEL;   // Unit: 10.0 m/s
  static const double MAX_ACC;   // Unit: 10.0 m/s^2
  static const double MAX_JERK;  // Unit: 210.0 m/s^3
  // Define some constant environment variables
  static const double VEHICLE_RADIUS;  // Unit: 5.0 m
  static const int LANE_AVAILABLE = 3;
  static const double MAX_S;
  static const double LANE_WIDTH;  // Unit: 4.0 m
  static const double COLLISION_THRESHOLD;
  // Define some useful constant values
  static const int PREV_PATH_REUSE = 20;
  static const double PREDICTION_TIME;
  static const double TIME_STEP;

  enum State { KL = 0, LCL = 1, LCR = 2 };
  /* const std::unordered_map<std::string, int> LANE_DIRECTION = {
      {"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}}; */

 private:
  int id_;

  State state_;
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;  // Unit: m/s
  double s_vel_;  // Unit: m/s
  double d_vel_;  // Unit: m/s
  double s_acc_;  // Unit: m/s^2
  double d_acc_;  // Unit: m/s^2
  std::vector<double> prev_trajectory_s_;
  std::vector<double> prev_trajectory_d_;
  // Used by other vehicle detected by sensor
  double x_vel_;  // Unit: m/s
  double y_vel_;  // Unit: m/s
  int lane_;
  int target_lane_;

  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;

 public:
  Vehicle();
  Vehicle(int id, double x, double y, double x_vel, double y_vel, double s,
          double d);
  ~Vehicle() {}

  void show_info();
  double x() { return x_; }
  double y() { return y_; }

  void set_map(const std::vector<double> &map_waypoints_x,
               const std::vector<double> &map_waypoints_y,
               const std::vector<double> &map_waypoints_s,
               const std::vector<double> &map_waypoints_dx,
               const std::vector<double> &map_waypoints_dy);

  void update_state(double x, double y, double s, double d, double yaw,
                    double speed);

  void get_trajectory(std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals, int prev_path_size,
                      std::unordered_map<int, Vehicle> &traffics);

  std::vector<std::string> successor_states();

  //   std::string get_next_state();

  // Predict the trajectory in PREDICTION_TIME seconds
  // Return format: prediction[i] = {s, d}
  std::vector<std::vector<double>> get_prediction();

  std::vector<std::vector<double>> generate_trajectory_to_lane(
      int target_lane, std::unordered_map<int, Vehicle> &traffics,
      int prev_path_size);

  std::vector<std::vector<double>> generate_trajectory(
      std::string state,
      std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
      int prev_path_size);

  std::vector<std::vector<double>> keep_lane_trajectory(
      std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
      int prev_path_size);

  std::vector<std::vector<double>> lane_change_trajectory(
      std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
      int prev_path_size);

  //   int get_vehicle_behind(int target_lane,
  //                          const std::unordered_map<int, Vehicle> &traffics);
  int get_vehicle_ahead(int target_lane,
                        const std::unordered_map<int, Vehicle> &traffics);

  std::vector<double> jerk_minimize_trajectory(std::vector<double> &start,
                                               std::vector<double> &end,
                                               double dt);
};

#endif  // VEHICLE_H
