#include "vehicle.h"
#include "helpers.h"
#include "spline.h"
#include <algorithm>
#include <iostream>
#include <limits>

using std::cout;
using std::endl;
using std::unordered_map;

const double Vehicle::MAX_VEL = 22.0;
const double Vehicle::MIN_VEL = 10.0;
const double Vehicle::MAX_ACC = 10.0;
const double Vehicle::MAX_JERK = 10.0;
const double Vehicle::VEHICLE_RADIUS = 5.0;
const double Vehicle::LANE_WIDTH = 4.0;
const double Vehicle::COLLISION_THRESHOLD = LANE_WIDTH;
const double Vehicle::PREDICTION_TIME = 3.0;

Vehicle::Vehicle(int id, double x, double y, double x_vel, double y_vel,
                 double s, double d, int lane) {
  id_ = id;
  x_ = x;
  y_ = y;
  x_vel_ = x_vel;
  y_vel_ = y_vel;
  s_ = s;
  d_ = d;
  lane_ = lane;
}

void Vehicle::show_info() {
  cout << "id: " << id_ << ", lane: " << lane_ << ", s: " << s_ << ", d: " << d_
       << endl;
}

void Vehicle::set_map(const std::vector<double> &map_waypoints_x,
                      const std::vector<double> &map_waypoints_y,
                      const std::vector<double> &map_waypoints_s,
                      const std::vector<double> &map_waypoints_dx,
                      const std::vector<double> &map_waypoints_dy) {
  map_waypoints_x_.assign(map_waypoints_x.begin(), map_waypoints_x.end());
  map_waypoints_y_.assign(map_waypoints_y.begin(), map_waypoints_y.end());
  map_waypoints_s_.assign(map_waypoints_s.begin(), map_waypoints_s.end());
  map_waypoints_dx_.assign(map_waypoints_dx.begin(), map_waypoints_dx.end());
  map_waypoints_dy_.assign(map_waypoints_dy.begin(), map_waypoints_dy.end());
}

void Vehicle::update_state(double x, double y, double s, double d, double yaw,
                           double speed) {
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  speed_ = speed;
  lane_ = get_lane_from_d(d);
}

void Vehicle::get_trajectory(std::vector<double> &next_x_vals,
                             std::vector<double> &next_y_vals,
                             const std::vector<std::vector<double>> &prev_path,
                             double end_path_s, double end_path_d,
                             const std::vector<Vehicle> &traffics) {
  next_x_vals.clear();
  next_y_vals.clear();

  /* if (traffics.size() > 0) {
    cout << "Traffic: " << traffics.size() << endl;
    for (Vehicle each : traffics) {
      each.show_info();
    }
  }
  cout << "We are in " << get_lane_from_d(d_) << " lane"
       << ", s: " << s_ << ", d: " << d_ << endl; */

  std::vector<std::string> next_states = successor_states();
  std::vector<double> costs;
  std::vector<std::vector<Vehicle>> trajectries;
  std::unordered_map<int, std::vector<Vehicle>> predictions;
  cout << "traffics size: " << traffics.size() << endl;
  for (Vehicle vehicle : traffics) {
    predictions.insert(std::make_pair(vehicle.id_, vehicle.get_prediction()));
  }
  cout << "predictions complete" << endl;
  for (std::string state : next_states) {
    std::vector<Vehicle> trajectory =
        generate_trajectory(state, predictions, prev_path);
    if (trajectory.size() > 0) {
      double cost = 0.0;
      costs.push_back(cost);
    } else {
      costs.push_back(std::numeric_limits<double>::infinity());
    }
    trajectries.push_back(trajectory);
  }
  cout << "generate_trajectory complete" << endl;

  double min_cost = std::numeric_limits<double>::infinity();
  int best_trajectory_idx = 0;
  for (int i = 0; i < trajectries.size(); ++i) {
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      best_trajectory_idx = i;
    }
  }
  cout << "best_trajectory_idx:" << best_trajectory_idx << std::endl;

  std::vector<Vehicle> &best_trajectory = trajectries[best_trajectory_idx];
  for (int i = 0; i < best_trajectory.size(); ++i) {
    // cout << "x: " << best_trajectory[i].x() << ", y: " <<
    // best_trajectory[i].y()
    //      << endl;
    next_x_vals.push_back(best_trajectory[i].x());
    next_y_vals.push_back(best_trajectory[i].y());
  }

  // Just for test
  /* double dist_inc = 0.5;
  for (int i = 0; i < 50; ++i) {
    next_x_vals.push_back(x_ + (dist_inc * i) * cos(deg2rad(yaw_)));
    next_y_vals.push_back(y_ + (dist_inc * i) * sin(deg2rad(yaw_)));
  } */
}

std::vector<std::string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  std::vector<std::string> states;
  states.push_back("KL");
  /* if (this->state_.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (this->state_.compare("PLCL") == 0) {
    if (lane_ != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (this->state_.compare("PLCR") == 0) {
    if (lane_ != LANE_AVAILABLE - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } */

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

std::vector<Vehicle> Vehicle::get_prediction(double delta_time) {
  double dt = 0.1;
  double elapsed_time = 0.0;
  std::vector<Vehicle> trajectory;
  double x = x_;
  double y = y_;
  do {
    if (dt + elapsed_time > delta_time) {
      dt = delta_time - elapsed_time;
    }
    double next_x = x + x_vel_ * dt;
    double next_y = y + y_vel_ * dt;
    double next_yaw = atan2(y - next_y, x - next_x);
    auto next_frenet =
        getFrenet(next_x, next_y, next_yaw, map_waypoints_x_, map_waypoints_y_);
    Vehicle vehicle(id_, next_x, next_y, x_vel_, y_vel_, next_frenet[0],
                    next_frenet[1], get_lane_from_d(next_frenet[1]));
    trajectory.push_back(vehicle);
    x = next_x;
    y = next_y;
    elapsed_time += dt;
  } while (elapsed_time < delta_time);

  return trajectory;
}

std::vector<Vehicle> Vehicle::generate_trajectory(
    std::string state,
    const std::unordered_map<int, std::vector<Vehicle>> &prediction,
    const std::vector<std::vector<double>> &prev_path) {
  std::vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    return constant_speed_trajectory(prediction, prev_path);
  } else if (state.compare("KL") == 0) {
    return keep_lane_trajectory(prediction, prev_path);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    return prep_lane_change_trajectory(prediction, prev_path);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    return lane_change_trajectory(prediction, prev_path);
  }
  return trajectory;
}

std::vector<Vehicle> Vehicle::constant_speed_trajectory(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction,
    const std::vector<std::vector<double>> &prev_path) {
  std::vector<Vehicle> trajectory;
  return trajectory;
}

std::vector<Vehicle> Vehicle::keep_lane_trajectory(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction,
    const std::vector<std::vector<double>> &prev_path) {
  int vehicle_ahead = get_vehicle_ahead(prediction);
  std::vector<Vehicle> trajectory;
  double next_s = s_ + MAX_VEL * PREDICTION_TIME;
  if (vehicle_ahead >= 0) {
  }
  //   cout << "next_s: " << next_s << ", s: " << s_ << endl;
  //   cout << "current, x: " << x_ << ", y: " << y_ << endl;
  double s = s_;
  do {
    s += 0.02 * MAX_VEL;
    if (s > next_s) {
      s = next_s;
    }
    vector<double> coord =
        getXY(s, d_, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    trajectory.push_back(
        Vehicle(id_, coord[0], coord[1], 0.0, 0.0, s, d_, lane_));
  } while (s < next_s);
  /* if (prev_path.size() > 0) {
  } else {
    for (int i = 0; i < prev_path.size() && i < PREV_PATH_REUSE; ++i) {
      trajectory.push_back(Vehicle(id_, prev_path[i][0], prev_path[i][1], 0.0,
  0.0, ));
    }
  } */

  //   cout << "keep_lane_trajectory--" << endl;
  return trajectory;
}

std::vector<Vehicle> Vehicle::prep_lane_change_trajectory(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction,
    const std::vector<std::vector<double>> &prev_path) {
  std::vector<Vehicle> trajectory;
  return trajectory;
}

std::vector<Vehicle> Vehicle::lane_change_trajectory(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction,
    const std::vector<std::vector<double>> &prev_path) {
  std::vector<Vehicle> trajectory;
  return trajectory;
}

int Vehicle::get_vehicle_behind(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double max_s = -1.0;
  int id = -1;
  Vehicle temp_vehicle;
  for (auto iter = prediction.begin(); iter != prediction.end(); ++iter) {
    const Vehicle &vehicle = iter->second[0];
    if (vehicle.lane_ == this->lane_ && vehicle.s_ < this->s_ &&
        vehicle.s_ > max_s) {
      max_s = vehicle.s_;
      id = vehicle.id_;
    }
  }

  return id;
}

int Vehicle::get_vehicle_ahead(
    const std::unordered_map<int, std::vector<Vehicle>> &prediction) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double min_s = std::numeric_limits<double>::infinity();
  int id = -1;
  bool found_vehicle = false;
  for (auto iter = prediction.begin(); iter != prediction.end(); ++iter) {
    const Vehicle &vehicle = iter->second[0];
    if (vehicle.lane_ == this->lane_ && vehicle.s_ > this->s_ &&
        vehicle.s_ < min_s) {
      min_s = vehicle.s_;
      id = vehicle.id_;
    }
  }

  return id;
}