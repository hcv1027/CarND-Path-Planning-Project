#include "vehicle.h"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "spline.h"
#include <algorithm>
#include <iostream>
#include <limits>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::cout;
using std::endl;
using std::pow;
using std::unordered_map;

const double Vehicle::MAX_VEL = 22.0;
const double Vehicle::MIN_VEL = 10.0;
const double Vehicle::MAX_ACC = 10.0;
const double Vehicle::VEHICLE_RADIUS = 5.0;
const double Vehicle::LANE_WIDTH = 4.0;
const double Vehicle::MAX_S = 6945.554;
const double Vehicle::COLLISION_THRESHOLD = 10.0;
const double Vehicle::PREDICTION_TIME = 1.0;
const double Vehicle::TIME_STEP = 0.02;

Vehicle::Vehicle() {
  s_vel_ = 0.0;
  d_vel_ = 0.0;
  s_acc_ = 0.0;
  d_acc_ = 0.0;
  state_ = "CS";
}

Vehicle::Vehicle(int id, double x, double y, double x_vel, double y_vel,
                 double s, double d) {
  id_ = id;
  x_ = x;
  y_ = y;
  x_vel_ = x_vel;
  y_vel_ = y_vel;
  s_ = s;
  d_ = d;
  lane_ = get_lane_from_d(d);
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
  //   cout << "curr s: " << s_ << endl;
}

void Vehicle::get_trajectory(std::vector<double> &next_x_vals,
                             std::vector<double> &next_y_vals,
                             int prev_path_size,
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

  // Generate traffic prediction
  std::unordered_map<int, vector<vector<double>>> predictions;
  // cout << "traffics size: " << traffics.size() << endl;
  for (Vehicle vehicle : traffics) {
    // predictions.insert(std::make_pair(vehicle.id_,
    // vehicle.get_prediction()));
    predictions[vehicle.id_] = vehicle.get_prediction();
  }

  std::vector<std::string> next_states = successor_states();
  std::vector<double> costs;
  std::vector<vector<vector<double>>> trajectries;

  // cout << "predictions complete" << endl;
  for (std::string state : next_states) {
    vector<vector<double>> trajectory =
        generate_trajectory(state, predictions, prev_path_size);
    if (trajectory.size() > 0) {
      double cost = 0.0;
      costs.push_back(cost);
    } else {
      costs.push_back(std::numeric_limits<double>::infinity());
    }
    trajectries.push_back(trajectory);
  }
  // cout << "generate_trajectory complete" << endl;

  double min_cost = std::numeric_limits<double>::infinity();
  int best_trajectory_idx = 0;
  for (int i = 0; i < trajectries.size(); ++i) {
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      best_trajectory_idx = i;
    }
  }
  // cout << "best_trajectory_idx:" << best_trajectory_idx << std::endl;

  prev_trajectory_s_ = trajectries[best_trajectory_idx][0];
  prev_trajectory_d_ = trajectries[best_trajectory_idx][1];
  for (int i = 0; i < prev_trajectory_s_.size(); ++i) {
    double s = prev_trajectory_s_[i];
    double d = prev_trajectory_d_[i];
    vector<double> xy =
        getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }
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

std::vector<std::vector<double>> Vehicle::get_prediction() {
  double dt = TIME_STEP;
  double elapsed_time = 0.0;
  vector<vector<double>> prediction;
  prediction.push_back({s_, d_});
  double x = x_;
  double y = y_;
  do {
    if (dt + elapsed_time > PREDICTION_TIME) {
      dt = PREDICTION_TIME - elapsed_time;
    }
    double next_x = x + x_vel_ * dt;
    double next_y = y + y_vel_ * dt;
    double next_yaw = atan2(next_y - y, next_x - x);
    vector<double> next_sd =
        getFrenet(next_x, next_y, next_yaw, map_waypoints_x_, map_waypoints_y_);
    prediction.push_back(next_sd);
    x = next_x;
    y = next_y;
    elapsed_time += dt;
  } while (elapsed_time < PREDICTION_TIME);

  return prediction;
}

vector<vector<double>> Vehicle::generate_trajectory(
    std::string state,
    std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
    int prev_path_size) {
  vector<vector<double>> trajectory;
  if (state.compare("KL") == 0) {
    return keep_lane_trajectory(prediction, prev_path_size);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    return lane_change_trajectory(prediction, prev_path_size);
  }
  return trajectory;
}

vector<vector<double>> Vehicle::generate_trajectory_to_lane(
    int lane, unordered_map<int, vector<vector<double>>> &prediction,
    int prev_path_size) {
  for (int i = 0; i < prev_path_size; ++i) {
    prev_trajectory_s_;
  }
  vector<double> trajectory_s;
  vector<double> trajectory_d;
  return {trajectory_s, trajectory_d};
}

std::vector<std::vector<double>> Vehicle::keep_lane_trajectory(
    std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
    int prev_path_size) {
  vector<vector<double>> trajectory;
  double ave_speed = speed_;
  /* if (prev_path.size() >= 2) {
    int prev_sample = 0;
    for (int i = 0; i < prev_path.size() && i < 10; ++i) {
      double theta = 0.0;
      double dist = 0.0;
      if (i == 0) {
        theta = atan2(prev_path[i][1] - y_, prev_path[i][0] - x_);
        dist = distance(prev_path[i][0], prev_path[i][1], x_, y_);
      } else {
        theta = atan2(prev_path[i][1] - prev_path[i - 1][1],
                      prev_path[i][0] - prev_path[i - 1][0]);
        dist = distance(prev_path[i][0], prev_path[i][1], prev_path[i - 1][0],
                        prev_path[i - 1][1]);
      }
      // if (i >= prev_path.size() - 5) {
      //   dist = distance(prev_path[i][0], prev_path[i][1], prev_path[i -
  1][0],
      //                   prev_path[i - 1][1]);
      //   ave_speed += dist;
      //   prev_sample++;
      //   printf("prev_sample: %d, dist: %f, \n", prev_sample, dist);
      // }
      ave_speed += dist;
      prev_sample++;

      vector<double> sd = getFrenet(prev_path[i][0], prev_path[i][1], theta,
                                    map_waypoints_x_, map_waypoints_y_);
      Vehicle vehicle(id_, prev_path[i][0], prev_path[i][1], 0.0, 0.0, sd[0],
                      sd[1], lane_);
      trajectory.push_back(vehicle);
    }
    // printf("ave_speed: %f, prev_sample: %d\n", ave_speed, prev_sample);
    ave_speed = ave_speed / prev_sample;
  } */

  double rev_vel = std::min(MAX_VEL * 0.9, speed_ + 1.0);
  rev_vel = MAX_VEL * 0.4;

  // Choose next_s
  double next_s;

  // rev_vel = MAX_VEL;
  cout << "rev_vel: " << rev_vel << endl;
  vector<double> start(3, 0.0);
  if (trajectory.empty()) {
    start[0] = s_;
    start[1] = speed_;
    start[2] = 5.0;
  } else {
    Vehicle &last_0 = trajectory[trajectory.size() - 1];
    Vehicle &last_1 = trajectory[trajectory.size() - 2];
    double yaw = atan2(last_0.y_ - last_1.y_, last_0.x_ - last_1.x_);
    vector<double> sd = getFrenet(last_0.x_, last_0.y_, yaw, map_waypoints_x_,
                                  map_waypoints_y_);
    start[0] = sd[0];
    start[1] = ave_speed;
    if (ave_speed < 0.7 * MAX_VEL) {
      start[2] = 2.0;
    }
    printf("ave_speed: %f, acc: %f\n", ave_speed, start[2]);
  }
  vector<double> end(3, 0.0);
  end[0] = start[0] + rev_vel * PREDICTION_TIME +
           0.5 * start[2] * std::pow(PREDICTION_TIME, 2);
  end[1] = rev_vel;
  end[2] = start[2];
  cout << "From " << start[0] << " to " << end[0] << endl;
  vector<double> params = jerk_minimize_trajectory(start, end, PREDICTION_TIME);
  vector<double> s_sample;
  double dt = 0.02;
  do {
    double t1 = dt;
    double t2 = pow(t1, 2);
    double t3 = pow(t1, 3);
    double t4 = pow(t1, 4);
    double t5 = pow(t1, 5);
    double s_sample = params[0] + params[1] * t1 + params[2] * t2 +
                      params[3] * t3 + params[4] * t4 + params[5] * t5;

    /* if (i == 0) {
      cout << "sample[0]: " << s_sample << endl;
    } else if (i == 29) {
      cout << "sample[29]: " << s_sample << endl;
    } */
    vector<double> xy = getXY(s_sample, d_, map_waypoints_s_, map_waypoints_x_,
                              map_waypoints_y_);
    // printf("(%f, %f)", xy[0], xy[1]);
    Vehicle vehicle(id_, xy[0], xy[1], 0.0, 0.0, s_sample, d_);
    trajectory.push_back(vehicle);
    dt += 0.02;
  } while (dt <= PREDICTION_TIME /* && trajectory.size() <= 100 */);

  /* int next_waypoint =
      NextWaypoint(x_, y_, yaw_, map_waypoints_x_, map_waypoints_y_);
  double next_s = s_ + 30.0;
  int vehicle_ahead = get_vehicle_ahead(prediction);
  if (vehicle_ahead >= 0) {
    std::vector<Vehicle> &vehicle_trajectory = prediction[vehicle_ahead];
    double safe_s = vehicle_trajectory.begin()->s_ - COLLISION_THRESHOLD;
    // cout << "next_s: " << next_s << ", safe_s: " << safe_s << endl;
    next_s = std::min(next_s, safe_s);
  }
  // cout << "curr s: " << s_ << ", next s: " << next_s << endl;
  const int sample_count = 10;
  vector<double> sample_s(sample_count, 0.0);
  vector<vector<double>> sample_xy(sample_count, vector<double>(2, 0.0));
  const double delta_s = (next_s - s_) / sample_count;
  for (int i = 0; i < sample_s.size(); ++i) {
    sample_s[i] = s_ + delta_s * i;
    vector<double> xy = getXY(sample_s[i], d_, map_waypoints_s_,
                              map_waypoints_x_, map_waypoints_y_);
    sample_xy[i] = xy;
  }
  std::sort(sample_xy.begin(), sample_xy.end(),
            [](const vector<double> &a, const vector<double> &b) {
              return a[0] < b[0];
            });
  vector<double> sample_x;
  vector<double> sample_y;
  for (int i = 0; i < sample_xy.size(); ++i) {
    sample_x.push_back(sample_xy[i][0]);
    sample_y.push_back(sample_xy[i][1]);
  }
  tk::spline sample_spline;
  sample_spline.set_points(sample_x, sample_y);

  std::vector<Vehicle> trajectory;
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
  } while (s < next_s); */

  //   cout << "keep_lane_trajectory--" << endl;
  return trajectory;
}

std::vector<std::vector<double>> Vehicle::lane_change_trajectory(
    std::unordered_map<int, std::vector<std::vector<double>>> &prediction,
    int prev_path_size) {
  vector<vector<double>> trajectory;
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
  // double s_lower = s_;
  // double s_upper = s_ + 50.0;
  for (auto iter = prediction.begin(); iter != prediction.end(); ++iter) {
    const Vehicle &vehicle = iter->second[0];
    double dist = distance(x_, y_, vehicle.x_, vehicle.y_);
    if (vehicle.lane_ == this->lane_ && vehicle.s_ > this->s_ &&
        vehicle.s_ < min_s) {
      min_s = vehicle.s_;
      id = vehicle.id_;
    }
  }

  return id;
}

std::vector<double>
Vehicle::jerk_minimize_trajectory(std::vector<double> &start,
                                  std::vector<double> &end, double dt) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time dt.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param dt - The duration, in seconds, over which this maneuver should
   * occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  vector<double> answer(6, 0.0);
  answer[0] = start[0];
  answer[1] = start[1];
  answer[2] = 0.5 * start[2];

  Matrix3d A;
  Vector3d b;
  A << pow(dt, 3), pow(dt, 4), pow(dt, 5), 3 * pow(dt, 2), 4 * pow(dt, 3),
      5 * pow(dt, 4), 6 * dt, 12 * pow(dt, 2), 20 * pow(dt, 3);
  b << end[0] - (start[0] + start[1] * dt + 0.5 * start[2] * pow(dt, 2)),
      end[1] - (start[1] + start[2] * dt), end[2] - start[2];
  Vector3d x = A.colPivHouseholderQr().solve(b);
  answer[3] = x[0];
  answer[4] = x[1];
  answer[5] = x[2];

  return answer;

  /* MatrixXd A = MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T, 3 * T * T, 4 * T * T * T,
      5 * T * T * T * T, 6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
      end[1] - (start[1] + start[2] * T), end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = {start[0], start[1], .5 * start[2]};
  for (int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result; */
}