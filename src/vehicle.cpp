#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "spline.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::cout;
using std::endl;
using std::pow;
using std::unordered_map;

const double Vehicle::MAX_VEL = 21.45;
const double Vehicle::MIN_VEL = 10.0;
const double Vehicle::MAX_ACC = 9.0;
const double Vehicle::MAX_JERK = 9.0;
const double Vehicle::VEHICLE_RADIUS = 5.0;
const double Vehicle::LANE_WIDTH = 4.0;
const double Vehicle::MAX_S = 6945.554;
const double Vehicle::COLLISION_THRESHOLD = 10.0;
const double Vehicle::PREDICTION_TIME = 3.0;
const double Vehicle::TIME_STEP = 0.02;

Vehicle::Vehicle() {
  s_vel_ = 0.0;
  d_vel_ = 0.0;
  s_acc_ = 0.0;
  d_acc_ = 0.0;
  //   state_ = "CS";
  target_lane_ = -1;
}

Vehicle::Vehicle(int id, double x, double y, double x_vel, double y_vel,
                 double s, double d) {
  id_ = id;
  x_ = x;
  y_ = y;
  x_vel_ = x_vel;
  y_vel_ = y_vel;
  speed_ = std::sqrt(x_vel * x_vel + y_vel * y_vel);
  s_ = s;
  d_ = d;
  lane_ = get_lane_from_d(d);
  target_lane_ = lane_;
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
  if (target_lane_ < 0) {
    target_lane_ = lane_;
  } else if (lane_ != target_lane_) {
    // Confirm that the lane change task has been completed
    double diff = fabs(d_ - (target_lane_ + 0.5) * LANE_WIDTH);
    // printf("diff: %f = %f - %f\n", diff, d_, (target_lane_ + 0.5) *
    // LANE_WIDTH);
    if (fabs(d_ - (target_lane_ + 0.5) * LANE_WIDTH) < 0.3) {
      target_lane_ = lane_;
    }
  }
  //   cout << "curr s: " << s_ << endl;
}

void Vehicle::get_trajectory(std::vector<double> &next_x_vals,
                             std::vector<double> &next_y_vals,
                             int prev_path_size,
                             unordered_map<int, Vehicle> &traffics) {
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
  for (auto iter = traffics.begin(); iter != traffics.end(); ++iter) {
    predictions[iter->first] = iter->second.get_prediction();
  }

  // Generate all possible trajectory
  vector<int> target_lanes;
  /* if (lane_ == target_lane_) {
    for (int i = 0; i < LANE_AVAILABLE; ++i) {
      target_lanes.push_back(i);
    }
  } else {
    target_lanes.push_back(lane_);
    target_lanes.push_back(target_lane_);
  } */
  for (int i = 0; i < LANE_AVAILABLE; ++i) {
    target_lanes.push_back(i);
  }
  std::unordered_map<int, vector<vector<double>>> next_trajectories;
  for (int i = 0; i < target_lanes.size(); ++i) {
    int target_lane = target_lanes[i];
    next_trajectories.insert(std::make_pair(
        target_lane,
        generate_trajectory_to_lane_v1(target_lane, traffics, prev_path_size)));
  }

  auto collision_cost = [&](vector<vector<double>> &trajectory) {
    const double cost = 100.0;
    vector<double> &s_trajectory = trajectory[0];
    vector<double> &d_trajectory = trajectory[1];
    for (int i = 0; i < trajectory.size(); ++i) {
      double s = s_trajectory[i];
      double d = d_trajectory[i];
      int ego_lane = get_lane_from_d(d);
      // vector<double> xy =
      //     getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
      for (auto iter = predictions.begin(); iter != predictions.end(); ++iter) {
        vector<vector<double>> &prediction = iter->second;
        double s1 = prediction[0][i];
        double d1 = prediction[1][i];
        int vehicle_lane = get_lane_from_d(d1);
        if (ego_lane == vehicle_lane && i == 0) {
          printf("id: %d, s: %f, s1: %f\n", iter->first, s, s1);
        }
        // double dist = distance(x1, y1, xy[0], xy[1]);
        if (ego_lane == vehicle_lane && s < s1 && s1 < s + 25.0) {
          printf("collide, id: %d, i: %d, s: %f, s1: %f\n", iter->first, i, s,
                 s1);
          return cost;
        }
      }
    }
    return 0.0;
  };

  auto lane_change_cost = [&](int target_lane) {
    const double cost = 4.0;
    if (target_lane == (lane_ + 1) || target_lane == (lane_ - 1)) {
      return 4.0;
    } else if (target_lane == (lane_ + 2) || target_lane == (lane_ - 2)) {
      return 6.0;
    }
    return 0.0;
  };

  auto speed_cost = [&](vector<vector<double>> &trajectory) {
    double cost;
    vector<double> &s_trajectory = trajectory[0];
    int size = s_trajectory.size();
    double s0 = s_trajectory[size - 1];
    double s1 = s_trajectory[size - 2];
    double s2 = s_trajectory[size - 3];
    double v0 = (s0 - s1) / TIME_STEP;
    double v1 = (s1 - s2) / TIME_STEP;
    double ave_speed = (v0 + v1) / 2;
    printf("speed score: %f\n", (MAX_VEL - ave_speed));
    if (ave_speed > MAX_VEL) {
      return 10.0;
    } else if (ave_speed < MAX_VEL * 0.85) {
      return (MAX_VEL - ave_speed) * 1.0;
    }
    return 0.0;
  };

  auto change_plan_cost = [&](int target_lane) {
    if (target_lane != target_lane_) {
      return 6.0;
    }

    return 0.0;
  };

  // Compute the cost of each trajectory
  printf("Curr lane: %d, target_lane: %d\n", lane_, target_lane_);
  auto best_iter = next_trajectories.begin();
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto iter = next_trajectories.begin(); iter != next_trajectories.end();
       ++iter) {
    int target_lane = iter->first;
    vector<vector<double>> &trajectory = iter->second;
    printf("lane: %d\n", target_lane);
    double cost1 = collision_cost(trajectory);
    printf("collision cost: %f\n", cost1);
    double cost2 = lane_change_cost(target_lane);
    printf("change lane cost: %f\n", cost2);
    double cost3 = speed_cost(trajectory);
    printf("speed cost: %f\n", cost3);
    double cost4 = change_plan_cost(target_lane);
    printf("Change plan cost: %f\n\n", cost4);
    double total_cost = cost1 + cost2 + cost3 + cost4;
    printf("total cost: %f\n\n", total_cost);
    if (total_cost < min_cost) {
      min_cost = total_cost;
      best_iter = iter;
    }
  }
  vector<vector<double>> &trajectory_sd = best_iter->second;
  if (best_iter->first != lane_) {
    target_lane_ = best_iter->first;
    printf("Change to %d\n", target_lane_);
  }
  /* if (lane_ == 1) {
    if (speed_ > 10.0 && fabs(d_ - (target_lane_ + 0.5) * LANE_WIDTH) < 0.1) {
      target_lane_ = lane_ - 1;
    }
  } else if (lane_ == 0) {
    if (speed_ > 10.0 && fabs(d_ - (target_lane_ + 0.5) * LANE_WIDTH) < 0.1) {
      target_lane_ = lane_ + 1;
    }
  }
  trajectory_sd =
      generate_trajectory_to_lane(target_lane_, traffics, prev_path_size); */
  // cout << "generate_trajectory_to_lane" << endl;
  // cout << "traffics size: " << traffics.size() << endl;

  //   std::vector<std::string> next_states = successor_states();
  //   std::vector<double> costs;
  //   std::vector<vector<vector<double>>> trajectries;

  //   // cout << "predictions complete" << endl;
  //   for (std::string state : next_states) {
  //     vector<vector<double>> trajectory =
  //         generate_trajectory(state, predictions, prev_path_size);
  //     if (trajectory.size() > 0) {
  //       double cost = 0.0;
  //       costs.push_back(cost);
  //     } else {
  //       costs.push_back(std::numeric_limits<double>::infinity());
  //     }
  //     trajectries.push_back(trajectory);
  //   }
  //   // cout << "generate_trajectory complete" << endl;

  //   double min_cost = std::numeric_limits<double>::infinity();
  //   int best_trajectory_idx = 0;
  //   for (int i = 0; i < trajectries.size(); ++i) {
  //     if (costs[i] < min_cost) {
  //       min_cost = costs[i];
  //       best_trajectory_idx = i;
  //     }
  //   }
  //   // cout << "best_trajectory_idx:" << best_trajectory_idx << std::endl;

  // prev_trajectory_s_ = trajectries[best_trajectory_idx][0];
  // prev_trajectory_d_ = trajectries[best_trajectory_idx][1];
  prev_trajectory_s_ = trajectory_sd[0];
  prev_trajectory_d_ = trajectory_sd[1];
  vector<double> prev_xy;
  double prev_s;
  for (int i = 0; i < prev_trajectory_s_.size(); ++i) {
    double s = prev_trajectory_s_[i];
    double d = prev_trajectory_d_[i];
    vector<double> xy =
        getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    /* if (!prev_xy.empty()) {
      double dist = distance(xy[0], xy[1], prev_xy[0], prev_xy[1]);
      printf("x: %f, y: %f, dist: %f, dist_s: %f\n", xy[0], xy[1], dist,
             s - prev_s);
    } */
    prev_xy = xy;
    prev_s = s;
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
  /* if (this->state_ == KL) {
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
  int total_step = PREDICTION_TIME / TIME_STEP;
  vector<double> s_trajectory;
  vector<double> d_trajectory;
  double x = x_;
  double y = y_;
  for (int i = 0; i < total_step; ++i) {
    double next_x = x_ + x_vel_ * TIME_STEP;
    double next_y = y_ + y_vel_ * TIME_STEP;
    double next_yaw = atan2(next_y - y, next_x - x);
    vector<double> next_sd =
        getFrenet(next_x, next_y, next_yaw, map_waypoints_x_, map_waypoints_y_);
    s_trajectory.push_back(next_sd[0]);
    d_trajectory.push_back(next_sd[1]);
    x = next_x;
    y = next_y;
  }
  vector<vector<double>> prediction;
  prediction.push_back(s_trajectory);
  prediction.push_back(d_trajectory);

  /* int total_step = PREDICTION_TIME / TIME_STEP;
  double elapsed_time = TIME_STEP;
  vector<double> x_trajectory;
  vector<double> y_trajectory;
  for (int i = 0; i < total_step; ++i) {
    double next_x = x_ + x_vel_ * elapsed_time;
    double next_y = y_ + y_vel_ * elapsed_time;
    x_trajectory.push_back(next_x);
    y_trajectory.push_back(next_y);
    elapsed_time += TIME_STEP;
  }
  vector<vector<double>> prediction;
  prediction.push_back(x_trajectory);
  prediction.push_back(y_trajectory); */

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
    int target_lane, unordered_map<int, Vehicle> &traffics,
    int prev_path_size) {
  auto derivative = [&](const vector<double> &coeff) {
    vector<double> derivative_coeffs;
    for (int i = 1; i < coeff.size(); i++) {
      derivative_coeffs.push_back(i * coeff[i]);
    }
    return derivative_coeffs;
  };

  auto poly_eval = [&](double x, const vector<double> &coeffs) {
    double result = 0.0;
    double t = 1.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * t;
      t *= x;
    }
    return result;
  };

  auto check_s_jmt = [&](const vector<double> &s_coeffs, double duration) {
    vector<double> s_vel_coeffs = derivative(s_coeffs);
    vector<double> s_acc_coeffs = derivative(s_vel_coeffs);
    vector<double> s_jerk_coeffs = derivative(s_acc_coeffs);
    for (double t = TIME_STEP; t <= duration; t += TIME_STEP) {
      double s_vel = poly_eval(t, s_vel_coeffs);
      double s_acc = fabs(poly_eval(t, s_acc_coeffs));
      double s_jerk = fabs(poly_eval(t, s_jerk_coeffs));
      // printf("duration: %f, s_vel: %f, s_acc: %f, s_jerk: %f\n", duration,
      //        s_vel, s_acc, s_jerk);
      if (s_vel >= MAX_VEL || s_vel <= 0.0 || s_acc >= MAX_ACC ||
          s_jerk >= MAX_JERK) {
        return false;
      }
    }
    return true;
  };

  auto check_d_jmt = [&](const vector<double> &d_coeffs, double duration) {
    vector<double> d_vel_coeffs = derivative(d_coeffs);
    vector<double> d_acc_coeffs = derivative(d_vel_coeffs);
    vector<double> d_jerk_coeffs = derivative(d_acc_coeffs);
    // printf("duration: %f\n", duration);
    for (double t = TIME_STEP; t <= duration; t += TIME_STEP) {
      double d_vel = fabs(poly_eval(t, d_vel_coeffs));
      double d_acc = fabs(poly_eval(t, d_acc_coeffs));
      double d_jerk = fabs(poly_eval(t, d_jerk_coeffs));
      if (d_vel >= MAX_VEL || d_acc >= MAX_ACC || d_jerk >= MAX_JERK) {
        // printf("Fail, t: %f, d_vel: %f, d_acc: %f, d_jerk: %f\n", t, d_vel,
        //        d_acc, d_jerk);
        return false;
      }
    }
    return true;
  };

  vector<double> trajectory_s;
  vector<double> trajectory_d;
  if (prev_path_size >= 2) {
    trajectory_s.insert(trajectory_s.begin(),
                        prev_trajectory_s_.begin() +
                            (prev_trajectory_s_.size() - prev_path_size),
                        prev_trajectory_s_.end());
    trajectory_d.insert(trajectory_d.begin(),
                        prev_trajectory_d_.begin() +
                            (prev_trajectory_d_.size() - prev_path_size),
                        prev_trajectory_d_.end());
  }

  vector<double> start_s(3, 0.0);
  vector<double> end_s(3, 0.0);
  vector<double> start_d(3, 0.0);
  vector<double> end_d(3, 0.0);
  if (trajectory_s.size() <= 2) {
    start_s[0] = s_;
    start_s[1] = speed_;
    start_s[2] = 0.0;
    start_d[0] = d_;
    start_d[1] = 0.0;
    start_d[2] = 0.0;
  } else {
    double s_0 = trajectory_s[prev_path_size - 1];
    double s_1 = trajectory_s[prev_path_size - 2];
    double s_2 = trajectory_s[prev_path_size - 3];
    double s_vel_0 = (s_0 - s_1) / TIME_STEP;
    double s_vel_1 = (s_1 - s_2) / TIME_STEP;
    double s_acc_0 = (s_vel_0 - s_vel_1) / TIME_STEP;
    double d_0 = trajectory_d[prev_path_size - 1];
    double d_1 = trajectory_d[prev_path_size - 2];
    double d_2 = trajectory_d[prev_path_size - 3];
    double d_vel_0 = (d_0 - d_1) / TIME_STEP;
    double d_vel_1 = (d_1 - d_2) / TIME_STEP;
    double d_acc_0 = (d_vel_0 - d_vel_1) / TIME_STEP;
    start_s[0] = s_0;
    start_s[1] = s_vel_0;
    start_s[2] = s_acc_0;
    start_d[0] = d_0;
    start_d[1] = d_vel_0;
    start_d[2] = d_acc_0;
  }

  int total_step = PREDICTION_TIME / TIME_STEP;
  int vehicle_ahead_id = get_vehicle_ahead(target_lane, traffics);
  end_s[0] = s_ + 100.0;
  end_s[1] = MAX_VEL;
  end_d[0] = (target_lane + 0.5) * LANE_WIDTH;
  if (target_lane != lane_) {
    if (target_lane > lane_) {
      end_d[1] = 0.3;
      end_d[2] = 0.3;
    } else {
      end_d[1] = -0.3;
      end_d[2] = -0.3;
    }
  } else {
    end_d[1] = 0.0;
    end_d[2] = 0.0;
  }
  // printf("target lane: %d\n", target_lane);
  if (vehicle_ahead_id >= 0) {
    Vehicle vehicle = traffics[vehicle_ahead_id];
    double predict_time = (total_step - prev_path_size) * TIME_STEP;
    double collision_time_1 = 1.0;
    double collision_time_2 = 2.0;
    double vehicle_predict_s = vehicle.s_ + vehicle.speed_ * predict_time;
    double ego_predict_s_1 = s_ + speed_ * collision_time_1 + 5.0;
    double ego_predict_s_2 = s_ + speed_ * collision_time_2 + 5.0;
    // printf("end_s[0]: %f, vehicle_predict_s: %f\n", end_s[0],
    //        vehicle_predict_s);
    if (ego_predict_s_1 < vehicle_predict_s &&
        vehicle_predict_s <= ego_predict_s_2) {
      // printf("Follow vehicle\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_);
    } else if (ego_predict_s_1 >= vehicle_predict_s) {
      // printf("Far away vehicle\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_ * 0.8);
    }
  }
  end_s[2] = 0.0;
  // end_d[2] = 0.0;
  // printf("s, start: %f, %f, %f\n", start_s[0], start_s[1], start_s[2]);
  // printf("s: end: %f, %f, %f\n", end_s[0], end_s[1], end_s[2]);
  // printf("d, start: %f, %f, %f\n", start_d[0], start_d[1], start_d[2]);
  // printf("d: end: %f, %f, %f\n\n", end_d[0], end_d[1], end_d[2]);

  vector<vector<double>> candidate_s_jmt_params;
  vector<vector<double>> candidate_d_jmt_params;
  for (double dt = 0.1; dt <= 20.0; dt += 0.1) {
    vector<double> s_jmt_coeffs = jerk_minimize_trajectory(start_s, end_s, dt);
    if (check_s_jmt(s_jmt_coeffs, dt)) {
      candidate_s_jmt_params.push_back(s_jmt_coeffs);
    }

    vector<double> d_jmt_coeffs = jerk_minimize_trajectory(start_d, end_d, dt);
    if (check_d_jmt(d_jmt_coeffs, dt)) {
      candidate_d_jmt_params.push_back(d_jmt_coeffs);
    }
  }

  // printf("candidate_d_jmt_params size: %d\n", candidate_d_jmt_params.size());
  if (!candidate_s_jmt_params.empty() && !candidate_d_jmt_params.empty()) {
    // Choose the best jmt_s_params
    int best_s_idx = 0;
    double min_target_s_speed_diff = 1000.0;
    for (int i = 0; i < candidate_s_jmt_params.size(); ++i) {
      vector<double> &s_coeffs = candidate_s_jmt_params[i];
      vector<double> s_vel_coeffs = derivative(s_coeffs);
      double t = (total_step - prev_path_size) * TIME_STEP;
      double speed = poly_eval(t, s_vel_coeffs);
      if (fabs(end_s[1] - speed) < min_target_s_speed_diff) {
        min_target_s_speed_diff = fabs(end_s[1] - speed);
        best_s_idx = i;
      }
    }
    vector<double> &final_s_coeffs = candidate_s_jmt_params[best_s_idx];

    // Choose the best jmt_s_params
    int best_d_idx = 0;
    double min_target_d_speed_diff = 1000.0;
    for (int i = 0; i < candidate_d_jmt_params.size(); ++i) {
      vector<double> &d_coeffs = candidate_d_jmt_params[i];
      vector<double> d_vel_coeffs = derivative(d_coeffs);
      double t = (total_step - prev_path_size) * TIME_STEP;
      double speed = poly_eval(t, d_vel_coeffs);
      if (fabs(end_d[1] - speed) < min_target_d_speed_diff) {
        min_target_d_speed_diff = fabs(end_d[1] - speed);
        best_d_idx = i;
      }
    }
    vector<double> &final_d_coeffs = candidate_d_jmt_params[best_d_idx];

    for (int i = 0; (i + prev_path_size) < total_step; ++i) {
      double t = (i + 1) * TIME_STEP;
      double s = poly_eval(t, final_s_coeffs);
      double d = poly_eval(t, final_d_coeffs);
      // printf("t: %f, s: %f\n", t, s);
      trajectory_s.push_back(s);
      trajectory_d.push_back(d);
    }
  }

  return {trajectory_s, trajectory_d};
}

std::vector<std::vector<double>> Vehicle::generate_trajectory_to_lane_v1(
    int target_lane, std::unordered_map<int, Vehicle> &traffics,
    int prev_path_size) {
  auto derivative = [&](const vector<double> &coeff) {
    vector<double> derivative_coeffs;
    for (int i = 1; i < coeff.size(); i++) {
      derivative_coeffs.push_back(i * coeff[i]);
    }
    return derivative_coeffs;
  };

  auto poly_eval = [&](double x, const vector<double> &coeffs) {
    double result = 0.0;
    double t = 1.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * t;
      t *= x;
    }
    return result;
  };

  auto check_s_jmt = [&](const vector<double> &s_coeffs, double duration) {
    vector<double> s_vel_coeffs = derivative(s_coeffs);
    vector<double> s_acc_coeffs = derivative(s_vel_coeffs);
    vector<double> s_jerk_coeffs = derivative(s_acc_coeffs);
    // printf("check_s_jmt duration: %f\n", duration);
    for (double t = TIME_STEP; t <= duration; t += TIME_STEP) {
      double s_vel = poly_eval(t, s_vel_coeffs);
      double s_acc = fabs(poly_eval(t, s_acc_coeffs));
      double s_jerk = fabs(poly_eval(t, s_jerk_coeffs));
      // printf("t: %f, s_vel: %f, s_acc: %f, s_jerk: %f\n", t, s_vel, s_acc,
      //        s_jerk);
      if (s_vel >= MAX_VEL || s_vel <= 0.0 || s_acc >= MAX_ACC ||
          s_jerk >= MAX_JERK) {
        // printf("\n");
        return false;
      }
    }
    // printf("\n");
    return true;
  };

  auto check_d_jmt = [&](const vector<double> &d_coeffs, double duration) {
    vector<double> d_vel_coeffs = derivative(d_coeffs);
    vector<double> d_acc_coeffs = derivative(d_vel_coeffs);
    vector<double> d_jerk_coeffs = derivative(d_acc_coeffs);
    // printf("duration: %f\n", duration);
    for (double t = TIME_STEP; t <= duration; t += TIME_STEP) {
      double d_vel = fabs(poly_eval(t, d_vel_coeffs));
      double d_acc = fabs(poly_eval(t, d_acc_coeffs));
      double d_jerk = fabs(poly_eval(t, d_jerk_coeffs));
      if (d_vel >= MAX_VEL || d_acc >= MAX_ACC || d_jerk >= MAX_JERK) {
        // printf("Fail, t: %f, d_vel: %f, d_acc: %f, d_jerk: %f\n", t, d_vel,
        //        d_acc, d_jerk);
        return false;
      }
    }
    return true;
  };

  vector<double> trajectory_s;
  vector<double> trajectory_d;
  int keep_prev_size = 50;
  if (prev_path_size >= keep_prev_size) {
    auto s_start = prev_trajectory_s_.begin() +
                   (prev_trajectory_s_.size() - prev_path_size);
    auto s_end = s_start + keep_prev_size;
    auto d_start = prev_trajectory_d_.begin() +
                   (prev_trajectory_d_.size() - prev_path_size);
    auto d_end = d_start + keep_prev_size;
    trajectory_s.insert(trajectory_s.begin(), s_start, s_end);
    trajectory_d.insert(trajectory_d.begin(), d_start, d_end);
  }

  vector<double> start_s(3, 0.0);
  vector<double> end_s(3, 0.0);
  vector<double> start_d(3, 0.0);
  vector<double> end_d(3, 0.0);
  if (trajectory_s.size() <= 2) {
    start_s[0] = s_;
    start_s[1] = speed_;
    start_s[2] = 0.0;
    start_d[0] = d_;
    start_d[1] = 0.0;
    start_d[2] = 0.0;
  } else {
    double s_0 = trajectory_s[keep_prev_size - 1];
    double s_1 = trajectory_s[keep_prev_size - 2];
    double s_2 = trajectory_s[keep_prev_size - 3];
    double s_vel_0 = (s_0 - s_1) / TIME_STEP;
    double s_vel_1 = (s_1 - s_2) / TIME_STEP;
    double s_acc_0 = (s_vel_0 - s_vel_1) / TIME_STEP;
    double d_0 = trajectory_d[keep_prev_size - 1];
    double d_1 = trajectory_d[keep_prev_size - 2];
    double d_2 = trajectory_d[keep_prev_size - 3];
    double d_vel_0 = (d_0 - d_1) / TIME_STEP;
    double d_vel_1 = (d_1 - d_2) / TIME_STEP;
    double d_acc_0 = (d_vel_0 - d_vel_1) / TIME_STEP;
    start_s[0] = s_0;
    start_s[1] = s_vel_0;
    start_s[2] = s_acc_0;
    start_d[0] = d_0;
    start_d[1] = d_vel_0;
    start_d[2] = d_acc_0;
  }

  int total_step = PREDICTION_TIME / TIME_STEP;
  int vehicle_ahead_id = get_vehicle_ahead(target_lane, traffics);
  end_s[0] = s_ + 100.0;
  end_s[1] = (speed_ <= 0.1) ? 0.5 : std::min(MAX_VEL, start_s[1] * 1.2);
  end_d[0] = (target_lane + 0.5) * LANE_WIDTH;
  if (target_lane != lane_) {
    if (target_lane > lane_) {
      end_d[1] = 0.3;
      end_d[2] = 0.3;
    } else {
      end_d[1] = -0.3;
      end_d[2] = -0.3;
    }
  } else {
    end_d[1] = 0.0;
    end_d[2] = 0.0;
  }
  printf("generate lane: %d\n", target_lane);
  if (vehicle_ahead_id >= 0) {
    Vehicle vehicle = traffics[vehicle_ahead_id];
    double predict_time = (total_step - keep_prev_size) * TIME_STEP;
    double collision_time_1 = 1.0;
    double collision_time_2 = 2.0;
    double vehicle_s1 = vehicle.s_ + vehicle.speed_ * collision_time_1;
    double vehicle_s2 = vehicle.s_ + vehicle.speed_ * collision_time_2;
    double ego_s1 = s_ + speed_ * collision_time_1 + 25.0;
    double ego_s2 = s_ + speed_ * collision_time_2 + 25.0;
    printf("ego_1: %f, ego_2: %f\n", ego_s1, ego_s1);
    printf("vehicle1: %f, vehicle2: %f\n", vehicle_s1, vehicle_s2);
    if (vehicle_s1 <= ego_s1) {
      printf("Far away vehicle\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_ * 0.8);
    } else if (vehicle_s2 <= ego_s2) {
      printf("Follow vehicle\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_);
    }
  }
  end_s[2] = 0.0;
  // end_d[2] = 0.0;
  printf("s, start: %f, %f, %f\n", start_s[0], start_s[1], start_s[2]);
  printf("s: end: %f, %f, %f\n", end_s[0], end_s[1], end_s[2]);
  // printf("d, start: %f, %f, %f\n", start_d[0], start_d[1], start_d[2]);
  // printf("d: end: %f, %f, %f\n\n", end_d[0], end_d[1], end_d[2]);

  vector<vector<double>> candidate_s_jmt_params;
  vector<vector<double>> candidate_d_jmt_params;
  for (double dt = 0.1; dt <= 20.0; dt += 0.1) {
    vector<double> s_jmt_coeffs = jerk_minimize_trajectory(start_s, end_s, dt);
    if (check_s_jmt(s_jmt_coeffs, dt)) {
      candidate_s_jmt_params.push_back(s_jmt_coeffs);
    }

    vector<double> d_jmt_coeffs = jerk_minimize_trajectory(start_d, end_d, dt);
    if (check_d_jmt(d_jmt_coeffs, dt)) {
      candidate_d_jmt_params.push_back(d_jmt_coeffs);
    }
  }

  printf("candidate_s_jmt_params size: %d\n", candidate_s_jmt_params.size());
  printf("candidate_d_jmt_params size: %d\n", candidate_d_jmt_params.size());
  if (!candidate_s_jmt_params.empty() && !candidate_d_jmt_params.empty()) {
    // Choose the best jmt_s_params
    int best_s_idx = 0;
    double min_target_s_speed_diff = 1000.0;
    for (int i = 0; i < candidate_s_jmt_params.size(); ++i) {
      vector<double> &s_coeffs = candidate_s_jmt_params[i];
      vector<double> s_vel_coeffs = derivative(s_coeffs);
      double t = (total_step - keep_prev_size) * TIME_STEP;
      double speed = poly_eval(t, s_vel_coeffs);
      if (fabs(end_s[1] - speed) < min_target_s_speed_diff) {
        min_target_s_speed_diff = fabs(end_s[1] - speed);
        best_s_idx = i;
      }
    }
    vector<double> &final_s_coeffs = candidate_s_jmt_params[best_s_idx];

    // Choose the best jmt_s_params
    int best_d_idx = 0;
    double min_target_d_speed_diff = 1000.0;
    for (int i = 0; i < candidate_d_jmt_params.size(); ++i) {
      vector<double> &d_coeffs = candidate_d_jmt_params[i];
      vector<double> d_vel_coeffs = derivative(d_coeffs);
      double t = (total_step - keep_prev_size) * TIME_STEP;
      double speed = poly_eval(t, d_vel_coeffs);
      if (fabs(end_d[1] - speed) < min_target_d_speed_diff) {
        min_target_d_speed_diff = fabs(end_d[1] - speed);
        best_d_idx = i;
      }
    }
    vector<double> &final_d_coeffs = candidate_d_jmt_params[best_d_idx];

    for (int i = 0; (i + keep_prev_size) < total_step; ++i) {
      double t = (i + 1) * TIME_STEP;
      double s = poly_eval(t, final_s_coeffs);
      double d = poly_eval(t, final_d_coeffs);
      // printf("t: %f, s: %f\n", t, s);
      trajectory_s.push_back(s);
      trajectory_d.push_back(d);
    }
  }

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

  // double rev_vel = std::min(MAX_VEL * 0.9, speed_ + 1.0);
  // rev_vel = MAX_VEL * 0.4;

  // // Choose next_s
  // double next_s;

  // // rev_vel = MAX_VEL;
  // cout << "rev_vel: " << rev_vel << endl;
  // vector<double> start(3, 0.0);
  // if (trajectory.empty()) {
  //   start[0] = s_;
  //   start[1] = speed_;
  //   start[2] = 5.0;
  // } else {
  //   Vehicle &last_0 = trajectory[trajectory.size() - 1];
  //   Vehicle &last_1 = trajectory[trajectory.size() - 2];
  //   double yaw = atan2(last_0.y_ - last_1.y_, last_0.x_ - last_1.x_);
  //   vector<double> sd = getFrenet(last_0.x_, last_0.y_, yaw,
  //   map_waypoints_x_,
  //                                 map_waypoints_y_);
  //   start[0] = sd[0];
  //   start[1] = ave_speed;
  //   if (ave_speed < 0.7 * MAX_VEL) {
  //     start[2] = 2.0;
  //   }
  //   printf("ave_speed: %f, acc: %f\n", ave_speed, start[2]);
  // }
  // vector<double> end(3, 0.0);
  // end[0] = start[0] + rev_vel * PREDICTION_TIME +
  //          0.5 * start[2] * std::pow(PREDICTION_TIME, 2);
  // end[1] = rev_vel;
  // end[2] = start[2];
  // cout << "From " << start[0] << " to " << end[0] << endl;
  // vector<double> params = jerk_minimize_trajectory(start, end,
  // PREDICTION_TIME); vector<double> s_sample; double dt = 0.02; do {
  //   double t1 = dt;
  //   double t2 = pow(t1, 2);
  //   double t3 = pow(t1, 3);
  //   double t4 = pow(t1, 4);
  //   double t5 = pow(t1, 5);
  //   double s_sample = params[0] + params[1] * t1 + params[2] * t2 +
  //                     params[3] * t3 + params[4] * t4 + params[5] * t5;

  //   /* if (i == 0) {
  //     cout << "sample[0]: " << s_sample << endl;
  //   } else if (i == 29) {
  //     cout << "sample[29]: " << s_sample << endl;
  //   } */
  //   vector<double> xy = getXY(s_sample, d_, map_waypoints_s_,
  //   map_waypoints_x_,
  //                             map_waypoints_y_);
  //   // printf("(%f, %f)", xy[0], xy[1]);
  //   Vehicle vehicle(id_, xy[0], xy[1], 0.0, 0.0, s_sample, d_);
  //   trajectory.push_back(vehicle);
  //   dt += 0.02;
  // } while (dt <= PREDICTION_TIME /* && trajectory.size() <= 100 */);

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

/* int Vehicle::get_vehicle_behind(const unordered_map<int, Vehicle> &traffics)
{
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double max_s = -1.0;
  int id = -1;
  for (auto iter = traffics.begin(); iter != traffics.end(); ++iter) {
    const Vehicle &vehicle = iter->second;
    double s = vehicle.s_;
    double d = vehicle.d_;
    double lane = vehicle.lane_;
    if (lane == this->lane_ && s < this->s_ && s > max_s) {
      max_s = s;
      id = vehicle.id_;
    }
  }

  return id;
} */

int Vehicle::get_vehicle_ahead(int target_lane,
                               const unordered_map<int, Vehicle> &traffics) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double min_s = std::numeric_limits<double>::infinity();
  int id = -1;
  for (auto iter = traffics.begin(); iter != traffics.end(); ++iter) {
    const Vehicle &vehicle = iter->second;
    double s = vehicle.s_;
    double d = vehicle.d_;
    double lane = vehicle.lane_;
    if (lane == target_lane && s > this->s_ && s < min_s) {
      min_s = s;
      id = vehicle.id_;
    }
  }

  return id;
}

std::vector<double> Vehicle::jerk_minimize_trajectory(
    std::vector<double> &start, std::vector<double> &end, double dt) {
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