#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "spline.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::cout;
using std::endl;
using std::list;
using std::pow;
using std::unordered_map;

const double Vehicle::MAX_VEL = 21.45;
const double Vehicle::MIN_VEL = 10.0;
const double Vehicle::MAX_ACC = 9.0;
const double Vehicle::MAX_JERK = 9.0;
const double Vehicle::VEHICLE_RADIUS = 5.0;
const double Vehicle::LANE_WIDTH = 4.0;
const double Vehicle::MAX_S = 6945.554;
const double Vehicle::HALF_MAX_S = MAX_S / 2.0;
const double Vehicle::COLLISION_THRESHOLD = 10.0;
const double Vehicle::PREDICTION_TIME = 3.0;
const double Vehicle::TIME_STEP = 0.02;
const int Vehicle::TOTAL_STEP = PREDICTION_TIME / TIME_STEP;

Vehicle::Vehicle() { target_lane_ = -1; }

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
  generate_splines();
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
  /* for (int i = 0; i < LANE_AVAILABLE; ++i) {
    target_lanes.push_back(i);
  } */
  if (lane_ == 0) {
    target_lanes.push_back(lane_);
    target_lanes.push_back(lane_ + 1);
  } else if (lane_ == LANE_AVAILABLE) {
    target_lanes.push_back(lane_ - 1);
    target_lanes.push_back(lane_);
  } else {
    target_lanes.push_back(lane_ - 1);
    target_lanes.push_back(lane_);
    target_lanes.push_back(lane_ + 1);
  }

  std::unordered_map<int, vector<vector<double>>> trajectories;
  for (int i = 0; i < target_lanes.size(); ++i) {
    int target_lane = target_lanes[i];
    trajectories.insert(std::make_pair(
        target_lane,
        generate_trajectory(target_lane, traffics, prev_path_size)));
  }

  auto collision_cost = [&](vector<vector<double>> &trajectory) {
    vector<double> &s_trajectory = trajectory[0];
    vector<double> &d_trajectory = trajectory[1];
    for (int i = 0; i < trajectory.size(); ++i) {
      double s0 = s_trajectory[i];
      double d0 = d_trajectory[i];
      int ego_lane = get_lane_from_d(d0);
      // vector<double> xy =
      //     getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
      for (auto iter = predictions.begin(); iter != predictions.end(); ++iter) {
        vector<vector<double>> &prediction = iter->second;
        double s1 = prediction[0][i];
        double d1 = prediction[1][i];
        int vehicle_lane = get_lane_from_d(d1);
        double s_dist = s_distance(s0, s1, MAX_S);
        /* if (ego_lane == vehicle_lane && i == 0) {
          printf("id: %d, s: %f, s1: %f\n", iter->first, s, s1);
        } */
        if (ego_lane == vehicle_lane && s_dist <= 25.0) {
          printf("Predict collision, id: %d, time: %f, s0: %f, s1: %f\n",
                 iter->first, (i + 1) * TIME_STEP, s0, s1);
          return 100.0;
        }
      }
    }
    return 0.0;
  };

  auto lane_change_cost = [&](int target_lane) {
    if (target_lane == lane_) {
      return 0.0;
    }
    // const double cost = 4.0;
    const double dangerous_dist = 20.0;
    int id_behind = get_vehicle_behind(target_lane, traffics);
    int id_ahead = get_vehicle_ahead(target_lane, traffics);
    bool dangerous = false;
    if (id_behind >= 0) {
      Vehicle &vehicle = traffics[id_behind];
      double s_dist = s_distance(s_, vehicle.s_, MAX_S);
      printf("behind dist: %f\n", s_dist);
      if (s_dist <= dangerous_dist) {
        return 100.0;
      }
    }
    if (id_ahead >= 0) {
      Vehicle &vehicle = traffics[id_ahead];
      double s_dist = s_distance(s_, vehicle.s_, MAX_S);
      printf("ahead dist: %f\n", s_dist);
      if (s_dist <= dangerous_dist) {
        return 100.0;
      }
    }
    if (target_lane == (lane_ + 1) || target_lane == (lane_ - 1)) {
      return 2.5;
    } else if (target_lane == (lane_ + 2) || target_lane == (lane_ - 2)) {
      // Don't change more than one lane each time. Very dangerous.
      return 100.0;
    }
    return 0.0;
  };

  // auto speed_cost = [&](vector<vector<double>> &trajectory) {
  auto speed_limit_cost = [&](int target_lane,
                              vector<vector<double>> &trajectory) {
    int id = get_vehicle_ahead(target_lane, traffics);
    if (id >= 0) {
      Vehicle &vehicle = traffics[id];
      const double s_end = *(trajectory[0].rbegin());
      const double path_dist =
          (trajectory.empty()) ? 100.0 : s_distance(s_, s_end, MAX_S);
      const double s_dist = s_distance(vehicle.s_, s_, MAX_S);
      if (MAX_VEL >= vehicle.speed_ && s_dist < path_dist) {
        return MAX_VEL - vehicle.speed_;
      }
    }
    return 0.0;
  };

  auto change_plan_cost = [&](int target_lane) {
    if (lane_ != target_lane_ && target_lane != target_lane_) {
      return 6.0;
    }

    return 0.0;
  };

  // Compute the cost of each trajectory
  printf("\nCurr lane: %d, target_lane: %d\n", lane_, target_lane_);
  auto best_iter = trajectories.begin();
  double min_cost = std::numeric_limits<double>::infinity();
  for (auto iter = trajectories.begin(); iter != trajectories.end(); ++iter) {
    int target_lane = iter->first;
    vector<vector<double>> &trajectory = iter->second;
    printf("Check lane: %d\n", target_lane);
    if (!trajectory[0].empty() && !!trajectory[1].empty()) {
      double cost1 = collision_cost(trajectory);
      printf("collision cost: %f\n", cost1);
      double cost2 = lane_change_cost(target_lane);
      printf("change lane cost: %f\n", cost2);
      double cost3 = speed_limit_cost(target_lane, trajectory);
      printf("speed cost: %f\n", cost3);
      double cost4 = change_plan_cost(target_lane);
      printf("Change plan cost: %f\n\n", cost4);
      double total_cost = cost1 + cost2 + cost3 + cost4;
      printf("total cost: %f\n\n", total_cost);
      if (total_cost < min_cost) {
        min_cost = total_cost;
        best_iter = iter;
      }
    } else {
      printf("Lane %d is empty!\n", target_lane);
    }
  }
  vector<vector<double>> &trajectory_sd = best_iter->second;
  if (best_iter->first != lane_) {
    target_lane_ = best_iter->first;
    printf("Change to %d\n", target_lane_);
  }

  // double s0 = trajectory_sd[0][0];
  // double d0 = trajectory_sd[1][0];
  // vector<double> xy0 =
  //     getXY(s0, d0, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  // double yaw = atan2(xy0[1] - y_, xy0[0] - x_);
  /* vector<vector<double>> sample_xy;
  double cos_yaw = cos(-yaw_);
  double sin_yaw = sin(-yaw_);
  double total_dist = 0.0;
  vector<double> prev_xy = {0.0, 0.0};
  // printf("yaw: %f, cos: %f, sin: %f\n", yaw, cos_yaw, sin_yaw);
  for (int i = 0; i < trajectory_sd[0].size(); i += 30) {
    double s = trajectory_sd[0][i];
    double d = trajectory_sd[1][i];
    vector<double> xy =
        getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    double dx = xy[0] - x_;
    double dy = xy[1] - y_;
    double local_x = dx * cos_yaw - dy * sin_yaw;
    double local_y = dx * sin_yaw + dy * cos_yaw;
    total_dist += distance(local_x, local_y, prev_xy[0], prev_xy[1]);
    prev_xy[0] = local_x;
    prev_xy[1] = local_y;
    // sample_x.push_back(local_x);
    // sample_y.push_back(local_y);
    sample_xy.push_back({local_x, local_y});
    // printf("1 x: %f, y: %f\n", xy[0], xy[1]);
    // printf("1, x: %f, y: %f\n", local_x, local_y);
  }
  printf("total distance: %f\n", total_dist);
  std::sort(sample_xy.begin(), sample_xy.end(),
            [](const vector<double> &a, const vector<double> &b) {
              return a[0] < b[0];
            });
  vector<double> sample_x;
  vector<double> sample_y;
  for (vector<double> xy : sample_xy) {
    sample_x.push_back(xy[0]);
    sample_y.push_back(xy[1]);
  }

  tk::spline spline_func;
  spline_func.set_points(sample_x, sample_y);
  int prev_used = prev_trajectory_s_.size() - prev_path_size;
  double prev_s = (prev_path_size > 0) ? prev_trajectory_s_[prev_used - 1] : s_;
  double total_s = s_distance(*(trajectory_sd[0].rbegin()), prev_s, MAX_S);
  double total_x = *(sample_x.rbegin());
  // printf("total s: %f, total x: %f\n", total_s, total_x);
  cos_yaw = cos(yaw_);
  sin_yaw = sin(yaw_);

  double prev_x = 0.0;
  double prev_y = 0.0;
  for (int i = 0; i < trajectory_sd[0].size(); ++i) {
    double s_dist = s_distance(trajectory_sd[0][i], prev_s, MAX_S);
    // printf("s1: %f, s0: %f\n", trajectory_sd[0][i], prev_s);
    double dist_ratio = total_dist * s_dist / total_s;
    // printf("dist_ratio: %f, s_dist: %f, total s: %f\n", dist_ratio, s_dist,
    // total_s);
    // double min_diff = std::numeric_limits<double>::infinity();
    double local_x = prev_x + total_x * s_dist / total_s;
    double local_y = spline_func(local_x);
    double min_dist = distance(local_x, local_y, prev_x, prev_y);
    double min_dist_diff = fabs(min_dist - dist_ratio);
    for (float j = 0.1; j <= 1.0; j += 0.1) {
      double x = prev_x + dist_ratio * j;
      double y = spline_func(x);

      double dist = distance(x, y, prev_x, prev_y);
      double dist_diff = fabs(dist - dist_ratio);
      // printf("i: %d, j: %f, dist: %f, diff: %f\n", i, j, dist, dist_diff);
      if (dist_diff < min_dist_diff) {
        printf("Hello");
        min_dist_diff = dist_diff;
        min_dist = dist;
        local_x = x;
        local_y = y;
      }
    }
    printf("i: %d, dist_ratio: %f, dist: %f, diff: %f\n", i, dist_ratio,
           min_dist, dist_ratio - min_dist);
    // double local_x = prev_x + total_x * s / total_s;
    // double local_y = spline_func(local_x);
    double global_x = x_ + local_x * cos_yaw - local_y * sin_yaw;
    double global_y = y_ + local_x * sin_yaw + local_y * cos_yaw;
    next_x_vals.push_back(global_x);
    next_y_vals.push_back(global_y);
    prev_s = trajectory_sd[0][i];
    prev_x = local_x;
    prev_y = local_y;
    // printf("2 x: %f, y: %f\n", global_x, global_y);
    // printf("2 x: %f, y: %f\n", local_x, local_y);
  }
  prev_trajectory_s_ = trajectory_sd[0];
  prev_trajectory_d_ = trajectory_sd[1]; */

  int prev_used_path = prev_trajectory_s_.size() - prev_path_size;
  // printf("prev_path_size: %d, prev_used_path: %d\n", prev_path_size,
  //        prev_used_path);
  vector<double> smooth_s;
  vector<double> smooth_d;
  vector<double> prev_xy = {x_, y_};
  double prev_vel = speed_;
  double prev_acc = 0.0;
  double prev_s = s_;
  const int record_size = 10;
  list<vector<double>> record;
  bool use_smooth = true;
  // bool use_smooth = false;
  for (int i = 0; i < trajectory_sd[0].size(); ++i) {
    double s = trajectory_sd[0][i];
    double d = trajectory_sd[1][i];
    // smooth_s.push_back(s);
    // smooth_d.push_back(d);
    vector<double> xy;
    if (!use_smooth) {
      xy = getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

      // if (i <) {
      // }
      double dist = distance(xy[0], xy[1], prev_xy[0], prev_xy[1]);
      double vel = dist / TIME_STEP;
      double acc = (vel - prev_vel) / TIME_STEP;
      double s_dist = s_distance(s, prev_s, MAX_S);

      if (record.size() > record_size) {
        record.pop_front();
      }
      record.push_back({xy[0], xy[1], vel, acc});
      printf("xy: %f, %f, dis: %f, vel: %f, acc: %f\n", xy[0], xy[1], dist, vel,
             acc);
      prev_xy = xy;
      prev_s = s;
      prev_vel = vel;
      prev_acc = acc;
    } else {
      xy = getXY_smooth(s, d);
    }

    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }
  prev_trajectory_s_ = trajectory_sd[0];
  prev_trajectory_d_ = trajectory_sd[1];
}

void Vehicle::generate_splines() {
  int num_points = map_waypoints_x_.size();
  for (int i = 0; i < num_points; i++) {
    // fit spline with 6 points
    // the target point is p2
    int p0 = (i - 2 + num_points) % num_points;
    int p1 = (i - 1 + num_points) % num_points;
    int p2 = (i + num_points) % num_points;
    int p3 = (i + 1 + num_points) % num_points;
    int p4 = (i + 2 + num_points) % num_points;
    int p5 = (i + 3 + num_points) % num_points;

    vector<double> X = {map_waypoints_x_[p0], map_waypoints_x_[p1],
                        map_waypoints_x_[p2], map_waypoints_x_[p3],
                        map_waypoints_x_[p4], map_waypoints_x_[p5]};
    vector<double> Y = {map_waypoints_y_[p0], map_waypoints_y_[p1],
                        map_waypoints_y_[p2], map_waypoints_y_[p3],
                        map_waypoints_y_[p4], map_waypoints_y_[p5]};

    // affine transformation
    double x_shift = X[2];
    double y_shift = Y[2];
    double theta = atan2(Y[3] - Y[2], X[3] - X[2]);

    int num_spline_points = X.size();
    vector<double> _X(num_spline_points), _Y(num_spline_points);
    for (int i = 0; i < num_spline_points; i++) {
      // translate P0 to origin
      double x_t = X[i] - x_shift;
      double y_t = Y[i] - y_shift;
      _X[i] = x_t * cos(-theta) - y_t * sin(-theta);
      _Y[i] = x_t * sin(-theta) + y_t * cos(-theta);
    }

    tk::spline spline_func;
    spline_func.set_points(_X, _Y);
    global_splines.push_back(spline_func);
  }
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Vehicle::getXY_smooth(double s, double d) {
  assert(0.0 <= s && s <= MAX_S);
  int num_points = map_waypoints_x_.size();
  // should generate splines before getXY;
  assert(num_points == global_splines.size());

  int prev_wp = -1;

  while (s > map_waypoints_s_[prev_wp + 1] &&
         (prev_wp < (int)(map_waypoints_s_.size() - 1))) {
    prev_wp++;
  }
  // handle last waypoin
  if (prev_wp == -1) {
    prev_wp = map_waypoints_x_.size() - 1;
  }

  int wp2 = (prev_wp + 1) % map_waypoints_x_.size();

  // fit spline
  auto spline_func = global_splines[prev_wp];
  // handle next_wp == 0 (s[0] will be 0.0)
  double next_wp_s = map_waypoints_s_[wp2];
  if (next_wp_s == 0.0) {
    next_wp_s = MAX_S;
  }
  double ratio =
      (s - map_waypoints_s_[prev_wp]) / (next_wp_s - map_waypoints_s_[prev_wp]);

  // Points in car coordinates on prev_wp
  double x0 = map_waypoints_x_[prev_wp];
  double x1 = map_waypoints_x_[wp2];
  double y0 = map_waypoints_y_[prev_wp];
  double y1 = map_waypoints_y_[wp2];
  double dx = x1 - x0;
  double dy = y1 - y0;
  double theta = atan2(dy, dx);

  double _x = ratio * sqrt(dx * dx + dy * dy);
  double _y = spline_func(_x);

  double x, y;
  // revert affine transformation
  x = x0 + _x * cos(theta) - _y * sin(theta);
  y = y0 + _x * sin(theta) + _y * cos(theta);

  // add d * unit norm vector
  double nx = map_waypoints_dx_[prev_wp] +
              ratio * (map_waypoints_dx_[wp2] - map_waypoints_dx_[prev_wp]);
  double ny = map_waypoints_dy_[prev_wp] +
              ratio * (map_waypoints_dy_[wp2] - map_waypoints_dy_[prev_wp]);
  x = x + d * nx;
  y = y + d * ny;

  return {x, y};
}

std::vector<std::vector<double>> Vehicle::get_prediction() {
  // int total_step = PREDICTION_TIME / TIME_STEP;
  // vector<double> s_trajectory;
  // vector<double> d_trajectory;
  vector<vector<double>> prediction = {vector<double>(), vector<double>()};
  double x = x_;
  double y = y_;
  for (int i = 0; i < TOTAL_STEP; ++i) {
    double next_x = x + x_vel_ * TIME_STEP;
    double next_y = y + y_vel_ * TIME_STEP;
    double next_yaw = atan2(next_y - y, next_x - x);
    vector<double> next_sd =
        getFrenet(next_x, next_y, next_yaw, map_waypoints_x_, map_waypoints_y_);
    // s_trajectory.push_back(next_sd[0]);
    // d_trajectory.push_back(next_sd[1]);
    prediction[0].push_back(next_sd[0]);
    prediction[1].push_back(next_sd[1]);
    x = next_x;
    y = next_y;
  }
  // prediction.push_back(s_trajectory);
  // prediction.push_back(d_trajectory);

  /* int total_step = PREDICTION_TIME / TIME_STEP;
  double elapsed_time = TIME_STEP;
  vector<double> x_trajectory;
  vector<double> y_trajectory;
  for (int i = 0; i < TOTAL_STEP; ++i) {
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

std::vector<std::vector<double>> Vehicle::generate_trajectory(
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

  const int vehicle_ahead_id = get_vehicle_ahead(target_lane, traffics);
  const int vehicle_behind_id = get_vehicle_behind(target_lane, traffics);
  vector<double> trajectory_s;
  vector<double> trajectory_d;
  int keep_prev_size = 50;
  if (target_lane != lane_) {
    keep_prev_size = 25;
  } else if (vehicle_ahead_id >= 0) {
    Vehicle &ahead = traffics[vehicle_ahead_id];
    const double s_dist = s_distance(s_, ahead.s_, MAX_S);
    if (20.0 < s_dist && s_dist <= 30.0) {
      keep_prev_size = 30;
    } else if (10.0 < s_dist && s_dist <= 20.0) {
      keep_prev_size = 20;
    } else if (s_dist <= 10.0) {
      keep_prev_size = 10;
    }
  }
  if (prev_path_size > 0 && prev_path_size >= keep_prev_size) {
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
    double s_vel_0 = s_distance(s_0, s_1, MAX_S) / TIME_STEP;
    double s_vel_1 = s_distance(s_1, s_2, MAX_S) / TIME_STEP;
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

  // const int total_step = PREDICTION_TIME / TIME_STEP;
  end_s[0] = s_ + 100.0;
  // end_s[1] = (speed_ <= 0.1) ? 0.5 : std::min(MAX_VEL, start_s[1] * 1.2);
  end_s[1] = MAX_VEL;
  end_d[0] = (target_lane + 0.5) * LANE_WIDTH;
  if (target_lane != lane_) {
    const double speed_d = 0.5;
    const double acc_d = 0.3;
    end_d[1] = speed_d;
    // end_d[2] = acc_d;
    if (target_lane < lane_) {
      end_d[1] *= -1.0;
      // end_d[2] *= -1.0;
    }
  } else {
    end_d[1] = 0.1;
    end_d[2] = 0.0;
  }
  printf("generate lane: %d\n", target_lane);
  if (vehicle_ahead_id >= 0) {
    Vehicle &vehicle = traffics[vehicle_ahead_id];
    double predict_time = (TOTAL_STEP - keep_prev_size) * TIME_STEP;
    const double time_lv_1 = 1.5;
    const double time_lv_2 = 3.0;
    const double time_lv_3 = 4.0;
    const double buffer = 25.0;

    double vehicle_s1 =
        round_frenet_s(vehicle.s_ + vehicle.speed_ * time_lv_1, MAX_S);
    double vehicle_s2 =
        round_frenet_s(vehicle.s_ + vehicle.speed_ * time_lv_2, MAX_S);
    double vehicle_s3 =
        round_frenet_s(vehicle.s_ + vehicle.speed_ * time_lv_3, MAX_S);
    double ego_s1 = round_frenet_s(s_ + speed_ * time_lv_1 + buffer, MAX_S);
    double ego_s2 = round_frenet_s(s_ + speed_ * time_lv_2 + buffer, MAX_S);
    double ego_s3 = round_frenet_s(s_ + speed_ * time_lv_3 + buffer, MAX_S);
    printf("ego, 1: %f, 2: %f, 3: %f\n", ego_s1, ego_s2, ego_s3);
    printf("id_%d, 1: %f, 2: %f, 3: %f\n", vehicle.id_, vehicle_s1, vehicle_s2,
           vehicle_s3);
    if (!is_vehicle_ahead(ego_s1, vehicle_s1)) {
      printf("Emergency!\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_ * 0.3);
    } else if (!is_vehicle_ahead(ego_s2, vehicle_s2)) {
      printf("Too close!\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_ * 0.8);
    } else if (!is_vehicle_ahead(ego_s3, vehicle_s3)) {
      printf("Follow vehicle\n");
      end_s[1] = std::min(end_s[1], vehicle.speed_);
    }
  }
  // end_s[2] = 0.0;
  // end_d[2] = 0.0;
  // printf("s, start: %f, %f, %f\n", start_s[0], start_s[1], start_s[2]);
  // printf("s: end: %f, %f, %f\n", end_s[0], end_s[1], end_s[2]);
  // printf("d, start: %f, %f, %f\n", start_d[0], start_d[1], start_d[2]);
  // printf("d: end: %f, %f, %f\n\n", end_d[0], end_d[1], end_d[2]);
  printf("\n");

  vector<vector<double>> candidate_s_jmt_params;
  vector<vector<double>> candidate_d_jmt_params;
  for (double dt = 0.1; dt <= 20.0; dt += 0.1) {
    vector<double> s_jmt_coeffs = jmt(start_s, end_s, dt);
    if (check_s_jmt(s_jmt_coeffs, dt)) {
      candidate_s_jmt_params.push_back(s_jmt_coeffs);
    }

    vector<double> d_jmt_coeffs = jmt(start_d, end_d, dt);
    if (check_d_jmt(d_jmt_coeffs, dt)) {
      candidate_d_jmt_params.push_back(d_jmt_coeffs);
    }
  }

  if (!candidate_s_jmt_params.empty() && !candidate_d_jmt_params.empty()) {
    // Choose the best jmt_s_params
    int best_s_idx = 0;
    double min_target_s_speed_diff = std::numeric_limits<double>::infinity();
    for (int i = 0; i < candidate_s_jmt_params.size(); ++i) {
      vector<double> &s_coeffs = candidate_s_jmt_params[i];
      vector<double> s_vel_coeffs = derivative(s_coeffs);
      double t = (TOTAL_STEP - keep_prev_size) * TIME_STEP;
      double speed = poly_eval(t, s_vel_coeffs);
      // printf("expected speed: %f\n", speed);
      if (fabs(end_s[1] - speed) < min_target_s_speed_diff) {
        min_target_s_speed_diff = fabs(end_s[1] - speed);
        best_s_idx = i;
      }
    }
    vector<double> &final_s_coeffs = candidate_s_jmt_params[best_s_idx];

    // Choose the best jmt_s_params
    int best_d_idx = 0;
    double min_target_d_speed_diff = std::numeric_limits<double>::infinity();
    for (int i = 0; i < candidate_d_jmt_params.size(); ++i) {
      vector<double> &d_coeffs = candidate_d_jmt_params[i];
      vector<double> d_vel_coeffs = derivative(d_coeffs);
      double t = (TOTAL_STEP - keep_prev_size) * TIME_STEP;
      double speed = poly_eval(t, d_vel_coeffs);
      if (fabs(end_d[1] - speed) < min_target_d_speed_diff) {
        min_target_d_speed_diff = fabs(end_d[1] - speed);
        best_d_idx = i;
      }
    }
    vector<double> &final_d_coeffs = candidate_d_jmt_params[best_d_idx];

    for (int i = 0; (i + keep_prev_size) < TOTAL_STEP; ++i) {
      double t = (i + 1) * TIME_STEP;
      double s = round_frenet_s(poly_eval(t, final_s_coeffs), MAX_S);
      double d = poly_eval(t, final_d_coeffs);
      if (d < LANE_WIDTH / 2) {
        d = LANE_WIDTH / 2;
      } else if (d > (LANE_AVAILABLE - 0.5) * LANE_WIDTH) {
        d = (LANE_AVAILABLE - 0.5) * LANE_WIDTH;
      }
      // printf("t: %f, s: %f\n", t, s);
      trajectory_s.push_back(s);
      trajectory_d.push_back(d);
    }
  } else {
    printf("No available trajectory on lane :d\n", target_lane);
    cout << "Candidate s_jmt: " << candidate_s_jmt_params.size() << endl;
    cout << "Candidate d_jmt: " << candidate_d_jmt_params.size() << endl;
  }
  trajectory_s.clear();
  trajectory_d.clear();

  return {trajectory_s, trajectory_d};
}

// if s2 is in front of s1, return true, otherwise return false
bool Vehicle::is_vehicle_ahead(double s1, double s2) {
  if ((s1 <= HALF_MAX_S)) {
    if (s2 >= s1 && s2 <= s1 + HALF_MAX_S) {
      return true;
    }
  } else {
    if (s2 > s1 || s2 < s1 - HALF_MAX_S) {
      return true;
    }
  }
  return false;
}

int Vehicle::get_vehicle_behind(int target_lane,
                                const unordered_map<int, Vehicle> &traffics) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double min_s_dist = std::numeric_limits<double>::infinity();
  int id = -1;
  for (auto iter = traffics.begin(); iter != traffics.end(); ++iter) {
    const Vehicle &vehicle = iter->second;
    double s = vehicle.s_;
    double d = vehicle.d_;
    double lane = vehicle.lane_;
    if (lane == target_lane && !is_vehicle_ahead(s_, s)) {
      double s_dist = s_distance(s_, s, MAX_S);
      if (s_dist < min_s_dist) {
        min_s_dist = s_dist;
        id = vehicle.id_;
      }
    }
  }

  return id;
}

int Vehicle::get_vehicle_ahead(int target_lane,
                               const unordered_map<int, Vehicle> &traffics) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is
  //   found.
  double min_s_dist = std::numeric_limits<double>::infinity();
  int id = -1;
  for (auto iter = traffics.begin(); iter != traffics.end(); ++iter) {
    const Vehicle &vehicle = iter->second;
    double s = vehicle.s_;
    double d = vehicle.d_;
    double lane = vehicle.lane_;
    if (lane == target_lane && is_vehicle_ahead(s_, s)) {
      double s_dist = s_distance(s_, s, MAX_S);
      if (s_dist < min_s_dist) {
        min_s_dist = s_dist;
        id = vehicle.id_;
      }
    }
  }

  return id;
}

std::vector<double> Vehicle::jmt(std::vector<double> &start,
                                 std::vector<double> &end, double T) {
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
  /* vector<double> answer(6, 0.0);
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

  return answer; */

  MatrixXd A = MatrixXd(3, 3);
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

  return result;
}