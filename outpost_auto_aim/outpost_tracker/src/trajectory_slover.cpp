// Copyright 2023 Chen Tingxu

#include "outpost_tracker/trajectory_slover.hpp"
#include "outpost_tracker/runge_kutta.hpp"

#include "rclcpp/rclcpp.hpp"

namespace outpost_auto_aim
{
TrajectorySlover::TrajectorySlover(
  int max_iter, double stop_error, double dt, double bullet_speed,
  double max_dist)
: max_iter_(max_iter), stop_error_(stop_error), dt_(dt), bullet_speed_(bullet_speed), max_dist_(
    max_dist)
{
  runge_kutta_ = std::make_shared<RungeKutta4>(dt_);
}

std::vector<double> TrajectorySlover::func(double x, std::vector<double> y)
{
  (void) x;
  std::vector<double> result(4);
  result[0] = y[2] * cos(y[3]);
  result[1] = y[2] * sin(y[3]);
  result[2] = -g * sin(y[3]) - k_d * y[2] * y[2];
  result[3] = -g * cos(y[3]) / y[2] + k_l * y[2];
  return result;
}

double TrajectorySlover::solvePitch(Eigen::Vector3d & point_world)
{
  // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "Solving pitch...");
  // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "point_world: %f, %f, %f", point_world[0], point_world[1], point_world[2]);

  trajectory_.clear();
  trajectory_world_.clear();
  flight_time_ = 0.0;

  auto h = point_world[2];
  auto vertical_tmp = h;
  auto theta = atan2(point_world[1], point_world[0]);
  auto horizonal_dist = sqrt(point_world.squaredNorm() - h * h); // squaredNorm()表示向量的模的平方
  auto pitch = atan(h / horizonal_dist) * 180.0 / M_PI;
  auto pitch_new = pitch;

  init_state_.clear();
  init_state_.resize(4);
  init_state_[0] = 0.0;
  init_state_[1] = 0.0;
  init_state_[2] = bullet_speed_;
  init_state_[3] = pitch_new * M_PI / 180.0;

  for (int i = 0; i < max_iter_; i++) {
    // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "Iteration %d", i);
    trajectory_.clear();
    trajectory_world_.clear();
    init_state_[3] = pitch_new * M_PI / 180.0;
    // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "init_state: %f, %f, %f, %f", init_state_[0], init_state_[1], init_state_[2], init_state_[3]);

    auto final_state = runge_kutta_->solve(
      std::bind(&TrajectorySlover::func, this, std::placeholders::_1, std::placeholders::_2),
      init_state_, 0.0, max_dist_ * 2.0 / bullet_speed_,
      [horizonal_dist](std::vector<double> y) {
        return y[0] > horizonal_dist;
      }
    );
    trajectory_ = runge_kutta_->get_track();

    double x_1 = trajectory_[trajectory_.size() - 2].first;
    double y_1 = trajectory_[trajectory_.size() - 2].second;
    double x_2 = trajectory_[trajectory_.size() - 1].first;
    double y_2 = trajectory_[trajectory_.size() - 1].second;
    double k = (y_2 - y_1) / (x_2 - x_1);
    double y = k * (horizonal_dist - x_1) + y_1;


    auto error = h - y;
    // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "h: %f, y: %f, error: %f", h, y, error);

    if (abs(error) <= stop_error_) {
      // RCLCPP_INFO(rclcpp::get_logger("outpost_tracker"), "Final state: %f, %f, %f, %f", final_state[0], final_state[1], final_state[2], final_state[3]);

      for (auto & point : trajectory_) {
        trajectory_world_.emplace_back(
          Eigen::Vector3d(
            point.first * cos(theta),
            point.first * sin(theta), point.second));
      }
      flight_time_ = (trajectory_.size() - 2) * dt_ + (horizonal_dist - x_1) / (x_2 - x_1) * dt_;
      break;
    } else {
      vertical_tmp += error;
      pitch_new = atan(vertical_tmp / horizonal_dist) * 180.0 / M_PI;
    }
  }
  return pitch_new - pitch;
}

}  // namespace outpost_auto_aim
