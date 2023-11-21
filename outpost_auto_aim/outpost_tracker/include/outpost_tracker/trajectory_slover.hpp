// Copyright 2023 Chen Tingxu

#ifndef OUTPOST_TRACKER__TRAJECTORY_SLOVER_HPP
#define OUTPOST_TRACKER__TRAJECTORY_SLOVER_HPP

#include "outpost_tracker/runge_kutta.hpp"

#include <iostream>
#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Dense>

namespace outpost_auto_aim
{

class TrajectorySlover
{
public:
  TrajectorySlover(
    int max_iter, double stop_error, double dt, double bullet_speed,
    double max_dist);
  double solvePitch(Eigen::Vector3d & point_world);

  inline void setBulletSpeed(const double & speed) {bullet_speed_ = speed;}
  inline double getBulletSpeed() {return bullet_speed_;}
  inline std::vector<Eigen::Vector3d> getTrajectoryWorld() {return trajectory_world_;}
  inline double getFlightTime() {return flight_time_;}

private:
  std::shared_ptr<RungeKutta4> runge_kutta_;
  std::vector<double> func(double x, std::vector<double> y);
  std::vector<double> init_state_;

  std::vector<std::pair<double, double>> trajectory_;
  std::vector<Eigen::Vector3d> trajectory_world_;

  const double g = 9.80665;
  const double k_d = 0.5 * 0.22 * 0.0445 * 6.0 / M_PI / pow(0.042, 3) * M_PI * pow(0.042, 2) / 4.0;
  const double k_l = 0.0;

  int max_iter_{0};
  double stop_error_{0};
  double dt_{0};
  double bullet_speed_{0};
  double max_dist_{0};

  double flight_time_{0};
};

}  // namespace outpost_auto_aim

#endif  //OUTPOST_TRACKER__TRAJECTORY_SLOVER_HPP
