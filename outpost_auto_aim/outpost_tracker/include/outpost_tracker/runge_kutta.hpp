// Copyright 2023 Chen Tingxu
// Licence MIT

#ifndef ARMOR_TRACKER__RUNGE_KUTTA_HPP
#define ARMOR_TARCKER__RUNGE_KUTTA_HPP

#include <cmath>
#include <iostream>
#include <vector>
#include <functional>

namespace rm_auto_aim
{
class RungeKutta4
{
public:
  explicit RungeKutta4(double step);

  std::vector<double> solve(
    std::function<std::vector<double>(double, std::vector<double>)> f,
    std::vector<double> y0, double x0, double x_end,
    std::function<bool(std::vector<double>)> stop_condition = [] (std::vector<double> y) {return false;}
  );

  std::vector<std::pair<double, double>> get_track() const {return track_;}
  void clear_track() {track_.clear();}
  void set_step_size(double step) {step_size_ = step;}
  double get_step_size() const {return step_size_;}

private:
  double step_size_;

  std::vector<double> add(
    std::vector<double> a, std::vector<double> b
  );

  std::vector<double> scale(
    double scalar, std::vector<double> a
  );

  std::vector<std::pair<double, double>> track_;
};
} // namespace rm_auto_aim

#endif // ARMOR_TRACKER__RUNGE_KUTTA_HPP
