#include "armor_tracker/runge_kutta.hpp"

namespace rm_auto_aim
{
RungeKutta4::RungeKutta4(double step)
{
  step_size_ = step;
}

std::vector<double> RungeKutta4::solve(
  std::function<std::vector<double>(double, std::vector<double>)> f,
  std::vector<double> y0, double x0, double x_end,
  std::function<bool(std::vector<double>)> stop_condition
)
{
  track_.clear();
  std::vector<double> y = y0;
  for (double x = x0; x < x_end; x += step_size_) {
    std::vector<double> k1 = scale(step_size_, f(x, y));
    std::vector<double> k2 = scale(step_size_, f(x + 0.5 * step_size_, add(y, scale(0.5, k1))));
    std::vector<double> k3 = scale(step_size_, f(x + 0.5 * step_size_, add(y, scale(0.5, k2))));
    std::vector<double> k4 = scale(step_size_, f(x + step_size_, add(y, k3)));

    y = add(y, scale(1.0 / 6.0, add(k1, scale(2.0, add(k2, add(k3, k4))))));
    track_.push_back(std::make_pair(y[0], y[1]));

    if (stop_condition(y)) {
      break;
    }
  }
  return y;
}

std::vector<double> RungeKutta4::add(std::vector<double> a, std::vector<double> b)
{
  std::vector<double> result;
  for (int i = 0; i < a.size(); i++) {
    result.push_back(a[i] + b[i]);
  }
  return result;
}

std::vector<double> RungeKutta4::scale(double scalar, std::vector<double> a)
{
  std::vector<double> result;
  for (int i = 0; i < a.size(); i++) {
    result.push_back(scalar * a[i]);
  }
  return result;
}

} // namespace rm_auto_aim
