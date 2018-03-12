#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "plot.h"

template <typename T> auto LinSpace(T a, T b, size_t n) {
  auto h = (b - a) / static_cast<T>(n - 1);
  return [ val = a, h ]() mutable {
    val += h;
    return val;
  };
}

int main() {
  using plotcpp::Lines;
  using plotcpp::Plot;
  using plotcpp::Points;
  using std::generate;
  using std::transform;
  using std::vector;

  const size_t n = 50;
  const float pi = M_PI;

  vector<float> x(n);
  generate(x.begin(), x.end(), LinSpace(-pi, pi, n));
  vector<float> y(n);
  std::transform(x.begin(), x.end(), y.begin(),
                 [](auto v) { return std::sin(v); });

  Plot plt;
  plt.SetTerminal("qt");
  plt.SetTitle("Sine");
  plt.SetXLabel("X");
  plt.SetYLabel("Y");
  plt.SetAutoscale();

  plt.Draw2D(Lines(x.begin(), x.end(), y.begin(), "lines"),
             Points(x.begin(), x.end(), y.begin(), "points"));

  plt.Flush();

  return 0;
}
