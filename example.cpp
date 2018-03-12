#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "plot.h"
#include "utils.h"

const bool persist_gnuplot_window = true;
const size_t n = 50;

void PlotAutoscale(const std::vector<float>& y, const std::vector<float>& x) {
  using namespace plotcpp;

  Plot plt(persist_gnuplot_window);
  plt.SetTerminal("qt");
  plt.SetTitle("Sine");
  plt.SetXLabel("X");
  plt.SetYLabel("Y");
  plt.SetAutoscale();

  plt.Draw2D(Lines(x.begin(), x.end(), y.begin(), "lines"),
             Points(x.begin(), x.end(), y.begin(), "points"));

  plt.Flush();
}

void PlotXYRanges(const std::vector<float>& y, const std::vector<float>& x) {
  using namespace plotcpp;

  Plot plt(persist_gnuplot_window);
  plt.SetTerminal("qt");
  plt.SetTitle("Sine");
  plt.SetXLabel("X");
  plt.SetYLabel("Y");
  plt.SetXRange(-pi / 2, pi / 2);
  plt.SetYRange(-0.5, 0.5);

  plt.Draw2D(Lines(x.begin(), x.end(), y.begin(), "lines"),
             Points(x.begin(), x.end(), y.begin(), "points"));

  plt.Flush();
}

void PlotToFile(const std::vector<float>& y, const std::vector<float>& x) {
  using namespace plotcpp;

  Plot plt;
  plt.SetTerminal("png");
  plt.SetOutput("test.png");
  plt.SetTitle("Sine");
  plt.SetXLabel("X");
  plt.SetYLabel("Y");
  plt.SetAutoscale();

  plt.Draw2D(Lines(x.begin(), x.end(), y.begin(), "lines"),
             Points(x.begin(), x.end(), y.begin(), "points"));

  plt.Flush();
  std::cout << "Plot saved to test.png\n";
}

void MultiPlot(const std::vector<float>& x) {
  using namespace plotcpp;

  std::vector<float> y_sin(n);
  std::transform(x.begin(), x.end(), y_sin.begin(),
                 [](auto v) { return std::sin(v); });
  std::vector<float> y_cos(n);
  std::transform(x.begin(), x.end(), y_cos.begin(),
                 [](auto v) { return std::cos(v); });
  std::vector<float> y_tan(n);
  std::transform(x.begin(), x.end(), y_tan.begin(),
                 [](auto v) { return std::tan(v); });
  std::vector<float> y_tanh(n);
  std::transform(x.begin(), x.end(), y_tanh.begin(),
                 [](auto v) { return std::tanh(v); });

  Plot plt(persist_gnuplot_window);
  plt.SetTerminal("qt");
  plt.SetXLabel("x");
  plt.SetYLabel("y");
  plt.SetAutoscale();

  plt.GnuplotCommand("set multiplot layout 2,2 rowsfirst");
  plt.Draw2D(Lines(x.begin(), x.end(), y_sin.cbegin(), "sin"));
  plt.Draw2D(Lines(x.begin(), x.end(), y_cos.cbegin(), "cos"));
  plt.Draw2D(Lines(x.begin(), x.end(), y_tan.cbegin(), "tan"));
  plt.Draw2D(Lines(x.begin(), x.end(), y_tanh.cbegin(), "tanh"));
  plt.GnuplotCommand("unset multiplot");
  plt.Flush();
}

void ShowHelp(const char* prog_path) {
  std::cout << prog_path
            << " [n]\n"
               "where n:\n"
               ""
               "0 - Simple Autoscale plot\n"
               "1 - Simple plot with x,y ranges\n"
               "2 - Plot to file\n"
               "3 - Multiplot with custom gnuplot commands\n";
}

int main(int argc, char* argv[]) {
  using namespace std;

  vector<float> x(n);
  generate(x.begin(), x.end(), LinSpace(-pi, pi, n));
  vector<float> y(n);
  std::transform(x.begin(), x.end(), y.begin(),
                 [](auto v) { return std::sin(v); });

  if (argc != 2) {
    ShowHelp(argv[0]);
  } else {
    auto ex = argv[1][0] - '0';
    switch (ex) {
      case 0:
        PlotAutoscale(y, x);
        break;
      case 1:
        PlotXYRanges(y, x);
        break;
      case 2:
        PlotToFile(y, x);
        break;
      case 3:
        MultiPlot(x);
        break;
      default:
        ShowHelp(argv[0]);
    }
  }
  return 0;
}
