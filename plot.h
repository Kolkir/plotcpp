#ifndef PLOT_CPP_
#define PLOT_CPP_

#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <type_traits>

namespace plotcpp {

enum class PlottingType { Points, Lines };

template <typename I, PlottingType PT> struct PlottingItem {
  using value_type = typename I::value_type;
  PlottingItem(I startX, I endX, I startY, std::string name)
      : startX(startX), endX(endX), startY(startY), plotType(PT), name(name) {}
  constexpr const char *GetTypeStr() const {
    if constexpr (PT == PlottingType::Lines)
      return "lines";
    else if constexpr (PT == PlottingType::Points)
      return "points";
    else
      static_assert(std::is_same<I, void>::value,
                    "Unsuported ploting item type");
  }
  I startX;
  I endX;
  I startY;
  PlottingType plotType;
  std::string name;
};

template <typename I> auto Lines(I startX, I endX, I startY, std::string name) {
  return PlottingItem<I, PlottingType::Lines>(startX, endX, startY, name);
}

template <typename I>
auto Points(I startX, I endX, I startY, std::string name) {
  return PlottingItem<I, PlottingType::Points>(startX, endX, startY, name);
}

class Plot {
public:
  Plot() {
    pipe_ = popen("gnuplot", "w");
    if (pipe_ == nullptr)
      throw std::runtime_error("Can't open pipe to gnuplot");
  }
  ~Plot() {
    auto err = pclose(pipe_);
    if (err == -1)
      std::cerr << "Error closing gnuplot pipe \n";
  }
  Plot(const Plot &) = delete;
  Plot &operator=(const Plot &) = delete;

  void SetTerminal(const std::string &term) { write("set terminal", term); }
  void SetOutput(const std::string &filename) { write("set out", filename); }
  void SetTitle(const std::string &title) { write("set title", title); }
  void SetXLabel(const std::string &label) { write("set xlabel", label); }
  void SetYLabel(const std::string &label) { write("set ylabel", label); }
  void EnableMultiplot() { write("set multiplot", ""); }
  void DisableMultiplot() { write("unset multiplot", ""); }
  void SetAutoscale() { write("set autoscale", ""); }
  void Flush() {
    if (fflush(pipe_) < 0)
      std::cerr << "Failed to flush gnuplot pipe \n";
  }

  template <typename... Args> void Draw2D(Args... items) {
    std::string cmd{"plot "};
    MakePlotParams(cmd, items...);
    write(cmd, "");
    DrawBinaries(items...);
  }

private:
  template <typename T> void MakePlotParams(std::string &base, const T &item) {
    base += MakePlotParam(item);
  }

  template <typename T, typename... Args>
  void MakePlotParams(std::string &base, const T &item, Args... items) {
    base += MakePlotParam(item);
    base += ",";
    MakePlotParams(base, items...);
  }

  template <typename T> std::string MakePlotParam(const T &item) const {
    auto n = std::distance(item.startX, item.endX);
    std::string cmd{R"("-" binary format=")"};
    cmd += GetFormat<typename T::value_type>();
    cmd += GetFormat<typename T::value_type>();
    cmd += R"(" record=)";
    cmd += std::to_string(n);
    cmd += " with ";
    cmd += item.GetTypeStr();
    cmd += R"( title ")";
    cmd += item.name;
    cmd += R"(")";
    return cmd;
  }

  template <typename T> constexpr const char *GetFormat() const {
    if constexpr (std::is_same<int, T>::value)
      return "%int32";
    else if constexpr (std::is_same<float, T>::value)
      return "%float32";
    else
      static_assert(std::is_same<T, void>::value, "Unsuported ploting type");
    return nullptr;
  }

  void DrawBinaries() {}

  template <typename T, typename... Args>
  void DrawBinaries(const T &item, Args... items) {
    DrawBinary(item.startX, item.endX, item.startY);
    DrawBinaries(items...);
  }

  template <typename I> void DrawBinary(I startX, I endX, I startY) {
    typename I::value_type data[2];
    while (startX != endX) {
      data[0] = *startX;
      data[1] = *startY;
      if (fwrite(&data[0], sizeof(typename I::value_type), 2, pipe_) != 2) {
        std::cerr << "Failed to write to gnuplot pipe \n";
      }
      ++startX;
      ++startY;
    }
  }

  void write(const std::string &cmd, const std::string &param) {
    const char *fmt = "%s\n";
    if (!param.empty()) {
      fmt = "%s \"%s\"\n";
    }
    if (fprintf(pipe_, fmt, cmd.c_str(), param.c_str()) < 0) {
      std::cerr << "Failed to write to gnuplot pipe \n";
    }
  }

private:
  FILE *pipe_;
};
} // namespace plotcpp

#endif // PLOT_CPP_
