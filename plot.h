/*
Copyright (c) 2018, Kirill Kolodiazhnyi (rotate@ukr.net)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PLOT_CPP_
#define PLOT_CPP_

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace plotcpp {

enum class PlottingType { Points, Lines };

template <typename I, PlottingType PT>
struct PlottingItem {
  using value_type = typename I::value_type;
  PlottingItem(I startX, I endX, I startY, std::string name)
      : startX(startX), endX(endX), startY(startY), plotType(PT), name(name) {}
  constexpr const char* GetTypeStr() const {
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

template <typename I>
auto Lines(I startX, I endX, I startY, std::string name) {
  return PlottingItem<I, PlottingType::Lines>(startX, endX, startY, name);
}

template <typename I>
auto Points(I startX, I endX, I startY, std::string name) {
  return PlottingItem<I, PlottingType::Points>(startX, endX, startY, name);
}

class Plot {
 public:
  Plot(bool persist = false) {
    if (persist)
      pipe_ = popen("gnuplot -p", "w");
    else
      pipe_ = popen("gnuplot", "w");
    if (pipe_ == nullptr)
      throw std::runtime_error("Can't open pipe to gnuplot");
  }
  ~Plot() {
    auto err = pclose(pipe_);
    if (err == -1)
      std::cerr << "Error closing gnuplot pipe \n";
  }
  Plot(const Plot&) = delete;
  Plot& operator=(const Plot&) = delete;

  void SetTerminal(const std::string& term) { write("set terminal", term); }
  void SetOutput(const std::string& filename) { write("set out", filename); }
  void SetTitle(const std::string& title) { write("set title", title); }
  void SetXLabel(const std::string& label) { write("set xlabel", label); }
  void SetYLabel(const std::string& label) { write("set ylabel", label); }
  void SetAutoscale() { write("set autoscale", ""); }
  void GnuplotCommand(const std::string& cmd) { write(cmd, ""); }

  void SetXRange(double min, double max) {
    std::stringstream range_str;
    range_str << "set xrange [" << min << ":" << max << "]";
    write(range_str.str(), "");
  }

  void SetYRange(double min, double max) {
    std::stringstream range_str;
    range_str << "set yrange [" << min << ":" << max << "]";
    write(range_str.str(), "");
  }

  void Flush() {
    if (fflush(pipe_) < 0)
      std::cerr << "Failed to flush gnuplot pipe \n";
  }

  template <typename... Args>
  void Draw2D(Args... items) {
    std::string cmd{"plot "};
    MakePlotParams(cmd, items...);
    write(cmd, "");
    DrawBinaries(items...);
  }

 private:
  template <typename T>
  void MakePlotParams(std::string& base, const T& item) {
    base += MakePlotParam(item);
  }

  template <typename T, typename... Args>
  void MakePlotParams(std::string& base, const T& item, Args... items) {
    base += MakePlotParam(item);
    base += ",";
    MakePlotParams(base, items...);
  }

  template <typename T>
  std::string MakePlotParam(const T& item) const {
    auto n = std::distance(item.startX, item.endX);
    std::stringstream cmd;
    cmd << R"("-" binary format=")";
    cmd << GetFormat<typename T::value_type>();
    cmd << GetFormat<typename T::value_type>();
    cmd << R"(" record=)";
    cmd << n;
    cmd << " with ";
    cmd << item.GetTypeStr();
    cmd << R"( title ")";
    cmd << item.name;
    cmd << R"(")";
    return cmd.str();
  }

  template <typename T>
  constexpr const char* GetFormat() const {
    if constexpr (std::is_same<std::int8_t, T>::value)
      return "%int8";
    else if constexpr (std::is_same<std::uint8_t, T>::value)
      return "%uint8";
    else if constexpr (std::is_same<std::int16_t, T>::value)
      return "%int16";
    else if constexpr (std::is_same<std::uint16_t, T>::value)
      return "%uint16";
    else if constexpr (std::is_same<std::int32_t, T>::value)
      return "%int32";
    else if constexpr (std::is_same<std::uint32_t, T>::value)
      return "%uint32";
    else if constexpr (std::is_same<std::int64_t, T>::value)
      return "%int64";
    else if constexpr (std::is_same<std::uint64_t, T>::value)
      return "%uint64";
    else if constexpr (std::is_same<float, T>::value)
      return "%float";
    else if constexpr (std::is_same<double, T>::value)
      return "%double";
    else
      static_assert(std::is_same<T, void>::value, "Unsuported ploting type");
    return nullptr;
  }

  void DrawBinaries() {}

  template <typename T, typename... Args>
  void DrawBinaries(const T& item, Args... items) {
    DrawBinary(item.startX, item.endX, item.startY);
    DrawBinaries(items...);
  }

  template <typename I>
  void DrawBinary(I startX, I endX, I startY) {
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

  void write(const std::string& cmd, const std::string& param) {
    const char* fmt = "%s\n";
    if (!param.empty()) {
      fmt = "%s \"%s\"\n";
    }
    if (fprintf(pipe_, fmt, cmd.c_str(), param.c_str()) < 0) {
      std::cerr << "Failed to write to gnuplot pipe \n";
    }
  }

 private:
  FILE* pipe_;
};
}  // namespace plotcpp

#endif  // PLOT_CPP_
