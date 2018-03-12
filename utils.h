#ifndef UTILS_H
#define UTILS_H

inline const float pi = 3.14159265358979323846;

template <typename T>
auto LinSpace(T a, T b, size_t n) {
  auto h = (b - a) / static_cast<T>(n - 1);
  return [ val = a, h ]() mutable {
    val += h;
    return val;
  };
}

#endif  // UTILS_H
