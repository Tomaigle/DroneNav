#pragma once

#include <chrono>

namespace drone_vehicle {

inline std::chrono::steady_clock::time_point deadline_after(double seconds) {
  return std::chrono::steady_clock::now() +
         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(seconds));
}
inline double age_s(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t)
      .count();
}
} // namespace drone_vehicle
