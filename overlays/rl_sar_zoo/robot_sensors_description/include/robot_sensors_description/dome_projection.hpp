// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot_sensors_description
{
constexpr double kPi = 3.14159265358979323846;

struct DomeSample
{
  std::array<double, 3> direction;
  std::size_t face;
  std::size_t pixel;
  double depth_to_range;
};

// Camera frames are Gazebo frames (+X forward, +Y left, +Z up), not optical frames.
// Faces point along +X, -X, +Y, -Y, +Z in the physical dome frame.
inline DomeSample ProjectDirection(const std::array<double, 3> & d, unsigned resolution)
{
  if (resolution < 2 || d[2] < -1e-12) {
    throw std::invalid_argument("Expected a +Z hemisphere direction and resolution >= 2");
  }
  std::size_t face;
  double forward, left, up;
  if (d[2] >= std::max(std::abs(d[0]), std::abs(d[1]))) {
    face = 4; forward = d[2]; left = d[1]; up = -d[0];
  } else if (std::abs(d[0]) >= std::abs(d[1])) {
    face = d[0] >= 0 ? 0 : 1;
    forward = std::abs(d[0]); left = face == 0 ? d[1] : -d[1]; up = d[2];
  } else {
    face = d[1] >= 0 ? 2 : 3;
    forward = std::abs(d[1]); left = face == 2 ? -d[0] : d[0]; up = d[2];
  }
  if (!(forward > 0)) {
    throw std::invalid_argument("Direction must be nonzero");
  }
  // Nearest pixel centre. Clamping handles exact cube-face boundaries.
  const auto pixel = [resolution](double coordinate) {
      return static_cast<unsigned>(std::clamp(
        std::floor((0.5 - 0.5 * coordinate) * resolution),
        0.0, static_cast<double>(resolution - 1)));
    };
  return {d, face, pixel(up / forward) * resolution + pixel(left / forward), 1.0 / forward};
}

inline std::vector<DomeSample> MakeDomeGrid(unsigned horizontal, unsigned vertical, unsigned resolution)
{
  if (horizontal < 2 || vertical < 2 || resolution < 2) {
    throw std::invalid_argument("Dome sample counts and GPU resolution must be >= 2");
  }
  std::vector<DomeSample> samples;
  samples.reserve(static_cast<std::size_t>(horizontal) * vertical);
  for (unsigned row = 0; row < vertical; ++row) {
    const double elevation = (kPi / 2) * row / (vertical - 1);
    for (unsigned col = 0; col < horizontal; ++col) {
      const double azimuth = -kPi + 2 * kPi * col / horizontal;
      samples.push_back(ProjectDirection({std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth), std::sin(elevation)}, resolution));
    }
  }
  return samples;
}
}  // namespace robot_sensors_description
