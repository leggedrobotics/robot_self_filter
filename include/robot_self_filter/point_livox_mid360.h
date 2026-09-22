#pragma once

#include <cstdint>
#include <pcl/point_types.h>

// Matches the PointXYZRTLT fields published by the Mid-360 driver and Newton.
struct EIGEN_ALIGN16 PointLivoxMid360 {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointLivoxMid360,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint8_t, tag, tag)
                                  (std::uint8_t, line, line)
                                  (double, timestamp, timestamp)
)
