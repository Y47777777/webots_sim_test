#pragma once

#include "geometry/geometry.h"

namespace VNSim {
typedef struct WLidarInfo {
    int hor_res = 0;          // horizontal_resolution 水平分辨率
    int ver_res = 0;          // vertical_resolution   垂直分辨率
    double hor_fov = 2 * PI;  // 水平 fov
    double ver_fov = PI;      // 垂直 fov
};

}  // namespace VNSim