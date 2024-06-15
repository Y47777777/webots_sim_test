#pragma once

#include "lidar_simulation/lidar_info.h"
#include "geometry/geometry.h"
#include <webots/Lidar.hpp>

namespace VNSim {
struct Spot {
    // 默认是全空间均为最大
    double hor_angle_start = 0;
    double hor_angle_end = 2 * PI;
    double ver_angle_start = 0;
    double ver_angle_end = PI;
};

class BlindSpot {
   public:
    BlindSpot(const WLidarInfo *info) { info_ = info; }
    ~BlindSpot();

    void push_back() { };

    bool check(){return false;}

   private:
    const WLidarInfo *info_ = nullptr;
    std::vector<Spot> v_sopt;
};

}  // namespace VNSim