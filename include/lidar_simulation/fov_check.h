#pragma once

#include "geometry/geometry.h"
#include <webots/Lidar.hpp>

namespace VNSim {

typedef struct FovInfo {
    double horizon_start = 0;  // 水平fov 起始角  以x轴为起点，弧度制
    double horizon_end = 0;     // 水平fov 结束角  同上
    double vertical_start = 0;  // 垂直fov
    double vertical_end = 0;    //
};

class Fov {
   public:
    Fov(FovInfo input) {
        info_ = input;
        if (fabs(info_.horizon_start - info_.horizon_end) > 0.1) {
            horizon_enable_ = true;
        }
        if (fabs(info_.vertical_start - info_.vertical_end) > 0.1) {
            vertical_enable_ = true;
            k_for_start_ = tan(info_.vertical_start);
            k_for_end_ = tan(info_.vertical_end);
        }
    };

    ~Fov() {
        WLidarInfo info = info_in;
        start_layer_ = 0;
        end_layer_ = size_of_layer_;

        // 水平方向
        if (!info.hor_bilnd_spot.empty()) {
            bilnd_spot_enable_ = true;
            double reslution = lidar_->getHorizontalResolution() / (2 * PI);

            for (auto iter = info.hor_bilnd_spot.begin();
                 iter != info.hor_bilnd_spot.end(); iter++) {
                if (iter->begin * iter->end < 0) {
                    info.hor_bilnd_spot.push_back({iter->end, 2 * PI});
                    info.hor_bilnd_spot.push_back({0, iter->begin});
                    // iter = info.hor_bilnd_spot.erase(iter);
                    continue;
                }
                if (iter->begin < 0) {
                    iter->begin += 2 * PI;
                }
                if (iter->end < 0) {
                    iter->end += 2 * PI;
                }

                BilndSpot tmp = {
                    .begin_index = std::round(reslution * iter->begin),
                    .end_index = std::round(reslution * iter->end)};
                bilnd_spot_.push_back(tmp);

                // iter++;
            }
        }

        if (!info.ver_bilnd_spot.empty()) {
            fov_vertical_enable_ = true;
            double reslution = size_of_layer_ / lidar_->getVerticalFov();
            for (auto iter = info.ver_bilnd_spot.begin();
                 iter != info.ver_bilnd_spot.end(); iter++) {
                iter->begin += PI / 2;
                iter->end += PI / 2;
                if (iter->begin < 0 || iter->begin > PI || iter->end < 0 ||
                    iter->end > PI) {
                    continue;
                }

                

            }
        }

        if (fabs(info.vertical_begin - info.vertical_end) > 0.1) {
            fov_vertical_enable_ = true;

            // 计算起始层
            double vertical_fov = lidar_->getVerticalFov();
            double resolution = size_of_layer_ / vertical_fov;
            double start_angle = PI / 2 - info.vertical_end;
            start_layer_ = std::round(start_angle * resolution);

            // 计算终点
            double end_angle = PI / 2 - info.vertical_begin;
            end_layer_ = std::round(end_angle * resolution);

            if (end_layer_ < 0) {
                end_layer_ = 0;
                LOG_ERROR("end_layer error :%d", end_layer_);
            }
            if (start_layer_ > size_of_layer_) {
                start_layer_ = 0;
                LOG_INFO("start_layer_:%d", start_layer_);
            }
        }
    }

    bool pointInFov(const WbLidarPoint &point) {
        return (checkVectorFov(point));
    }
    // && checkHorzonFov(point)
   private:
    bool checkWithAtan(const WbLidarPoint &point) {
        // double angle = atan(point.z / point.x);
        // return (angle > info_.vertical_start && angle < info_.vertical_end);
        double a = point.x, b = point.y, c = point.z;

        double cos_theta = c / sqrt(a * a + b * b + c * c);
        double theta = PI / 2 - acos(cos_theta);

        return (theta > info_.vertical_start && theta < info_.vertical_end);
    }

    bool checkVectorFov(const WbLidarPoint &point) {
        if (!vertical_enable_) {
            return true;
        }
        // k_for_start_ * k_for_end_ >= 0 &&
        if (checkPositionOfLine(point, k_for_start_) == 1 &&
            checkPositionOfLine(point, k_for_end_) == -1) {
            // 位于两根fov直线中间
            return true;
        }
        return false;
    }

    // else if (k_for_start_ * k_for_start_ < 0 &&
    //                checkPositionOfLine(point, k_for_start_) == 1 &&
    //                checkPositionOfLine(point, k_for_end_) == 1) {
    //         // 位于两根fov直线中间
    //         return true;
    //     }

    bool checkHorzonFov(const WbLidarPoint &point) {
        if (!horizon_enable_) {
            return true;
        }

        double angle = atan(point.y / point.x);

        return (angle > info_.horizon_start && angle < info_.horizon_end);
    }

    /**
     * @brief 检查点与fov直线的关系
     *
     * @return int = 1   在直线上方
     * @return int = -1  在直线下方
     */
    int checkPositionOfLine(const WbLidarPoint &point, const double &k) {
        double radius_line = (k == 0) ? MAXFLOAT : (std::pow(point.z / k, 2));
        double radius_point = std::pow(point.x, 2) + std::pow(point.y, 2);

        int result = radius_point < radius_line ? 1 : -1;
      

        return result;
    }

    FovInfo info_;

    bool horizon_enable_ = false;
    bool vertical_enable_ = false;

    double k_for_start_ = 0;  // 垂直fov对应的直线斜率
    double k_for_end_ = 0;    // 同上
};

}  // namespace VNSim