#pragma once

#include "geometry/geometry.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "foxglove-vn/Pose.pb.h"
#include "logvn/logvn.h"

namespace VNSim {
typedef struct Reflector {
    Reflector(Eigen::Vector4d cen, std::vector<Eigen::Vector4d> p_list,
              std::vector<Eigen::Vector4d> a_list, std::vector<double> min_list,
              std::vector<double> max_list) {
        center = cen;
        point_list = p_list;
        axis_list = a_list;
        min_dot_prduct_list = min_list;
        max_dot_prduct_list = max_list;

        // 获取z范围加速
        for (auto &point : point_list) {
            if (min_z > point.z()) {
                min_z = point.z();
            }
            if (max_z < point.z()) {
                max_z = point.z();
            }
        }
    }

    std::vector<Eigen::Vector4d> point_list;
    std::vector<Eigen::Vector4d> axis_list;
    std::vector<double> min_dot_prduct_list;
    std::vector<double> max_dot_prduct_list;

    // 用这三个
    Eigen::Vector4d center;
    double min_z = MAXFLOAT;
    double max_z = -MAXFLOAT;
};

class ReflectorChecker {
   public:
    static std::shared_ptr<ReflectorChecker> getInstance() {
        if (instance_ptr_ == nullptr) {
            instance_ptr_ = std::make_shared<ReflectorChecker>();
        }
        return instance_ptr_;
    }
    ReflectorChecker() {}
    ~ReflectorChecker() {}

    void addReflectorChecker(Reflector &reflector) {
        reflector_list_.push_back(reflector);
    }

    // TODO: 改为从protobuf 中拷贝
    void copyFrom(std::vector<Reflector> list) {
        reflector_list_ = list;
        LOG_INFO("reflector size: %d", reflector_list_.size());
    }

    void setSensorPose(std::string name, foxglove::Pose pose) {
        Eigen::Quaterniond rotation(
            pose.orientation().x(), pose.orientation().y(),
            pose.orientation().z(), pose.orientation().w());

        Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                    pose.position().z());

        m_sensor_matrix_.insert(
            std::pair(name, poseToMatrix4d(rotation, translation)));
    }

    void setSensorMatrix4d(std::string name, Eigen::Matrix4d matrix) {
        m_sensor_matrix_.insert(std::pair(name, matrix));
        LOG_INFO("matrix : %s", name.c_str());

        LOG_INFO("%.2f, %.2f, %.2f, %.2f", matrix(0, 0), matrix(0, 1),
                 matrix(0, 2), matrix(0, 3));
        LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(1, 0), matrix(1, 1),
                 matrix(1, 2), matrix(1, 3));
        LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(2, 0), matrix(2, 1),
                 matrix(2, 2), matrix(2, 3));
        LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(3, 0), matrix(3, 1),
                 matrix(3, 2), matrix(3, 3));
    }

    void setCurPose(foxglove::Pose pose) {
        Eigen::Quaterniond rotation(
            pose.orientation().x(), pose.orientation().y(),
            pose.orientation().z(), pose.orientation().w());

        Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                    pose.position().z());

        cur_pose_matrix_ = poseToMatrix4d(rotation, translation);
    }

    void setCurPose(Eigen::Matrix4d matrix) {
        cur_pose_matrix_ = matrix;

        // LOG_INFO("matrix : pose");
        // LOG_INFO("%.2f, %.2f, %.2f, %.2f", matrix(0, 0), matrix(0, 1),
        //          matrix(0, 2), matrix(0, 3));
        // LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(1, 0), matrix(1, 1),
        //          matrix(1, 2), matrix(1, 3));
        // LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(2, 0), matrix(2, 1),
        //          matrix(2, 2), matrix(2, 3));
        // LOG_INFO("%.2f, % .2f, % .2f, % .2f", matrix(3, 0), matrix(3, 1),
        //          matrix(3, 2), matrix(3, 3));
    }

    bool checkInReflector(std::string name,
                          const sim_data_flow::LidarPoint *p) {
        if (m_sensor_matrix_.find(name) == m_sensor_matrix_.end()) {
            return false;
        }
        if (reflector_list_.empty()) {
            return false;
        }
        // 变换至世界
        Eigen::Vector4d point(p->x(), p->y(), p->z(), 1);
        point = m_sensor_matrix_[name] * point;
        point = cur_pose_matrix_ * point;

        for (const auto &reflector : reflector_list_) {
            // 先通过 z 排除
            if (point.z() < reflector.min_z)
                continue;
            if (point.z() > reflector.max_z)
                continue;

            // 通过中心位置排除 > 1m
            if (fabs(point.x() - reflector.center.x()) > 1)
                continue;
            if (fabs(point.y() - reflector.center.y()) > 1)
                continue;

            if (isPonitInCube(point, reflector)) {
                return true;
            }
        }

        return false;
    }

   private:
    bool isPonitInCube(const Eigen::Vector4d &point,
                       const Reflector &reflector) {
        for (int i = 0; i < reflector.axis_list.size(); i++) {
            double dotP = dotProduct(point, reflector.axis_list[i]);
            if (dotP < reflector.min_dot_prduct_list[i] ||
                dotP > reflector.max_dot_prduct_list[i]) {
                return false;
            }
        }
        return true;
    }

    std::vector<Reflector> reflector_list_;

    // TODO: 激光帧需要与里程计对齐，目前只使用最新接受的，这不对
    Eigen::Matrix4d cur_pose_matrix_;
    std::map<std::string, Eigen::Matrix4d> m_sensor_matrix_;  // 外参
    static std::shared_ptr<ReflectorChecker> instance_ptr_;
};

}  // namespace VNSim

// 计算交叉乘积
// bool isPointInCube(const Eigen::Vector4d &p,
//                    const std::vector<Eigen::Vector4d> &cubeVertices) {
//     static bool first = true;
//     // 对于每个轴（立方体的每个边）
//     for (int i = 0; i < 12; i++) {
//         Eigen::Vector4d axis;
//         // 计算轴（立方体的边）
//         if (i < 4) {
//             axis = cubeVertices[i + 1] - cubeVertices[i];
//         } else if (i < 8) {
//             axis = cubeVertices[i - 3] - cubeVertices[i - 4];
//         } else {
//             axis = cubeVertices[i - 8] - cubeVertices[i - 7];
//         }

//         // 计算点和立方体在轴上的投影
//         double dotP = dotProduct(p, axis);
//         double minDotCube = dotProduct(cubeVertices[0], axis);
//         double maxDotCube = minDotCube;
//         for (int j = 1; j < 8; j++) {
//             double dotCube = dotProduct(cubeVertices[j], axis);
//             minDotCube = std::min(minDotCube, dotCube);
//             maxDotCube = std::max(maxDotCube, dotCube);
//         }
//         if (first) {
//             LOG_INFO("min : %.2f", minDotCube);
//             LOG_INFO("max : %.2f", maxDotCube);
//         }
//         // 如果点的投影不在立方体的投影范围内，那么这个点就不在立方体内
//         if (dotP < minDotCube || dotP > maxDotCube) {
//             return false;
//         }
//     }
//     first = false;

//     // 点在所有轴上的投影都在立方体的投影范围内，所以这个点在立方体内
//     return true;
// }

// double crossProduct(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
//         return a.x() * b.y() - a.y() * b.x();
//     }
// return true;
// 检查是否在当前包围盒中
// Eigen::Vector2d point_xy(point.x(), point.y());
// Eigen::Vector2d a(reflector.point_list[0].x(),
//                   reflector.point_list[0].y());
// Eigen::Vector2d b(reflector.point_list[1].x(),
//                   reflector.point_list[1].y());
// Eigen::Vector2d c(reflector.point_list[2].x(),
//                   reflector.point_list[2].y());
// Eigen::Vector2d d(reflector.point_list[3].x(),
//                   reflector.point_list[3].y());

// return isPointInBox(point_xy, a, b, c, d);
// // }

//   double crossProduct(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
//         return a.x() * b.y() - a.y() * b.x();
//     }

//     // 判断点是否在矩形内
//     bool isPointInBox(const Eigen::Vector2d &p, const Eigen::Vector2d &a,
//                       const Eigen::Vector2d &b, const Eigen::Vector2d &c,
//                       const Eigen::Vector2d &d) {
//         Eigen::Vector2d ap = p - a;
//         Eigen::Vector2d bp = p - b;
//         Eigen::Vector2d cp = p - c;
//         Eigen::Vector2d dp = p - d;

//         double ab = crossProduct(ap, bp);
//         double bc = crossProduct(bp, cp);
//         double cd = crossProduct(cp, dp);
//         double da = crossProduct(dp, ap);

//         return (ab >= 0 && bc >= 0 && cd >= 0 && da >= 0) ||
//                (ab <= 0 && bc <= 0 && cd <= 0 && da <= 0);
//     }