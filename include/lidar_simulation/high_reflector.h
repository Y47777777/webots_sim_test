#pragma once

#include "geometry/geometry.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "foxglove-vn/Pose.pb.h"
#include "logvn/logvn.h"

namespace VNSim {
typedef struct Reflector {
    Reflector(Eigen::Vector4d cen, std::vector<Eigen::Vector4d> list) {
        center = cen;
        point_list = list;

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
    void copyFrom(std::vector<Reflector> list) { reflector_list_ = list; }

    void setSensorMatrix4d(std::string name, foxglove::Pose pose) {
        Eigen::Quaterniond rotation(
            pose.orientation().x(), pose.orientation().y(),
            pose.orientation().z(), pose.orientation().w());

        Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                    pose.position().z());

        m_sensor_matrix_.insert(
            std::pair(name, poseToMatrix4d(rotation, translation)));
    }

    void setCurPose(foxglove::Pose pose) {
        Eigen::Quaterniond rotation(
            pose.orientation().x(), pose.orientation().y(),
            pose.orientation().z(), pose.orientation().w());

        Eigen::Vector3d translation(pose.position().x(), pose.position().y(),
                                    pose.position().z());

        cur_pose_matrix_ = poseToMatrix4d(rotation, translation);
    }

    bool checkInReflector(std::string name,
                          const sim_data_flow::LidarPoint *p) {
        if (m_sensor_matrix_.find(name) == m_sensor_matrix_.end()) {
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

            // 通过中心位置排除 >3m
            if (fabs(point.x() - reflector.center.x()) > 2)
                continue;
            if (fabs(point.y() - reflector.center.y()) > 2)
                continue;

            // return true;
            // 检查是否在当前包围盒中
            Eigen::Vector2d point_xy(point.x(), point.y());
            Eigen::Vector2d a(reflector.point_list[0].x(),
                              reflector.point_list[0].y());
            Eigen::Vector2d b(reflector.point_list[1].x(),
                              reflector.point_list[1].y());
            Eigen::Vector2d c(reflector.point_list[2].x(),
                              reflector.point_list[2].y());
            Eigen::Vector2d d(reflector.point_list[3].x(),
                              reflector.point_list[3].y());

            return isPointInBox(point_xy, a, b, c, d);
        }

        return false;
    }

   private:
    // 计算交叉乘积
    double crossProduct(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
        return a.x() * b.y() - a.y() * b.x();
    }

    // 判断点是否在矩形内
    bool isPointInBox(const Eigen::Vector2d &p, const Eigen::Vector2d &a,
                      const Eigen::Vector2d &b, const Eigen::Vector2d &c,
                      const Eigen::Vector2d &d) {
        Eigen::Vector2d ap = p - a;
        Eigen::Vector2d bp = p - b;
        Eigen::Vector2d cp = p - c;
        Eigen::Vector2d dp = p - d;

        double ab = crossProduct(ap, bp);
        double bc = crossProduct(bp, cp);
        double cd = crossProduct(cp, dp);
        double da = crossProduct(dp, ap);

        return (ab >= 0 && bc >= 0 && cd >= 0 && da >= 0) ||
               (ab <= 0 && bc <= 0 && cd <= 0 && da <= 0);
    }

    std::vector<Reflector> reflector_list_;

    // TODO: 激光帧需要与里程计对齐，目前只使用最新接受的，这不对
    Eigen::Matrix4d cur_pose_matrix_;
    std::map<std::string, Eigen::Matrix4d> m_sensor_matrix_;  // 外参
    static std::shared_ptr<ReflectorChecker> instance_ptr_;
};
}  // namespace VNSim