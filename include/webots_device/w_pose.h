/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 10:45:19
 * @FilePath:
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Node.hpp>
#include <shared_mutex>

#include "geometry/geometry.h"
#include "webots_device/w_base.h"
#include "logvn/logvn.h"
#include "lidar_simulation/high_reflector.h"

namespace VNSim {
using namespace webots;

class WPose : public WBase {
   public:
    /**
     * @brief Construct a new WPose object
     *
     * @param[in] pose_motor_name   webots 下 pose名字
     * @param[in] solid_name        需要旋转 或 控制旋转轴 填入webots solid 名称
     * @param[in] sensor_name       positionSensor 如果有motor 可输入为空
     */
    WPose(std::string pose_node_name_ = "") : WBase() {
        node_ = super_->getFromDef(pose_node_name_);
        if (node_ == nullptr) {
            LOG_ERROR("%s ,is nullptr", pose_node_name_.c_str());
        }

        node_name_ = pose_node_name_;
        translation_ptr_ = node_->getField("translation");
        rotation_ptr_ = node_->getField("rotation");
    }

    Eigen::Matrix4d getTransferMatrix() {
        return createTransformMatrix(tf_rotation_, tf_translation_);
    }

    Eigen::Vector3d getTransfer() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Vector3d result;
        result[0] = tf_rotation_[0];
        result[1] = tf_rotation_[1];
        result[2] = tf_rotation_[2];
        return result;
    }

    // Eigen::AngleAxisd getAngleAxisd() {
    //     std::shared_lock<std::shared_mutex> lock(rw_mutex_);
    //     return rotation_;
    // }

    // TODO: 这段要改
    double getVehicleYaw() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        // static auto *robot_rot =
        //     super_->getFromDef("RobotNode_ST")->getField("rotation");

        // auto tmp_r = robot_rot->getSFRotation();

        // Eigen::AngleAxisd tmp_angleaxis(
        //     tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));

        Eigen::AngleAxisd rotation = {
            tf_rotation_[3],
            Eigen::Vector3d(tf_rotation_[0], tf_rotation_[1], tf_rotation_[2])};

        Eigen::Vector3d r_eulerangle3 = rotation.matrix().eulerAngles(2, 1, 0);
        return r_eulerangle3[0];
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        const double *translation_address = translation_ptr_->getSFVec3f();
        const double *rotation_address = rotation_ptr_->getSFRotation();
        tf_translation_[0] = translation_address[0];
        tf_translation_[1] = translation_address[1];
        tf_translation_[2] = translation_address[2];

        tf_rotation_[0] = rotation_address[0];
        tf_rotation_[1] = rotation_address[1];
        tf_rotation_[2] = rotation_address[2];
        tf_rotation_[3] = rotation_address[3];

        ReflectorChecker::getInstance()->setCurPose(this->getTransferMatrix());
    }

   private:
    Node *node_ = nullptr;
    std::string node_name_ = "";
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 1, 0};
};

}  // namespace VNSim
