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
        translation_address_ = translation_ptr_->getSFVec3f();
        rotation_address_ = rotation_ptr_->getSFRotation();
    }

    Eigen::Matrix4d getTransferMatrix() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        // TODO:change to geometry
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.rotate(rotation_);
        transform.translation() = translation_.head<3>();

        return transform.matrix();
    }

    Eigen::Vector3d getTransfer() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        Eigen::Vector3d result;
        result[0] = translation_[0];
        result[1] = translation_[1];
        result[2] = translation_[2];
        return result;
    }

    Eigen::AngleAxisd getAngleAxisd() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return rotation_;
    }

    // TODO: 这段要改
    double getVehicleYaw() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        // static auto *robot_rot =
        //     super_->getFromDef("RobotNode_ST")->getField("rotation");

        // auto tmp_r = robot_rot->getSFRotation();

        // Eigen::AngleAxisd tmp_angleaxis(
        //     tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));
        Eigen::Vector3d r_eulerangle3 = rotation_.matrix().eulerAngles(2, 1, 0);
        return r_eulerangle3[0];
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        translation_[0] = translation_address_[0];
        translation_[1] = translation_address_[1];
        translation_[2] = translation_address_[2];

        rotation_.axis()[0] = rotation_address_[0];
        rotation_.axis()[1] = rotation_address_[1];
        rotation_.axis()[2] = rotation_address_[2];
        rotation_.angle() = rotation_address_[3];
    }

   private:
    Node *node_ = nullptr;
    std::string node_name_ = "";
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    const double *translation_address_ = nullptr;
    const double *rotation_address_ = nullptr;

    Eigen::Vector4d translation_ = {0, 0, 0, 1};
    Eigen::AngleAxisd rotation_ = {0, Eigen::Vector3d(0, 1, 0)};
};

}  // namespace VNSim
