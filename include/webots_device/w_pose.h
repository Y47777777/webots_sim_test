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
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return createTransformMatrix(tf_rotation_, tf_translation_);
    }

    double *getTransfer() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return tf_translation_;
    }

    double *getRotaion() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return tf_rotation_;
    }

    Node *getRobotNode() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return node_;
    }

    void setTransferWithTime(double *transfer, double *rotation, uint64_t time_stamp) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        had_recive_tran_ = true;

        set_tf_translation_[0] = transfer[0];
        set_tf_translation_[1] = transfer[1];
        set_tf_translation_[2] = transfer[2];

        set_tf_rotation_[0] = rotation[0];
        set_tf_rotation_[1] = rotation[1];
        set_tf_rotation_[2] = rotation[2];
        set_tf_rotation_[3] = rotation[3];
        t_stamp_befoe_set_ = time_stamp;
    }

    void setTransfer(double *transfer) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        had_recive_tran_ = true;
        
        set_tf_translation_[0] = transfer[0];
        set_tf_translation_[1] = transfer[1];
        set_tf_translation_[2] = transfer[2];
    }

    uint64_t getTimeStamp() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return last_time_stamp_;
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

        ReflectorChecker::getInstance()->setCurPose(
            createTransformMatrix(tf_rotation_, tf_translation_));

        if (had_recive_tran_) {
            had_recive_tran_ = false;
            translation_ptr_->setSFVec3f(set_tf_translation_);
            rotation_ptr_->setSFRotation(set_tf_rotation_);

            last_time_stamp_ = t_stamp_cur_;
            t_stamp_cur_ = t_stamp_befoe_set_;
        }
    }

   private:
    Node *node_ = nullptr;
    std::string node_name_ = "";
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    double tf_translation_[3] = {0, 0, 0};
    double tf_rotation_[4] = {0, 0, 1, 0};

    bool had_recive_tran_ = false;

    double set_tf_translation_[3] = {0, 0, 0};
    double set_tf_rotation_[4] = {0, 0, 1, 0};
    uint64_t AGVController = 0;

    uint64_t t_stamp_befoe_set_ = 0;  // 收到未设入webots 的webots
    uint64_t t_stamp_cur_ = 0;
    uint64_t last_time_stamp_ = 0;
};

}  // namespace VNSim
