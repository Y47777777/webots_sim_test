/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 15:15:00
 * @FilePath: /webots_ctrl/include/webots_device/w_imu.h
 * @Description:   webots imu 接口
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Node.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>

#include "webots_device/w_base.h"
#include "foxglove-vn/Imu.pb.h"
#include "foxglove-vn/Vector3.pb.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;
class WImu : public WBase {
   public:
    /**
     * @brief Construct a new WImu object
     *
     * @param[in] inertial_init     惯性模块名称
     * @param[in] gyro              陀螺仪
     * @param[in] accelerometer     加速度计
     * @param[in] node              惯性模块solid(tf)
     */
    WImu(std::string inertial_init = "", std::string gyro = "",
         std::string accelerometer = "", std::string node = "")
        : WBase() {
        inertial_unit_ptr_ = super_->getInertialUnit(inertial_init);
        gyro_ptr_ = super_->getGyro(gyro);
        acc_ptr_ = super_->getAccelerometer(accelerometer);

        inertial_unit_ptr_->enable(step_duration_);
        gyro_ptr_->enable(step_duration_);
        acc_ptr_->enable(step_duration_);
    }

    ~WImu() {}

    foxglove::Vector3 getGyroValue() {
        AutoAtomicLock lock(spin_mutex_);
        foxglove::Vector3 result;
        getVector3(result, gyro_);

        return result;
    }

    foxglove::Vector3 getAccValue() {
        AutoAtomicLock lock(spin_mutex_);
        foxglove::Vector3 result;
        getVector3(result, acc_);

        return result;
    }

    // foxglove::Quaternion getInertialValue() {
    //     AutoAtomicLock lock(spin_mutex_);
    //     foxglove::Quaternion result;
    //     getRollPitchYaw(result);
    //     return result;
    // }

    double getInertialYaw() { return angles_[2]; }

    foxglove::Vector3 getInertialRollPitchYaw(){
        AutoAtomicLock lock(spin_mutex_);
        foxglove::Vector3 result;
        getVector3(result, angles_);
        return result;
    }

    void spin() {
        memcpy(gyro_.data(), gyro_ptr_->getValues(), 3 * sizeof(gyro_[0]));
        memcpy(acc_.data(), acc_ptr_->getValues(), 3 * sizeof(acc_[0]));
        memcpy(angles_.data(), inertial_unit_ptr_->getRollPitchYaw(), 3 * sizeof(acc_[0]));
    }

   private:
    void getVector3(foxglove::Vector3 &result,
                    const std::vector<double> &source) {
        result.set_x(source[0]);
        result.set_y(source[1]);
        result.set_z(source[2]);
    }

   private:
    Node *node_ = nullptr;

    InertialUnit *inertial_unit_ptr_ = nullptr;
    Gyro *gyro_ptr_ = nullptr;
    Accelerometer *acc_ptr_ = nullptr;

    std::vector<double> gyro_ = std::vector<double>(3);
    std::vector<double> acc_ = std::vector<double>(3);
    // std::vector<double> quaternion_ = std::vector<double>(4);
    std::vector<double> angles_ = std::vector<double>(3);  // Roll, Pitch, Yaw
};

}  // namespace VNSim