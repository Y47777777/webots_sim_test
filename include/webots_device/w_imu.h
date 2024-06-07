/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 11:19:05
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

    // TODO: why not return Vector3d?
    double getInertialValue(int index) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return ginertial_[index];
    }
    double getGyroValue(int index) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return gyro_[index];
    }
    double getAccValue(int index) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return acc_[index];
    }

    /**
     * @brief Get the * value
     *
     * @param[out] result 需要拷贝的数据
     */
    void getInertialValue(std::vector<double> &result) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        getData(result, ginertial_);
    }
    void getGyroValue(std::vector<double> &result) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        getData(result, gyro_);
    }
    void getAccValue(std::vector<double> &result) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        getData(result, acc_);
    }

    // TODO: move to wheel
    double getVehicleYaw() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        static auto *robot_rot =
            super_->getFromDef("RobotNode_ST")->getField("rotation");

        auto tmp_r = robot_rot->getSFRotation();

        Eigen::AngleAxisd tmp_angleaxis(
            tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));
        Eigen::Vector3d r_eulerangle3 =
            tmp_angleaxis.matrix().eulerAngles(2, 1, 0);
        return r_eulerangle3[0];
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        memcpy(gyro_.data(), gyro_ptr_->getValues(), 3 * sizeof(gyro_[0]));
        memcpy(acc_.data(), acc_ptr_->getValues(), 3 * sizeof(acc_[0]));
        memcpy(ginertial_.data(), inertial_unit_ptr_->getRollPitchYaw(),
               3 * sizeof(ginertial_[0]));
    }

   private:
    void getData(std::vector<double> &result,
                 const std::vector<double> &source) {
        result.resize(source.size());
        memcpy(result.data(), source.data(), source.size() * sizeof(source[0]));
    }

   private:
    Node *node_ = nullptr;

    InertialUnit *inertial_unit_ptr_ = nullptr;
    Gyro *gyro_ptr_ = nullptr;
    Accelerometer *acc_ptr_ = nullptr;

    std::vector<double> gyro_ = {std::vector<double>(0, 3)};
    std::vector<double> acc_ = {std::vector<double>(0, 3)};
    std::vector<double> ginertial_ = {std::vector<double>(0, 3)};
};

}  // namespace VNSim