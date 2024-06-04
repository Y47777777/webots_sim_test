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
    WImu(std::string inertial_init = "", std::string gyro = "",
         std::string accelerometer_ptr_ = "", std::string node = "")
        : WBase() {
        inertial_unit_ptr_ = super_->getInertialUnit("inertial unit");
        gyro_ptr_ = super_->getGyro("gyro");
        acc_ptr_ = super_->getAccelerometer("accelerometer");

        inertial_unit_ptr_->enable(5);
        gyro_ptr_->enable(5);
        acc_ptr_->enable(5);
    }

    ~WImu() {}

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
    double getVehicleYaw() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        static auto *robot_rot =
            super_->getFromDef("RobotNode_ST")->getField("rotation");

        auto tmp_r = robot_rot->getSFRotation();

        Eigen::AngleAxisd tmp_angleaxis(
            tmp_r[3], Eigen::Vector3d(tmp_r[0], tmp_r[1], tmp_r[2]));
        Eigen::Vector3d r_eulerangle3 =
            tmp_angleaxis.matrix().eulerAngles(2, 1, 0);
        vehicle_yaw_ = r_eulerangle3[0];
        return vehicle_yaw_;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        memcpy(gyro_, gyro_ptr_->getValues(), 3 * sizeof(gyro_[0]));
        memcpy(acc_, acc_ptr_->getValues(), 3 * sizeof(acc_[0]));
        memcpy(ginertial_, inertial_unit_ptr_->getRollPitchYaw(),
               3 * sizeof(ginertial_[0]));
    }

   private:
    Node *node_ = nullptr;

    InertialUnit *inertial_unit_ptr_ = nullptr;
    Gyro *gyro_ptr_ = nullptr;
    Accelerometer *acc_ptr_ = nullptr;

    double gyro_[3] = {0, 0, 0};
    double acc_[3] = {0, 0, 0};
    double ginertial_[3] = {0, 0, 0};
    double vehicle_yaw_ = {0};
};

}  // namespace VNSim