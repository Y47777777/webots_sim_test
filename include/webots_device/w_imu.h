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

    void getImuValue() {}

    void spin() {
        memcpy(gyro_, gyro_ptr_->getValues(), 4 * sizeof(gyro_[0]));
        memcpy(acc_, acc_ptr_->getValues(), 4 * sizeof(acc_[0]));
        // memcpy(ginertial_, inertial_unit_ptr_->getValues(),
        //        4 * sizeof(ginertial_[0]));
    }

   private:
    Node *node_ = nullptr;

    InertialUnit *inertial_unit_ptr_ = nullptr;
    Gyro *gyro_ptr_ = nullptr;
    Accelerometer *acc_ptr_ = nullptr;

    double gyro_[3] = {0, 0, 0};
    double acc_[3] = {0, 0, 0};
    double ginertial_[3] = {0, 0, 0};
};

}  // namespace VNSim