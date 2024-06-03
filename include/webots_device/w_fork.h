#pragma once

#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <shared_mutex>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;

class WFork : public WBase {
   public:
    /*
     * @brief  构建 fork对象
     *
     * @param[fork_name]  webots 下 fork名字
     * @param[solid_name]   需要旋转 或 控制旋转轴 填入webots solid 名称
     * @param[sensor_name] positionSensor
     */
    WFork(std::string fork_motor_name = "", std::string solid_name = "",
          std::string sensor_name = "")
        : WBase() {
        // creat fork
        motor_ = super_->getMotor(fork_motor_name);
        if (motor_ != nullptr) {
            motor_->setPosition(INFINITY);
            motor_->setVelocity(0);

            // LOG_INFO("creat fork: %s", fork_motor_name.c_str());
        }

        // creat node
        Node *node = super_->getFromDef(solid_name);
        if (node != nullptr) {
            webots::Field *translation_ptr_ = node->getField("translation");
            webots::Field *rotation_ptr_ = node->getField("rotation");

            // TODO: 如果需要添加再修改
        }

        // creat pose sensor
        {
            if (sensor_name == "" && motor_ != nullptr) {
                position_sensor_ = motor_->getPositionSensor();
            } else {
                position_sensor_ = super_->getPositionSensor(sensor_name);
            }

            if (position_sensor_ != nullptr) {
                position_sensor_->enable(5);
                // LOG_INFO("creat fork:%s  fork sensor :%s",
                //          fork_motor_name.c_str(), sensor_name.c_str());
            }
        }
    }

    ~WFork(){};

    void setVelocity(double v) { speed_ = v; }

    double getSenosorValue() { return pos_sensor_value_; }
    double getVelocityValue() { return speed_; }

    void forkSpin() {
        // get fork pos value
        if (position_sensor_ != nullptr) {
            pos_sensor_value_ = position_sensor_->getValue();
        }

        // set fork rotate center
        if (rotation_ptr_ != nullptr) {
            // TODO: set fork rotate center

            // TODO: set fork yaw
        }

        if (motor_ != nullptr) {
            motor_->setVelocity(speed_);
        }
    }

    void spin() {
        // get wheel pos value
        if (position_sensor_ != nullptr) {
            pos_sensor_value_ = position_sensor_->getValue();
        }

        // set speed
        if (motor_ != nullptr) {
            motor_->setVelocity(speed_);
        }
    };

   private:
    Motor *motor_ = nullptr;
    PositionSensor *position_sensor_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    std::vector<double> init_pose_ = {0, 0, 0};

    double pos_sensor_value_ = 0;
    double speed_ = 0;
};

}  // namespace VNSim
