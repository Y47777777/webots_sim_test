/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 10:45:19
 * @FilePath: /webots_ctrl/include/webots_device/w_fork.h
 * @Description: webots 车叉接口
 *               由 pose + motor + possensor 组成
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Motor.hpp>
#include <webots/Brake.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <shared_mutex>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

namespace VNSim {
using namespace webots;

class WFork : public WBase {
   public:
    /**
     * @brief Construct a new WFork object
     *
     * @param[in] fork_motor_name   webots 下 fork名字
     * @param[in] solid_name        需要旋转 或 控制旋转轴 填入webots solid 名称
     * @param[in] sensor_name       positionSensor 如果有motor 可输入为空
     * @param[in] break_name        break 如果有motor 可输入为空
     */
    WFork(std::string fork_motor_name = "", std::string solid_name = "",
          std::string sensor_name = "", std::string brake_name = "")
        : WBase() {
        // creat fork
        motor_ = super_->getMotor(fork_motor_name);
        if (motor_ != nullptr) {
            // user should set model +- 0.1 for (minStop, minStart, minPosition,
            // maxPosition)
            high_bound_ = motor_->getMaxPosition() - 0.1;
            low_bound_ = motor_->getMinPosition() + 0.1;
            motor_->setPosition(INFINITY);
            motor_->setVelocity(0);
            LOG_INFO("creat fork: %s", fork_motor_name.c_str());
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
                LOG_INFO("creat fork:%s  fork sensor :%s",
                         fork_motor_name.c_str(), sensor_name.c_str());
            }
        }

        // creat brake
        if (brake_name == "" && motor_ != nullptr) {
            brake_ = motor_->getBrake();
        } else {
            brake_ = super_->getBrake(brake_name);
        }

        if (brake_ != nullptr) {
            LOG_INFO("creat fork brake:%s  fork brake :%s",
                     fork_motor_name.c_str(), brake_name.c_str());
        }
    }

    /**
     * @brief Set the Velocity object
     *
     * @param[in] v  Velocity
     */
    void setVelocity(double v) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        speed_ = v;
    }

    /**
     * @brief Get the Senosor Value object
     *
     * @return double sensor value
     */
    double getSenosorValue() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return pos_sensor_value_;
    }

    /**
     * @brief Get the Velocity Value object
     *
     * @return double
     */
    // TODO: is this necessary?
    double getVelocityValue() {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        return speed_;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        // get wheel pos value
        if (position_sensor_ != nullptr) {
            pos_sensor_value_ = position_sensor_->getValue();
        }

        // set speed
        if (motor_ != nullptr) {
            int bound = -1;
            // 0.0002 is a value to allow forks stop just around bounding value
            if (pos_sensor_value_ > (high_bound_ + 0.0002)) {
                bound = 0;
            }
            if (pos_sensor_value_ < (low_bound_ - 0.0002)) {
                bound = 1;
            }
            if ((fabs(speed_) < 0.001)) {
                motor_->setPosition(last_pos_);
                motor_->setVelocity(0.01);
                brake_->setDampingConstant(1000);
            } else if ((speed_ > 0.001) && (bound == 0)) {
                // high bound
                motor_->setPosition(high_bound_);
                motor_->setVelocity(0.01);
                brake_->setDampingConstant(1000);
            } else if ((speed_ < -0.001) && (bound == 1)) {
                // high bound
                motor_->setPosition(low_bound_);
                motor_->setVelocity(0.01);
                brake_->setDampingConstant(1000);
            } else {
                brake_->setDampingConstant(0);
                motor_->setPosition(INFINITY);
                motor_->setVelocity(speed_);
                last_pos_ = pos_sensor_value_;
            }
        }
    }

   private:
    Brake *brake_ = nullptr;
    Motor *motor_ = nullptr;
    PositionSensor *position_sensor_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;
    std::vector<double> init_pose_ = {0, 0, 0};
    double min_pos_ = 0;
    double pos_sensor_value_ = 0;
    double last_pos_ = 0.0;
    double speed_ = 0;
    double high_bound_ = 0;
    double low_bound_ = 0;
};  // namespace VNSim

}  // namespace VNSim
