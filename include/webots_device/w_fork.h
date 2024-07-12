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
          std::string sensor_name = "", std::string brake_name = "",
          bool isNeedShadowMove = false)
        : WBase() {
        // creat fork
        isShadowMove_ = isNeedShadowMove;
        motor_ = super_->getMotor(fork_motor_name);
        if (motor_ != nullptr) {
            motor_->setPosition(INFINITY);
            motor_->setVelocity(0);

            LOG_INFO("creat fork: %s", fork_motor_name.c_str());
        }

        // creat node
        Node *node = super_->getFromDef(solid_name);
        if (node != nullptr) {
            translation_ptr_ = node->getField("translation");
            rotation_ptr_ = node->getField("rotation");

            // TODO: 如果需要添加再修改
            if (isNeedShadowMove) {
                translation_ptr_->setSFVec3f(shadowPos);
            }
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
     * @brief Set Shadow Position
     *
     * @param[in] y Y pos
     */
    void setShadowPos(double y) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        shadowPos[1] = y;
        translation_ptr_->setSFVec3f(shadowPos);
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
            if (brake_ != nullptr) {
                if (fabs(speed_) < 0.01) {
                    brake_->setDampingConstant(MAXFLOAT);
                } else {
                    brake_->setDampingConstant(0);
                }
            }

            motor_->setVelocity(speed_);
        }
    };

   private:
    Brake *brake_ = nullptr;
    Motor *motor_ = nullptr;
    PositionSensor *position_sensor_ = nullptr;
    Field *rotation_ptr_ = nullptr;
    Field *translation_ptr_ = nullptr;

    std::vector<double> init_pose_ = {0, 0, 0};
    bool isShadowMove_ = false;
    double shadowPos[3] = {0, 0, 0};
    double pos_sensor_value_ = 0;
    double speed_ = 0;
};

}  // namespace VNSim
