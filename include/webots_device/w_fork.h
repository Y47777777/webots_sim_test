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

#define FORK_DELAY 10

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
        v_delay_speed_.resize(FORK_DELAY, 0);
        std::fill(v_delay_speed_.begin(), v_delay_speed_.end(), 0);
        sub_speed_iter_ = 0;
        set_speed_iter_ = 1;
    }

    /**
     * @brief Set the Velocity object
     *
     * @param[in] v  Velocity
     */
    void setVelocityAll(double v) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        std::fill(v_delay_speed_.begin(), v_delay_speed_.end(), v);
        sub_speed_iter_ = 0;
        set_speed_iter_ = 1;
    }

    /**
     * @brief Set the Velocity object
     *
     * @param[in] v  Velocity
     */
    void setVelocity(double v) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        v_delay_speed_[sub_speed_iter_] = v;
        sub_speed_iter_++;
        if (sub_speed_iter_ >= FORK_DELAY) {
            sub_speed_iter_ = 0;
        }
        if (fabs(v) > 1.0) {
            LOG_ERROR("v_delay_speed_1 %.2f, %.2f, %.2f, %.2f, %.2f",
                      v_delay_speed_[0], v_delay_speed_[1], v_delay_speed_[2],
                      v_delay_speed_[3], v_delay_speed_[4]);
            LOG_ERROR("v_delay_speed_2 %.2f, %.2f, %.2f, %.2f, %.2f",
                      v_delay_speed_[5], v_delay_speed_[6], v_delay_speed_[7],
                      v_delay_speed_[8], v_delay_speed_[9]);

            LOG_ERROR("sub_speed_iter_: %d,set_speed_iter_: %d",
                      sub_speed_iter_, set_speed_iter_);
            LOG_ERROR("spin v %f", v);
        }
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
        return v_delay_speed_[set_speed_iter_];
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        set_speed_iter_ = sub_speed_iter_ + 1;
        if (set_speed_iter_ >= FORK_DELAY) {
            set_speed_iter_ = 0;
        }
        double speed = v_delay_speed_[set_speed_iter_];

        if (fabs(speed) > 1.0) {
            LOG_ERROR("v_delay_speed_1 %.2f, %.2f, %.2f, %.2f, %.2f",
                      v_delay_speed_[0], v_delay_speed_[1], v_delay_speed_[2],
                      v_delay_speed_[3], v_delay_speed_[4]);
            LOG_ERROR("v_delay_speed_2 %.2f, %.2f, %.2f, %.2f, %.2f",
                      v_delay_speed_[5], v_delay_speed_[6], v_delay_speed_[7],
                      v_delay_speed_[8], v_delay_speed_[9]);

            LOG_ERROR("sub_speed_iter_: %d,set_speed_iter_: %d",
                      sub_speed_iter_, set_speed_iter_);
            LOG_ERROR("spin v %f", speed);
        }

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
            if ((fabs(speed) < 0.001)) {
                motor_->setPosition(last_pos_);
                motor_->setVelocity(0.01);
                if (brake_ != nullptr) {
                    brake_->setDampingConstant(1000);
                }
            } else if ((speed > 0) && (bound == 0)) {
                // high bound
                motor_->setPosition(high_bound_);
                motor_->setVelocity(0.01);
                if (brake_ != nullptr) {
                    brake_->setDampingConstant(1000);
                }
            } else if ((speed < 0) && (bound == 1)) {
                // high bound
                motor_->setPosition(low_bound_);
                motor_->setVelocity(0.01);
                if (brake_ != nullptr) {
                    brake_->setDampingConstant(1000);
                }
            } else {
                if (brake_ != nullptr) {
                    brake_->setDampingConstant(0);
                }
                motor_->setPosition(INFINITY);
                motor_->setVelocity(speed);
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

    bool isShadowMove_ = false;
    double shadowPos[3] = {0, 0, 0};

    std::vector<double> v_delay_speed_;

    double sub_speed_iter_ = 0;
    double set_speed_iter_ = 0;

    double min_pos_ = 0;
    double pos_sensor_value_ = 0;
    double last_pos_ = 0.0;
    // double speed_ = 0;
    double high_bound_ = 0;
    double low_bound_ = 0;
};

}  // namespace VNSim
