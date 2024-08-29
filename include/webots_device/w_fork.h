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

#include "geometry/geometry.h"

#define FORK_DELAY 10
#define TOTAL_MAX_FORCE_UNITS_NUMBER 3

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
     * @param[in] slower_move       每次移动时随机减少一定值
     */
    WFork(std::string fork_motor_name = "", std::string solid_name = "",
          std::string sensor_name = "", std::string brake_name = "",
          bool isNeedShadowMove = false, double slower_move = 0.0,
          bool isNeedReadForce = false, double force_sample_frequency = 100, bool isNeedAddExtraPoints = false, double unit_factor = TOTAL_MAX_FORCE_UNITS_NUMBER)
        : WBase() {
        // creat fork
        isShadowMove_ = isNeedShadowMove;
        isNeedExtraForce_ = isNeedAddExtraPoints;
        isNeedReadForce_ = isNeedReadForce;
        solid_name_ = solid_name;
        motor_ = super_->getMotor(fork_motor_name);
        if (motor_ != nullptr) {
            // user should set model +- 0.1 for (minStop, minStart, minPosition,
            // maxPosition)
            high_bound_ = motor_->getMaxPosition() - 0.1;
            low_bound_ = motor_->getMinPosition() + 0.1;
            motor_->setPosition(INFINITY);
            motor_->setVelocity(0);
            if (isNeedReadForce) {
                motor_->enableForceFeedback(force_sample_frequency);
                unit_force_ = motor_->getMaxForce() / unit_factor;
                LOG_INFO("fork unit force = %f", unit_force_);
            }
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
                super_->step(step_duration_);
                pos_sensor_value_ = position_sensor_->getValue();
                last_pos_ = pos_sensor_value_;
            }
        }

        // creat brake
        {
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

        if (fabs(slower_move) > 0.00001) {
            random_generator_ = std::make_shared<RandomGenerator>(slower_move);
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
        if (random_generator_ != nullptr) {
            if (fabs(v) > 0.001) {
                double random = random_generator_->generate();
                if (v > 0) {
                    v = v - random;
                    if(v < 0){
                        v = v + random;
                    }
                } else {
                    v = v + random;
                    if(v > 0){
                        v = v - random;
                    }
                }
            }
        }
        std::fill(v_delay_speed_.begin(), v_delay_speed_.end(), v);
        sub_speed_iter_ = 0;
        set_speed_iter_ = 1;
    }

    int isOnBoundary(){
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        if ((std::fabs(pos_sensor_value_ - high_bound_) < 0.002) || (pos_sensor_value_ > high_bound_)) {
            return -1;
        }
        if (std::fabs(pos_sensor_value_ - low_bound_) < 0.002 || (pos_sensor_value_ < low_bound_)) {
            return -2;
        }
        return 0;
    }

    void forceReset(bool reset){
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);
        forceReset_ = reset;
    }

    /**
     * @brief Set the Velocity object
     *
     * @param[in] v  Velocity
     */
    void setVelocity(double v) {
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);

        if (random_generator_ != nullptr) {
            if (fabs(v) > 0.001) {
                if (v > 0) {
                    v = v - random_generator_->generate();
                } else {
                    v = v + random_generator_->generate();
                }
            }
        }

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

    double getForce() {
        double force = 0;
        std::shared_lock<std::shared_mutex> lock(rw_mutex_);
        if(!isNeedExtraForce_){
            if (motor_ != nullptr) {
                force = std::fabs(motor_->getForceFeedback());
            }
        }else{
            force = output_feedback_force_;
        }
        return force;
    }

    void spin() {
        std::unique_lock<std::shared_mutex> lock(rw_mutex_);

        set_speed_iter_ = sub_speed_iter_ + 1;
        if (set_speed_iter_ >= FORK_DELAY) {
            set_speed_iter_ = 0;
        }
        double speed = v_delay_speed_[set_speed_iter_];
        if(forceReset_){
            LOG_INFO("%s --> forceReset_ on", solid_name_.c_str());
            speed = 0;
        }

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

            if(isNeedExtraForce_ && isNeedReadForce_){
                if((pos_sensor_value_ > (low_bound_ - 0.0002)) && (pos_sensor_value_ < (high_bound_ + 0.0002) ))
                    target_feedback_force_ = std::fabs(motor_->getForceFeedback());
                if(std::fabs(target_feedback_force_ - output_feedback_force_) > unit_force_){
                    if((target_feedback_force_ - output_feedback_force_) >= 0){
                        // increase
                        output_feedback_force_ += unit_force_;
                    }else{
                        // decrease
                        output_feedback_force_ -= unit_force_;
                    }
                }else{
                    output_feedback_force_ = target_feedback_force_;
                }
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
    bool isNeedExtraForce_ = false;
    bool isNeedReadForce_ = false;
    double output_feedback_force_ = 0;
    double target_feedback_force_ = 0;
    double unit_force_ = 0;
    std::string solid_name_ = "";
    bool forceReset_ = false;

    // 随机数生成器
    std::shared_ptr<RandomGenerator> random_generator_ = nullptr;
};

}  // namespace VNSim
