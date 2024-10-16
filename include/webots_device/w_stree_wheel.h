/*
 * @Author: weijchen weijchen@visionnav.com / CJA
 * @Date: 2024-10-16 18:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 11:24:20
 * @FilePath: /webots_ctrl/include/webots_device/w_stree_wheel.h
 * @Description: webots based on wheel接口,to control Hinge2Joint StreeWheel
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#pragma once

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <shared_mutex>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

#include "geometry/geometry.h"

namespace VNSim {
using namespace webots;

class SWWheel : public WBase {
   public:
    /*//构建类 可做定向轮可两关节舵轮
     * @brief  构建 motor对象
     *
     * @param[motorrun_name]       webots 下 motor名字
     * @param[motorturn_name]       转弯电机名
     * @param[radius_name]          轮半径如geometry这个节点的DEF名
     * @param[runsensor_name]       执行位置传感器
     * @param[turnsensor_name]      转弯位置传感器
    
     */
    SWWheel(std::string motorrun_name = "",std::string motorturn_name = "", std::string radius_name = "", 
        std::string runsensor_name = "", std::string turnsensor_name = "")
        : WBase() 
    {
        motorrun_name_ = motorrun_name;
        motorturn_name_ = motorturn_name;
        runsensor_name_ = runsensor_name;
        turnsensor_name_ = turnsensor_name;

        LOG_INFO("init streewheel: %s,%s,%s,%s,%s", motorrun_name_.c_str(), motorturn_name_.c_str(),
                 radius_name.c_str(), runsensor_name_.c_str(), turnsensor_name_.c_str());

        /************************creat motor************************/
        motorRun_ = super_->getMotor(motorrun_name_);
        if (motorRun_ != nullptr) 
        {
            motorRun_->setPosition(INFINITY);
            motorRun_->setVelocity(0);
            LOG_INFO("creat runmotor: %s", motorrun_name_.c_str());
        }
        else
        {
            LOG_INFO("Can't create runmotor: %s", motorrun_name_.c_str());

        }
        motorturn_ = super_ ->getMotor(motorturn_name_);
        if(motorturn_ != nullptr)
        {
            motorturn_->setPosition(0);
            //motorturn_->setVelocity(0);
            LOG_INFO("creat motor: %s", motorturn_name_.c_str());
        }
        else
        {
            LOG_INFO("Can't create motor: %s", motorturn_name_.c_str());

        }
        /*******************creat pose sensor****************************/ 
        //create runsensor
        if (runsensor_name_ == "" && motorRun_ != nullptr) 
        {
            position_runsensor_ = motorRun_->getPositionSensor();
        } 
        else 
        {
            position_runsensor_ = super_->getPositionSensor(runsensor_name_);
        }

        if (position_runsensor_ != nullptr) 
        {
            position_runsensor_->enable(step_duration_);
            last_pos_runsensor_value_ = position_runsensor_->getValue();
            LOG_INFO("creat motor:%s  motor_runsensor :%s", motorrun_name_.c_str(), runsensor_name_.c_str());
        }
        //create turnsensor
        if (turnsensor_name_ == "" && motorturn_ != nullptr) 
        {
            position_turnsensor_ = motorturn_->getPositionSensor();
        } 
        else 
        {
            position_turnsensor_ = super_->getPositionSensor(turnsensor_name_);
        }

        if (position_turnsensor_ != nullptr) 
        {
            position_turnsensor_->enable(step_duration_);
            last_pos_runsensor_value_ = position_turnsensor_->getValue();
            LOG_INFO("creat motor:%s  motor_tunsensor :%s",motorturn_name_.c_str(), turnsensor_name_.c_str());
        }
        /**************************set wheel radius***************************/ 
        if (radius_name != "") 
        {
            radius_ = super_->getFromDef(radius_name)->getField("radius")->getSFFloat();
            LOG_INFO("%s %s radius :%s", motorrun_name_.c_str(), motorturn_name_.c_str(), radius_name.c_str());
        }
    }

    ~SWWheel(){};

    void setVelocity(double v) 
    {
        AutoAtomicLock lock(spin_mutex_);
        speed_ = v / radius_;
    }

    void setYaw(double yaw) 
    {
        AutoAtomicLock lock(spin_mutex_);
        yaw_ = yaw;
    }

    void stree_run(double v, double yaw) 
    {
        setVelocity(v);
        setYaw(yaw);
    }

    // TODO: 这个接口定义的不好，要改一下
    double getWheelArcLength() { return getSenosorValue() * radius_; }

    double getSenosorValue() {
        AutoAtomicLock lock(spin_mutex_);
        double result = pos_runsensor_value_;
        pos_runsensor_value_ = 0;
        return result;
    }
    // TODO: 获取转向角度
    double getTurnSenosorValue() {
        AutoAtomicLock lock(spin_mutex_);
        double result = pos_turnsensor_value_;
        pos_turnsensor_value_ = 0;
        return result;
    }

    //因为webots中存在上锁机制，所以不能直接获取速度，需要通过寄存值获取
    //线速度
    double getSpeed() {
        AutoAtomicLock lock(spin_mutex_);
        return speed_ * radius_;
    }

    double getMotorYaw() {
        AutoAtomicLock lock(spin_mutex_);
        return yaw_;
    }
 
    void spin() 
    {
        // get run pos value
        if (position_runsensor_ != nullptr) 
        {
            // 增量式编码器，只计算增量
            double now_value = position_runsensor_->getValue();
            double diff = now_value - last_pos_runsensor_value_;

            // 死区
            if (fabs(diff) > 0.00005) {
                pos_runsensor_value_ += diff;
            }
            last_pos_runsensor_value_ = now_value;
        }
        // get turn pos value
        if(position_turnsensor_ != nullptr)
        {
            pos_turnsensor_value_ = position_turnsensor_->getValue();
        }  
        // set speed
        if (motorRun_ != nullptr) {
            motorRun_->setVelocity(speed_);
        }
        // yaw
        if (motorturn_ != nullptr) 
        {
            motorturn_->setPosition(yaw_);
        }
    }

   private:
    // motor
    std::string motorrun_name_;
    std::string motorturn_name_;
    std::string runsensor_name_;
    std::string turnsensor_name_;
    Motor *motorRun_ = nullptr;
    Motor *motorturn_ = nullptr;
    double speed_ = 0;  //角速度
    double radius_ = 1;
    double yaw_ = 0;  // 偏航角度

    // pos_sensor
    double pos_runsensor_value_ = 0;
    double last_pos_runsensor_value_ = 0;
    PositionSensor *position_runsensor_ = nullptr;

    double pos_turnsensor_value_ = 0;
    double last_pos_turnsensor_value_ = 0;
    PositionSensor *position_turnsensor_ = nullptr;
};

}  // namespace VNSim
