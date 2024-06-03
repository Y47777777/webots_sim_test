#pragma once

#include <webots/Motor.hpp>
#include <webots/Node.hpp>
#include <webots/PositionSensor.hpp>
#include <shared_mutex>

#include "webots_device/w_base.h"
#include "logvn/logvn.h"

#include "geometry/geometry.h"

namespace VNSim {
using namespace webots;

class WWheel : public WBase {
   public:
    /*
     * @brief  构建 motor对象
     *
     * @param[motor_name]       webots 下 motor名字
     * @param[yaw_node_name]    最外层旋转pose 位置
     * @param[solid_name]       需要控制旋初始旋转轴的 solid
     * @param[raidus]           轮径
     * @param[sensor_name]      positionSensor
     */
    WWheel(std::string motor_name = "", std::string yaw_node_name = "",
           std::string solid_name = "", std::string radius_name = "",
           std::string sensor_name = "")
        : WBase() {
        LOG_INFO("init wheel: %s,%s,%s,%s,%s", motor_name.c_str(),
                 yaw_node_name.c_str(), solid_name.c_str(), radius_name.c_str(),
                 sensor_name.c_str());

        // creat motor
        motor_ = super_->getMotor(motor_name);
        if (motor_ != nullptr) {
            motor_->setPosition(INFINITY);
            motor_->setVelocity(0);

            LOG_INFO("creat motor: %s", motor_name.c_str());
        }

        Node *mid360Node = super_->getFromDef(yaw_node_name);
        Field *mid360_tf = mid360Node->getField("rotation");
        const double *mid360_pose = mid360_tf->getSFRotation();

        // creat yaw node
        Node *yaw_node = super_->getFromDef(yaw_node_name);
        if (yaw_node != nullptr) {
            yaw_rotation_ptr_ = yaw_node->getField("rotation");

            memcpy(&set_yaw_rotation_, yaw_rotation_ptr_->getSFRotation(),
                   4 * sizeof(set_yaw_rotation_[0]));
            memcpy(&get_yaw_rotation_, set_yaw_rotation_,
                   4 * sizeof(get_yaw_rotation_[0]));

            LOG_INFO("yaw node :", yaw_node_name.c_str());
            LOG_INFO("yaw rotation %.3f, %.3f, %.3f, %.3f",
                     set_yaw_rotation_[0], set_yaw_rotation_[1],
                     set_yaw_rotation_[2], set_yaw_rotation_[3]);
        }

        // creat wheel solid
        Node *soild_node = super_->getFromDef(solid_name);
        if (soild_node != nullptr) {
            solid_rotation_ptr_ = soild_node->getField("rotation");
            solid_translation_ptr_ = soild_node->getField("translation");

            memcpy(solid_init_rotation_, solid_rotation_ptr_->getSFRotation(),
                   4 * sizeof(solid_init_rotation_[0]));
            memcpy(solid_init_translation_,
                   solid_translation_ptr_->getSFVec3f(),
                   3 * sizeof(solid_translation_ptr_[0]));

            LOG_INFO("soild node :", solid_name.c_str());
            LOG_INFO("soild rotation %.3f, %.3f, %.3f, %.3f",
                     solid_init_rotation_[0], solid_init_rotation_[1],
                     solid_init_rotation_[2], solid_init_rotation_[3]);

            LOG_INFO("soild translation %.3f, %.3f, %.3f",
                     solid_init_translation_[0], solid_init_translation_[1],
                     solid_init_translation_[2]);
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
                LOG_INFO("creat motor:%s  motor sensor :%s", motor_name.c_str(),
                         sensor_name.c_str());
            }
        }

        // set wheel radius
        if (radius_name != "") {
            radius_ = super_->getFromDef(radius_name)
                          ->getField("radius")
                          ->getSFFloat();
            LOG_INFO("%s radius :%s", motor_name.c_str(), radius_name.c_str());
        }
    }

    ~WWheel(){};

    void setVelocity(double v) { speed_ = v / radius_; }

    void setYaw(double yaw) {
        yaw = yaw > PI / 2 ? PI / 2 : yaw;
        yaw = yaw < -PI / 2 ? -PI / 2 : yaw;

        set_yaw_rotation_[3] = yaw;
    }

    void setSpeed(double v, double yaw) {
        setVelocity(v);
        setYaw(yaw);
    }

    double getSenosorValue() { return pos_sensor_value_; }
    double getMotorYaw() { return get_yaw_rotation_[3]; }
    double getSpeed() { return speed_ * radius_; }

    void spin() {
        // get wheel pos value
        if (position_sensor_ != nullptr) {
            pos_sensor_value_ = position_sensor_->getValue();
        }

        // set wheel rotate center
        if (solid_rotation_ptr_ != nullptr) {
            // set translation
            solid_translation_ptr_->setSFVec3f(solid_init_translation_);
            // set rotation axis
            const double *angle = solid_rotation_ptr_->getSFRotation();
            solid_init_rotation_[3] = angle[3];
            solid_rotation_ptr_->setSFRotation(solid_init_rotation_);
        }

        // set speed
        if (motor_ != nullptr) {
            motor_->setVelocity(speed_);
        }

        // yaw
        if (yaw_rotation_ptr_) {
            // get
            memcpy(&get_yaw_rotation_, yaw_rotation_ptr_->getSFRotation(),
                   4 * sizeof(get_yaw_rotation_[0]));

            // set
            yaw_rotation_ptr_->setSFRotation(set_yaw_rotation_);
        }
    }

   private:
    // motor
    Motor *motor_ = nullptr;
    double speed_ = 0;
    double radius_ = 1;

    // stree
    Field *yaw_rotation_ptr_ = nullptr;
    double set_yaw_rotation_[4] = {0};
    double get_yaw_rotation_[4] = {0};

    // pos_sensor
    double pos_sensor_value_ = 0;
    PositionSensor *position_sensor_ = nullptr;

    // circle solid
    Field *solid_rotation_ptr_ = nullptr;
    Field *solid_translation_ptr_ = nullptr;
    Field *steer_solid_ptr_ = nullptr;
    double solid_init_rotation_[4] = {0};
    double solid_init_translation_[3] = {0};
};

}  // namespace VNSim
