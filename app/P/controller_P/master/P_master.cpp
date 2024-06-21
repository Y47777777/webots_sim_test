/**
 * @file P_master.cpp
 * @author weijchen (weijchen@visionnav.com)
 * @brief 主控 ctrl
 * @version 2.0
 * @date 2024-06-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ecal/msg/protobuf/publisher.h>
#include <qelapsedtimer.h>
#include <QElapsedTimer>

#include "sim_data_flow/point_cloud.pb.h"
#include "sim_data_flow/pose.pb.h"

#include "geometry/geometry.h"
#include "time/time.h"

#include "logvn/logvn.h"

#include "P_master.h"

using namespace VNSim;
using namespace webots;

#define BP_LIDAR_MSG_BUF 900000
#define MAXIMUM_BP_UPLOAD 28800
#define MAXIMUM_MID360_UPLOAD 20722

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

NormalSTController::NormalSTController() : BaseController() {
    // sensor init
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "FLWheel");
    l_ptr_ = std::make_shared<WWheel>("", "", "", "RS", "BRPS");
    r_ptr_ = std::make_shared<WWheel>("", "", "", "LS", "BLPS");

    pose_ptr_ = std::make_shared<WPose>("RobotNode");

    // TODO: creat task
    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, l_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, r_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));

    ecal_ptr_->addEcal("webot/P_msg");
    ecal_ptr_->addEcal("webot/transfer");

    ecal_ptr_->addEcal("svc_model_st/P_msg",
                       std::bind(&NormalSTController::onRemoteSerialMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

NormalSTController::~NormalSTController() {}

void NormalSTController::manualSetState(
    const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double fork_speed = 0;
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        fork_speed = msg.at("fork_speed");
        stree_ptr_->setSpeed(steer_speed, steer_yaw);
        fork_ptr_->setVelocity(fork_speed);
    }
}

void NormalSTController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["real_speed"] = 0;
    // TODO: fork_speed real_speed
}

void NormalSTController::whileSpin() {
    // 主循环 在super_->step()后
    this->sendSerialSpin();


}



void NormalSTController::onRemoteSerialMsg(
    const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::PMsg payload;
        payload.ParseFromArray(data->buf, data->size);

        stree_ptr_->setSpeed(payload.down_msg().steering_speed(),
                             payload.down_msg().steering_theta());
        fork_ptr_->setVelocity(payload.down_msg().forkspeedz());
    }
}

void NormalSTController::sendSerialSpin() {
    sim_data_flow::PUp payload;

    // payload.set_timestamp(time_sys_);
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());
    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());

    payload.set_steering_theta(stree_ptr_->getMotorYaw());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_orientation()->CopyFrom(imu_ptr_->getInertialValue());
    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getGyroValue());
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getAccValue());

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/P_msg", buf, payload.ByteSize());
}
