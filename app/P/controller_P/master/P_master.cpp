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

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

AGVController::AGVController() : BaseController("webots_master") {
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "FLWheel");
    l_ptr_ = std::make_shared<WWheel>("", "", "", "RS", "BRPS");
    r_ptr_ = std::make_shared<WWheel>("", "", "", "LS", "BLPS");
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, l_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, r_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));

    // pub
    ecal_ptr_->addEcal("webot/P_msg");
    ecal_ptr_->addEcal("webot/transfer");
    ecal_ptr_->addEcal("webot/pose");

    // sub
    ecal_ptr_->addEcal("svc/P_msg",
                       std::bind(&AGVController::subPMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void AGVController::manualSetState(const std::map<std::string, double> &msg) {
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

void AGVController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["real_speed"] = 0;
    // TODO: fork_speed real_speed
}

void AGVController::whileSpin() {
    /* 主循环 在super_->step()后*/
    // 发送至svc
    pubSerialSpin();

    // 发送至shadow
    pubRobotPoseSpin();
    pubTransferSpin();
}

void AGVController::pubTransferSpin() {
    // std::string msg_str = transfer_ptr_->getTransfer().dump();
    // std::vector<uint8_t> buf(msg_str.begin(), msg_str.end());

    // //publish
    // ecal_ptr_->send("webot/transfer", buf.data(), buf.size());
}

void AGVController::pubRobotPoseSpin() {
    double *tran = pose_ptr_->getTransfer();
    double *rotation = pose_ptr_->getRotaion();

    sim_data_flow::Pose pose;
    pose.mutable_position()->set_x(tran[0]);
    pose.mutable_position()->set_y(tran[1]);
    pose.mutable_position()->set_z(tran[2]);

    pose.mutable_orientation()->set_x(rotation[0]);
    pose.mutable_orientation()->set_y(rotation[1]);
    pose.mutable_orientation()->set_z(rotation[2]);
    pose.mutable_orientation()->set_w(rotation[3]);
    pose.set_timestamp(time_stamp_);

    uint8_t buf[pose.ByteSize()];
    pose.SerializePartialToArray(buf, pose.ByteSize());
    ecal_ptr_->send("webot/pose", buf, pose.ByteSize());
}

void AGVController::subPMsgCallBack(const char *topic_name,
                                    const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::PMsgDown payload;
        payload.ParseFromArray(data->buf, data->size);

        stree_ptr_->setSpeed(payload.steering_speed(),
                             payload.steering_theta());
        fork_ptr_->setVelocity(payload.forkspeedz());
    }
}

void AGVController::pubSerialSpin() {
    sim_data_flow::PMsgUp payload;

    payload.set_timestamp(time_stamp_);
    payload.set_forkposeZ(fork_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());

    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());

    payload.set_steering_theta(stree_ptr_->getMotorYaw());

    payload.set_gyroscope(imu_ptr_->getInertialYaw());

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/P_msg", buf, payload.ByteSize());
}
