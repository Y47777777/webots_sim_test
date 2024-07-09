/**
 * @file E_master.cpp
 * @author xyjie (xyjie@visionnav.com)
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

#include "E_master.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

AGVController::AGVController() : BaseController("webots_master") {
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    fork_ptr_ = std::make_shared<WFork>("fork height motor", "ForkZAxis");
    forkY_ptr_ = std::make_shared<WFork>("YMotor", "ForkYAxis", "YSensor");
    forkP_ptr_ = std::make_shared<WFork>("PMotor", "ForkPAxis", "PSensor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolidL", "FLWheel");
    stree2_ptr_ =
        std::make_shared<WWheel>("FR", "SteerWheel", "SteerSolidR", "FLWheel");
    l_ptr_ = std::make_shared<WWheel>("", "", "", "RS", "BRPS");
    r_ptr_ = std::make_shared<WWheel>("", "", "", "RS", "BLPS");
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    lidar_pose_ptr_ = std::make_shared<WLidar>("perception", 100, false);
    transfer_ptr_ = std::make_shared<WTransfer>();

    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, stree2_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, l_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, r_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, forkY_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, forkP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));

    // pub
    ecal_ptr_->addEcal("webot/E_msg");
    ecal_ptr_->addEcal("webot/transfer");
    ecal_ptr_->addEcal("webot/pose");

    // sub
    ecal_ptr_->addEcal("svc/E_msg",
                       std::bind(&AGVController::subEMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void AGVController::manualSetState(const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double fork_speed = 0;
    static double forkY_speed = 0;
    static double forkP_speed = 0;
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        fork_speed = msg.at("fork_speed");
        forkY_speed = msg.at("forkY_speed");
        forkP_speed = msg.at("forkP_speed");
        stree_ptr_->setSpeed(steer_speed, steer_yaw);
        stree2_ptr_->setSpeed(steer_speed, steer_yaw);
        fork_ptr_->setVelocity(fork_speed);
        forkY_ptr_->setVelocity(forkY_speed);
        forkP_ptr_->setVelocity(forkP_speed);
    }
}

void AGVController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw()
    ;
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["forkY_speed"] = forkY_ptr_->getVelocityValue();
    msg["forkP_speed"] = forkP_ptr_->getVelocityValue();

    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["forkY_height"] = forkY_ptr_->getSenosorValue();
    msg["forkP_height"] = forkP_ptr_->getSenosorValue();

    msg["real_speed"] = 0;
    // TODO: fork_speed real_speed
}

void AGVController::whileSpin() {
    /* 主循环 在super_->step()后*/
    // 发送至svc
    pubSerialSpin();

    // 移动感知激光
    movePerLidarSpin();

    // 发送至shadow
    pubRobotPoseSpin();

    // 休眠一下
    Timer::getInstance()->sleep<microseconds>(10);
    pubTransferSpin();
}

void AGVController::movePerLidarSpin() {
    double fork_z = fork_ptr_->getSenosorValue();  // 米制

    // TODO: 激光随动
    lidar_pose_ptr_->moveLidar(fork_z);
    // LOG_INFO("fork :%.2f", fork_z);
}

void AGVController::pubTransferSpin() {
    sim_data_flow::MTransfer payload;
    transfer_ptr_->getTransfer(payload);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/transfer", buf, payload.ByteSize());
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

void AGVController::subEMsgCallBack(const char *topic_name,
                                    const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::EMsgDown payload;
        payload.ParseFromArray(data->buf, data->size);
        stree_ptr_->setSpeed(payload.steering_speed(),
                             payload.steering_theta());
        stree2_ptr_->setSpeed(payload.steering_speed(),
                              payload.steering_theta());
        fork_ptr_->setVelocity(payload.forkspeedz());
        fork_ptr_->setVelocity(payload.forkspeedy());
        forkP_ptr_->setVelocity(payload.forkspeedp());
    }
}

void AGVController::pubSerialSpin() {
    sim_data_flow::EMsgUp payload;
    payload.set_timestamp(time_stamp_);
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_forkposey(forkY_ptr_->getSenosorValue());
    payload.set_forkposep(forkP_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());
    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());
    payload.set_steering_theta(stree_ptr_->getMotorYaw());
    payload.set_gyroscope(imu_ptr_->getInertialYaw());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getGyroValue());
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getAccValue());

    // double *rotation = pose_ptr_->getRotaion();
    // LOG_INFO("imu: %.2f, robot: %.2f", imu_ptr_->getInertialYaw(),
    // rotation[3]);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/E_msg", buf, payload.ByteSize());
}
