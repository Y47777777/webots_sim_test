/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 17:37:28
 * @FilePath: /webots_ctrl/app/ST/controller_ST/controller_ST.cpp
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#include <ecal/msg/protobuf/publisher.h>
#include "sim_data_flow/point_cloud.pb.h"
#include "time/time.h"
#include "geometry/geometry.h"
#include <qelapsedtimer.h>

#include "controller_ST.h"
#include <QElapsedTimer>

using namespace VNSim;
using namespace webots;

#define BP_LIDAR_MSG_BUF 900000
#define MAXIMUM_BP_UPLOAD 28800
#define MAXIMUM_MID360_UPLOAD 20722

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;

NormalSTController::NormalSTController() : BaseController() {
    // sensor init
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    BP_ptr_ = std::make_shared<WLidar>("BP");
    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360", 100);
    mid360_ptr_->setSimulationNRLS(true);

    // motor init
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "S");

    // TODO: creat task
    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, BP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, mid360_ptr_));

    std::thread local_thread(
        std::bind(&NormalSTController::BpReportSpin, this));
    m_thread_["bp_report"] = std::move(local_thread);

    ecal_ptr_->addEcal("webot/ST_msg");
    ecal_ptr_->addEcal("webot/pointCloud");
    ecal_ptr_->addEcal("webot/perception");
    ecal_ptr_->addEcal("svc_model_st/ST_msg",
                       std::bind(&NormalSTController::onRemoteSerialMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    payload_Up.set_allocated_imu(&payload_imu);
    payload.set_allocated_up_msg(&payload_Up);
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
        sim_data_flow::STMsg payload;
        payload.ParseFromArray(data->buf, data->size);
        stree_ptr_->setSpeed(payload.down_msg().steering_speed(),
                             payload.down_msg().steering_theta());
        fork_ptr_->setVelocity(payload.down_msg().forkspeedz());
    }
}

void NormalSTController::sendSerialSpin() {
    // payload_Up.set_forkposez(fork_ptr_->getSenosorValue());
    // payload_Up.set_steerposition(stree_ptr_->getSenosorValue());
    // payload_imu.add_orientation_covariance(imu_ptr_->getVehicleYaw());  //
    // // z
    // for (int i = 0; i < 3; i++) {
    //     payload_imu.add_angular_velocity_covariance(imu_ptr_->getGyroValue(i));
    //     payload_imu.add_linear_acceleration_covariance(
    //         imu_ptr_->getAccValue(i));
    // }
    // payload.SerializePartialToArray(buf, payload.ByteSize());
    // ecal_ptr_->send("webot/ST_msg", buf, payload.ByteSize());
    // payload_imu.Clear();

    sim_data_flow::STUp payload;
    // TODO: time stamp
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_orientation()->CopyFrom(imu_ptr_->getImuValue("Inertial"));
    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getImuValue("Gyro"));
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getImuValue("Acc"));

    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/ST_msg", buf, payload.ByteSize());
}

void NormalSTController::Mid360ReportSpin() {
    uint8_t buf[BP_LIDAR_MSG_BUF];
    LOG_INFO("Mid360ReportSpin start\n");
    sim_data_flow::WBPointCloud payload;

    while (!webotsExited_) {
        // FIXME: 可以修改为信号量触发
        if (!mid360_ptr_->checkDataReady()) {
            continue;
        }
        mid360_ptr_->getLocalPointCloud(payload, MAXIMUM_MID360_UPLOAD);
        if (payload.ByteSize() > BP_LIDAR_MSG_BUF) {
            LOG_WARN(
                "%s --> payload bytes size is larger, current = %d, expect = ",
                __FUNCTION__, payload.ByteSize(), BP_LIDAR_MSG_BUF);
            continue;
        }
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send("webot/perception", buf, payload.ByteSize());
        // wake_up_timer.wait();
    }
    return;
}

void NormalSTController::BpReportSpin() {
    uint8_t buf[BP_LIDAR_MSG_BUF];
    LOG_INFO("BpReportSpin start\n");
    sim_data_flow::WBPointCloud payload;

    while (!webotsExited_) {
        // FIXME: 可以修改为信号量触发
        if (!BP_ptr_->checkDataReady()) {
            continue;
        }
        BP_ptr_->getLocalPointCloud(payload, MAXIMUM_BP_UPLOAD);
        if (payload.ByteSize() > BP_LIDAR_MSG_BUF) {
            LOG_WARN(
                "%s --> payload bytes size is larger, current = %d, expect = ",
                __FUNCTION__, payload.ByteSize(), BP_LIDAR_MSG_BUF);
            continue;
        }
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send("webot/pointCloud", buf, payload.ByteSize());
    }
    return;
}
