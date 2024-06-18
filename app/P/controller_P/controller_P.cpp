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

#include "controller_P.h"
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
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ =
        std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "FLWheel");
    // l_ptr_ = std::make_shared<WWheel>("", "", "R_D_SteerSolid", "", "BRPS");
    // r_ptr_ = std::make_shared<WWheel>("", "", "L_D_SteerSolid", "", "BLPS");
    l_ptr_ = std::make_shared<WWheel>("", "", "", "", "BRPS");
    r_ptr_ = std::make_shared<WWheel>("", "", "", "", "BLPS");

    BP_ptr_ = std::make_shared<WLidar>("BP", "", 50);
    VertivalFov fov = {.begin = 0, .end = PI / 2};
    BP_ptr_->setFov(fov);

    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360", 100);
    mid360_ptr_->setSimulationNRLS("mid360.csv");

    pose_ptr_ = std::make_shared<WPose>("RobotNode");

    // TODO: creat task
    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, l_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, r_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, BP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, mid360_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));

    ecal_ptr_->addEcal("webot/P_msg");
    ecal_ptr_->addEcal("webot/pointCloud");
    ecal_ptr_->addEcal("webot/perception");

    m_thread_.insert(std::pair<std::string, std::thread>(
        "bp_report", std::bind(&NormalSTController::BpReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "mid360_report",
        std::bind(&NormalSTController::Mid360ReportSpin, this)));

    // std::thread local_thread(
    //     std::bind(&NormalSTController::BpReportSpin, this));
    // m_thread_["bp_report"] = std::move(local_thread);

    // ecal_ptr_->addEcal("webot/pointCloud");
    // ecal_ptr_->addEcal("webot/perception");
    ecal_ptr_->addEcal("svc_model_st/P_msg",
                       std::bind(&NormalSTController::onRemoteSerialMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    // payload_Up.set_allocated_imu(&payload_imu);
    // payload.set_allocated_up_msg(&payload_Up);
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
        // std::cout << "onMsg speed = " << payload.down_msg().steering_speed()
        //           << ", theta = " << payload.down_msg().steering_theta()
        //           << ", forkZ = " << payload.down_msg().forkspeedz()
        //           << std::endl;
        stree_ptr_->setSpeed(payload.down_msg().steering_speed(),
                             payload.down_msg().steering_theta());
        fork_ptr_->setVelocity(payload.down_msg().forkspeedz());
    }
}

void NormalSTController::sendSerialSpin() {
    sim_data_flow::PUp payload;
    payload.set_timestamp(timer_ptr_->getTimeStamp());
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());
    payload.set_l_wheel(l_ptr_->getSenosorValue());
    payload.set_r_wheel(r_ptr_->getSenosorValue());
    payload.set_steering_theta(stree_ptr_->getMotorYaw());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_orientation()->CopyFrom(imu_ptr_->getInertialValue());
    // TODO: 临时使用pose位置，后续要排查imu问题
    imu->mutable_orientation()->set_w(pose_ptr_->getVehicleYaw());

    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getGyroValue());
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getAccValue());

    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/P_msg", buf, payload.ByteSize());
}

void NormalSTController::Mid360ReportSpin() {
    LOG_INFO("Mid360ReportSpin start\n");
    sim_data_flow::WBPointCloud payload;

    while (!webotsExited_) {
        // FIXME: 可以修改为信号量触发
        if (!mid360_ptr_->checkDataReady()) {
            timer_ptr_->sleep<microseconds>(5);
            continue;
        }
        mid360_ptr_->getLocalPointCloud(payload);
        // if (payload.ByteSize() > BP_LIDAR_MSG_BUF) {
        //     LOG_WARN(
        //         "%s --> payload bytes size is larger, current = %d, expect =
        //         ",
        //         __FUNCTION__, payload.ByteSize(), BP_LIDAR_MSG_BUF);
        //     continue;
        // }
        uint8_t buf[payload.ByteSize()];
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send("webot/perception", buf, payload.ByteSize());
        timer_ptr_->sleep<milliseconds>(90);
    }
    return;
}

void NormalSTController::BpReportSpin() {
    LOG_INFO("BpReportSpin start\n");
    sim_data_flow::WBPointCloud payload;
    timer_ptr_->alarmTimerInit(50);
    while (!webotsExited_) {
        // FIXME: 可以修改为信号量触发
        if (!BP_ptr_->checkDataReady()) {
            timer_ptr_->sleep<microseconds>(5);
            continue;
        }
        BP_ptr_->getLocalPointCloud(payload);
        // if (payload.ByteSize() > BP_LIDAR_MSG_BUF) {
        //     LOG_WARN(
        //         "%s --> payload bytes size is larger, current = %d, expect
        //         =",
        //         __FUNCTION__, payload.ByteSize(), BP_LIDAR_MSG_BUF);
        //     continue;
        // }
        uint8_t buf[payload.ByteSize()];
        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send("webot/pointCloud", buf, payload.ByteSize());
        timer_ptr_->sleep<milliseconds>(45);
    }
    return;
}
