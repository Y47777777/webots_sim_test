/**
 * @file P_master.cpp
 * @author weijchen (weijchen@visionnav.com)
 * @brief lidar shadow (雷达镜像)
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

#include "shadow_perception.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string BP_webots_topic = "webots/Lidar/.55/PointCloud";
std::string MID360Per_webots_topic = "webots/Lidar/.57/PointCloud";

AGVController::AGVController() : BaseLidarControl("webots_shadow_perception") {
    // Sensor
    BP_ptr_ = std::make_shared<WLidar>("BP", 50);
    VertivalFov fov = {.begin = 0, .end = (PI / 2)};
    BP_ptr_->setFov(fov);

    mid360_perception_ptr_ = std::make_shared<WLidar>("mid360Per", 100);
    mid360_perception_ptr_->setSimulationNRLS("mid360.csv",
                                              MID360_ONCE_CLOUD_SIZE);

    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    // 删除所有物理属性，碰撞属性
    collision_ptr_ = std::make_shared<WCollision>();

    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // 想高反判断部分注册外参
    reflector_check_ptr_->setSensorMatrix4d("BP",
                                            BP_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d(
        "mid360Per", mid360_perception_ptr_->getMatrixFromLidar());

    v_while_spin_.push_back(bind(&WBase::spin, BP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, mid360_perception_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));

    // creat publish
    ecal_ptr_->addEcal(BP_webots_topic.c_str());
    ecal_ptr_->addEcal(MID360Per_webots_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "bp_report", std::bind(&AGVController::BpReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "mid360per_report",
        std::bind(&AGVController::Mid360PerceptionReportSpin, this)));
}

void AGVController::whileSpin() {
    // 主循环 在super_->step()后
}

void AGVController::poseCallBack(const char *topic_name,
                                 const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::Pose pose;
    pose.ParseFromArray(data->buf, data->size);

    double transfer[3] = {pose.position().x(), pose.position().y(),
                          pose.position().z()};
    double rotation[4] = {pose.orientation().x(), pose.orientation().y(),
                          pose.orientation().z(), pose.orientation().w()};

    pose_ptr_->setTransferWithTime(transfer, rotation, pose.timestamp());
}

void AGVController::transferCallBack(const char *topic_name,
                                     const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::MTransfer transfer;
    transfer.ParseFromArray(data->buf, data->size);
    transfer_ptr_->setTransfer(transfer);
}

void AGVController::BpReportSpin() {
    LOG_INFO("BpReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(BP_webots_topic, BP_ptr_, pose_ptr_);
    }
}

void AGVController::Mid360PerceptionReportSpin() {
    LOG_INFO("Mid360TwoReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(MID360Per_webots_topic, mid360_perception_ptr_,
                       pose_ptr_);
    }
}
