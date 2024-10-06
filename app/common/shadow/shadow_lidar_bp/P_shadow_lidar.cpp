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

#include "P_shadow_lidar.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string BP_webots_topic = "webots/Lidar/.55/PointCloud";
std::string lidar_0_webots_topic = "webots/Lidar/.109/PointCloud";

AGVController::AGVController() : BaseLidarControl("webots_shadow_lidar") {
    // Sensor
    BP_ptr_ = std::make_shared<WLidar>("BP", 50);
    VertivalFov fov = {.begin = 0, .end = (PI / 2 + 0.1)};
    BP_ptr_->setFov(fov);

    lidar_0_ptr_ = std::make_shared<WLidar>("lidar_0", 100);
    lidar_0_ptr_->setSimulationNRLS("mid360.csv");

    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    // 升降门
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(true);

    // 删除所有物理属性，碰撞属性
    collision_ptr_ = std::make_shared<WCollision>();

    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // 想高反判断部分注册外参
    reflector_check_ptr_->setSensorMatrix4d("BP", BP_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d("lidar_0", lidar_0_ptr_->getMatrixFromLidar());

    whileSpinPushBack((BP_ptr_));
    whileSpinPushBack((lidar_0_ptr_));
    whileSpinPushBack((pose_ptr_));
    whileSpinPushBack((transfer_ptr_));
    whileSpinPushBack((liftdoor_ptr_));

    // creat publish
    ecal_ptr_->addEcal(BP_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_0_webots_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this, std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("webot/liftdoor",
                       std::bind(&AGVController::liftdoorCallBack, this, std::placeholders::_1, std::placeholders::_2));
    // 创建线程
    m_thread_.insert(
        std::pair<std::string, std::thread>("lidar_bp_report", std::bind(&AGVController::BpReportSpin, this)));
    m_thread_.insert(
        std::pair<std::string, std::thread>("lidar_0_report", std::bind(&AGVController::Lidar0ReportSpin, this)));
}

void AGVController::whileSpin() {
    // 主循环 在super_->step()后
}

void AGVController::poseCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::Pose pose;
    pose.ParseFromArray(data->buf, data->size);

    double transfer[3] = {pose.position().x(), pose.position().y(), pose.position().z()};
    double rotation[4] = {pose.orientation().x(), pose.orientation().y(), pose.orientation().z(),
                          pose.orientation().w()};

    pose_ptr_->setTransferWithTime(transfer, rotation, pose.timestamp());
}

void AGVController::transferCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::MTransfer transfer;
    transfer.ParseFromArray(data->buf, data->size);
    transfer_ptr_->setTransfer(transfer);
}

void AGVController::BpReportSpin() {
    LOG_INFO("%s start --> %s", __FUNCTION__, lidar_0_webots_topic.c_str());
    while (!webotsExited_) { sendPointCloud(BP_webots_topic, BP_ptr_, pose_ptr_); }
}

void AGVController::Lidar0ReportSpin() {
    LOG_INFO("%s start --> %s", __FUNCTION__, BP_webots_topic.c_str());
    while (!webotsExited_) { sendPointCloud(lidar_0_webots_topic, lidar_0_ptr_, pose_ptr_); }
}

void VNSim::AGVController::liftdoorCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::MTransfer transfer;
    transfer.ParseFromArray(data->buf, data->size);
    liftdoor_ptr_->setTag(transfer);
}
