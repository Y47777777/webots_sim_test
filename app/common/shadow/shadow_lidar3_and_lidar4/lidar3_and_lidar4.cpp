/**
 * @file E_shadow_lidar_4.cpp
 * @author xyjie (xyjie@visionnav.com)
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

#include "geometry/geometry.h"
#include "time/time.h"

#include "logvn/logvn.h"

#include "lidar3_and_lidar4.h"

#include "dataTransform.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string lidar_3_webots_topic = "webots/Lidar/.112/PointCloud";
std::string lidar_4_webots_topic = "webots/Lidar/.113/PointCloud";

AGVController::AGVController() : BaseLidarControl("shadow_lidar3_and_lidar4") {
    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    // Sensor
    lidar_3_ptr_ = std::make_shared<WLidar>("lidar_3", pose_ptr_, 100);
    lidar_3_ptr_->setSimulationNRLS("mid360.csv");

    lidar_4_ptr_ = std::make_shared<WLidar>("lidar_4", pose_ptr_, 100);
    lidar_4_ptr_->setSimulationNRLS("mid360.csv");

    // 删除所有物理属性，碰撞属性
    collision_ptr_ = std::make_shared<WCollision>();

    // 升降门
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(true);


    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // 想高反判断部分注册外参
    reflector_check_ptr_->setSensorMatrix4d("lidar_3", lidar_3_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d("lidar_4", lidar_4_ptr_->getMatrixFromLidar());

    whileSpinPushBack((transfer_ptr_));
    whileSpinPushBack(bind(&WBase::spin, liftdoor_ptr_));
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));

    whileSpinPushBack((lidar_3_ptr_));
    whileSpinPushBack((lidar_4_ptr_));

    // creat publish
    ecal_ptr_->addEcal(lidar_3_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_webots_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/liftdoor",
                       std::bind(&AGVController::liftdoorCallBack, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this, std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this, std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(
        std::pair<std::string, std::thread>("lidar_3_report", std::bind(&AGVController::Lidar3ReportSpin, this)));
    m_thread_.insert(
        std::pair<std::string, std::thread>("lidar_4_report", std::bind(&AGVController::Lidar4ReportSpin, this)));
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

void AGVController::Lidar3ReportSpin() {
    LOG_INFO("Lidar3ReportSpin start\n");
    while (!webotsExited_) { sendPointCloud(lidar_3_webots_topic, lidar_3_ptr_, pose_ptr_); }
}

void AGVController::Lidar4ReportSpin() {
    LOG_INFO("lidar_4ReportSpin start\n");
    while (!webotsExited_) { sendPointCloud(lidar_4_webots_topic, lidar_4_ptr_, pose_ptr_); }
}

void VNSim::AGVController::liftdoorCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::MTransfer transfer;
    transfer.ParseFromArray(data->buf, data->size);
    liftdoor_ptr_->setTag(transfer);
}