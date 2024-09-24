/**
 * @file E_shadow_lidar_0.cpp
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

#include "E_shadow_perception.h"

#include "dataTransform.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string lidar_3_webots_topic = "webots/Lidar/.112/PointCloud";
std::string lidar_0_webots_topic = "webots/Lidar/.109/PointCloud";
std::string lidar_3_webots_base_topic = "webots/LidarToBase/.112/PointCloud";

AGVController::AGVController() : BaseLidarControl("webots_shadow_perception") {
    // Sensor
    lidar_3_ptr_ = std::make_shared<WLidar>("lidar_3", 100);
    lidar_3_ptr_->setSimulationNRLS("mid360.csv");

    lidar_0_ptr_ = std::make_shared<WLidar>("lidar_0", 100);
    lidar_0_ptr_->setSimulationNRLS("mid360.csv");

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
    reflector_check_ptr_->setSensorMatrix4d("lidar_3",
                                            lidar_3_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d(
        "lidar_0", lidar_0_ptr_->getMatrixFromLidar());

    whileSpinPushBack((lidar_3_ptr_));
    whileSpinPushBack((lidar_0_ptr_));
    whileSpinPushBack((transfer_ptr_));
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));

    // creat publish
    ecal_ptr_->addEcal(lidar_3_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_0_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_3_webots_base_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_3_report", std::bind(&AGVController::Slam1ReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_0_report", std::bind(&AGVController::Slam2ReportSpin, this)));
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

void AGVController::Slam1ReportSpin() {
    LOG_INFO("Slam3ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(lidar_3_webots_topic, lidar_3_ptr_, pose_ptr_,
                       lidar_3_webots_base_topic);
    }
}

void AGVController::Slam2ReportSpin() {
    LOG_INFO("lidar_0ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(lidar_0_webots_topic, lidar_0_ptr_, pose_ptr_);
    }
}
