/**
 * @file E_shadow_lidar.cpp
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

#include "E_shadow_lidar.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string slam_1_webots_topic = "webots/Lidar/.54/PointCloud";
std::string slam_2_webots_topic = "webots/Lidar/.56/PointCloud";
std::string slam_1_webots_base_topic = "webots/LidarToBase/.54/PointCloud";
std::string slam_2_webots_base_topic = "webots/LidarToBase/.56/PointCloud";

AGVController::AGVController() : BaseLidarControl("webots_shadow_lidar") {
    // Sensor
    slam_1_ptr_ = std::make_shared<WLidar>("slam_1", 100);
    slam_1_ptr_->setSimulationNRLS("mid360.csv");

    slam_2_ptr_ = std::make_shared<WLidar>("slam_2", 100);
    slam_2_ptr_->setSimulationNRLS("mid360.csv");

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
    reflector_check_ptr_->setSensorMatrix4d("slam_1",
                                            slam_1_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d("slam_2",
                                            slam_2_ptr_->getMatrixFromLidar());

    v_while_spin_.push_back(bind(&WBase::spin, slam_1_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, slam_2_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, liftdoor_ptr_));

    // creat publish
    ecal_ptr_->addEcal(slam_1_webots_topic.c_str());
    ecal_ptr_->addEcal(slam_2_webots_topic.c_str());
    ecal_ptr_->addEcal(slam_1_webots_base_topic.c_str());
    ecal_ptr_->addEcal(slam_2_webots_base_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "slam_1_report", std::bind(&AGVController::Slam1ReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "slam_2_report", std::bind(&AGVController::Slam2ReportSpin, this)));
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

void VNSim::AGVController::liftdoorCallBack(
    const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::MTransfer transfer;
    transfer.ParseFromArray(data->buf, data->size);
    liftdoor_ptr_->setTag(transfer);
}

void AGVController::Slam1ReportSpin() {
    LOG_INFO("Slam1ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(slam_1_webots_topic, slam_1_ptr_, pose_ptr_,
                       slam_1_webots_base_topic);
    }
}

void AGVController::Slam2ReportSpin() {
    LOG_INFO("Slam2ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(slam_2_webots_topic, slam_2_ptr_, pose_ptr_,
                       slam_2_webots_base_topic);
    }
}
