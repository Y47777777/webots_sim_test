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

#include "P_shadow_be.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

std::string HAP_webots_topic = "webots/Lidar/.200/PointCloud";

AGVController::AGVController() : BaseController("webots_shadow_be") {
    // Sensor
    HAP_ptr_ = std::make_shared<WLidar>("HAP", 50);
    HAP_ptr_->setSimulationNRLS("HAP.csv", HAP_ONCE_CLOUD_SIZE);

    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    // 删除所有物理属性，碰撞属性
    collision_ptr_ = std::make_shared<WCollision>();

    // // 高反
    // reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    // reflector_check_ptr_ = ReflectorChecker::getInstance();
    // reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // // 想高反判断部分注册外参
    // reflector_check_ptr_->setSensorMatrix4d("HAP",
    //                                         HAP_ptr_->getMatrixFromLidar());

    whileSpinPushBack(bind(&WBase::spin, HAP_ptr_));
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));
    whileSpinPushBack(bind(&WBase::spin, transfer_ptr_));

    // creat publish
    ecal_ptr_->addEcal(HAP_webots_topic.c_str());

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "HAP_report", std::bind(&AGVController::HapReportSpin, this)));
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

void AGVController::HapReportSpin() {
    LOG_INFO("HapReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(HAP_webots_topic, HAP_ptr_);
    }
}

// TODO:可以放到base中
bool AGVController::sendPointCloud(std::string topic,
                                   std::shared_ptr<WLidar> lidar_ptr) {
    if (lidar_ptr == nullptr) {
        return false;
    }

    if (!lidar_ptr->checkDataReady()) {
        Timer::getInstance()->sleep<microseconds>(5);
        return false;
    }

    Timer lidar_alarm;
    lidar_alarm.alarmTimerInit(lidar_ptr->getSleepTime());

    sim_data_flow::WBPointCloud payload;

    lidar_ptr->getLocalPointCloud(payload);

    payload.set_timestamp(pose_ptr_->getTimeStamp());

    // 在数量大的情况下约为10ms
    // for (int i = 0; i < payload.point_cloud_size(); i++) {
    //     if (ReflectorChecker::getInstance()->checkInReflector(
    //             payload.name(), &payload.point_cloud().at(i))) {
    //         payload.mutable_point_cloud()->at(i).set_intensity(200);
    //     }
    // }
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send(topic.c_str(), buf, payload.ByteSize());

    lidar_alarm.wait();
    return true;
}
