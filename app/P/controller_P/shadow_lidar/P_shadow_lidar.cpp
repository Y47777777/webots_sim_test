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
std::string MID360_webots_topic = "webots/Lidar/.54/PointCloud";
std::string MID360Two_webots_topic = "webots/Lidar/.56/PointCloud";

AGVController::AGVController() : BaseController("webots_shadow_lidar") {
    // Sensor
    // BP_ptr_ = std::make_shared<WLidar>("BP", "BP", 50);
    // VertivalFov fov = {.begin = 0, .end = (PI / 2 + 0.1)};
    // BP_ptr_->setFov(fov);

    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360", 100);
    mid360_ptr_->setSimulationNRLS("mid360.csv");

    mid360Two_ptr_ = std::make_shared<WLidar>("mid360Two", "MID360Two", 100);
    mid360Two_ptr_->setSimulationNRLS("mid360.csv");

    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");

    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    reflector_check_ptr_->setSensorMatrix4d("mid360",
                                            mid360_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d(
        "mid360Two", mid360Two_ptr_->getMatrixFromLidar());

    // reflector_check_ptr_->setSensorMatrix4d("BP",
    //                                         BP_ptr_->getMatrixFromLidar());

    // v_while_spin_.push_back(bind(&WBase::spin, BP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, mid360_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, mid360Two_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));

    ecal_ptr_->addEcal(BP_webots_topic.c_str());
    ecal_ptr_->addEcal(MID360_webots_topic.c_str());
    ecal_ptr_->addEcal(MID360Two_webots_topic.c_str());

    ecal_ptr_->addEcal("webot/robot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "bp_report", std::bind(&AGVController::BpReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "mid360_report", std::bind(&AGVController::Mid360ReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "mid360two_report",
        std::bind(&AGVController::Mid360TwoReportSpin, this)));
}

void AGVController::whileSpin() {
    // 主循环 在super_->step()后
}

void AGVController::transferCallBack(const char *topic_name,
                                     const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::Pose pose;
    pose.ParseFromArray(data->buf, data->size);

    double transfer[3] = {pose.position().x(), pose.position().y(),
                          pose.position().z()};
    double rotation[4] = {pose.orientation().x(), pose.orientation().y(),
                          pose.orientation().z(), pose.orientation().w()};

    pose_ptr_->setTransfer(transfer, rotation, pose.timestamp());
}

void AGVController::BpReportSpin() {
    LOG_INFO("BpReportSpin start\n");
    while (!webotsExited_) { sendPointCloud(BP_webots_topic, BP_ptr_); }
}

void AGVController::Mid360ReportSpin() {
    LOG_INFO("Mid360ReportSpin start\n");
    while (!webotsExited_) { sendPointCloud(MID360_webots_topic, mid360_ptr_); }
}

void AGVController::Mid360TwoReportSpin() {
    LOG_INFO("Mid360TwoReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(MID360Two_webots_topic, mid360Two_ptr_);
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
    for (int i = 0; i < payload.point_cloud_size(); i++) {
        if (ReflectorChecker::getInstance()->checkInReflector(
                payload.name(), &payload.point_cloud().at(i))) {
            payload.mutable_point_cloud()->at(i).set_intensity(200);
        }
    }
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send(topic.c_str(), buf, payload.ByteSize());

    lidar_alarm.wait();
    return true;
}
