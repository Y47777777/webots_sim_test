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

AGVController::AGVController() : BaseController() {
    // BP_ptr_ = std::make_shared<WLidar>("BP", "BP", 50);
    // VertivalFov fov = {.begin = 0, .end = (PI / 2 + 0.1)};
    // BP_ptr_->setFov(fov);

    mid360_ptr_ = std::make_shared<WLidar>("mid360", "MID360", 100);
    mid360_ptr_->setSimulationNRLS("mid360.csv");

    mid360Two_ptr_ = std::make_shared<WLidar>("mid360Two", "MID360Two", 100);
    mid360Two_ptr_->setSimulationNRLS("mid360.csv");

    pose_ptr_ = std::make_shared<WPose>("RobotNode");

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

    ecal_ptr_->addEcal("webot/pointCloud");
    ecal_ptr_->addEcal("webot/perception");
    ecal_ptr_->addEcal("webot/perceptionTwo");

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::TransferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

void AGVController::whileSpin() {
    // 主循环 在super_->step()后
    Mid360ReportSpin();
    Mid360TwoReportSpin();
    // BpReportSpin();
}

void AGVController::TransferCallBack(const char *topic_name,
                                     const eCAL::SReceiveCallbackData *data) {
    sim_data_flow::Pose pose;
    pose.ParseFromArray(data->buf, data->size);

    double transfer[3] = {pose.position().x(), pose.position().y(),
                          pose.position().z()};
    double rotation[4] = {pose.orientation().x(), pose.orientation().y(),
                          pose.orientation().z(), pose.orientation().w()};

    pose_ptr_->setTransfer(transfer, rotation, pose.timestamp());
}

// TODO: 改成一条函数的格式
void AGVController::Mid360ReportSpin() {
    sim_data_flow::WBPointCloud payload;

    if (!mid360_ptr_->checkDataReady()) {
        return;
    }
    mid360_ptr_->getLocalPointCloud(payload);

    payload.set_timestamp(pose_ptr_->getTimeStamp());

    for (int i = 0; i < payload.point_cloud_size(); i++) {
        if (ReflectorChecker::getInstance()->checkInReflector(
                payload.name(), &payload.point_cloud().at(i))) {
            payload.mutable_point_cloud()->at(i).set_intensity(200);
        }
    }
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/perception", buf, payload.ByteSize());
}

void AGVController::Mid360TwoReportSpin() {
    sim_data_flow::WBPointCloud payload;

    if (!mid360Two_ptr_->checkDataReady()) {
        return;
    }
    mid360Two_ptr_->getLocalPointCloud(payload);

    payload.set_timestamp(pose_ptr_->getTimeStamp());

    for (int i = 0; i < payload.point_cloud_size(); i++) {
        if (ReflectorChecker::getInstance()->checkInReflector(
                payload.name(), &payload.point_cloud().at(i))) {
            payload.mutable_point_cloud()->at(i).set_intensity(200);
        }
    }
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/perceptionTwo", buf, payload.ByteSize());
}

void AGVController::BpReportSpin() {
    sim_data_flow::WBPointCloud payload;

    if (!BP_ptr_->checkDataReady()) {
        return;
    }
    BP_ptr_->getLocalPointCloud(payload);

    payload.set_timestamp(pose_ptr_->getTimeStamp());
    for (int i = 0; i < payload.point_cloud_size(); i++) {
        if (ReflectorChecker::getInstance()->checkInReflector(
                payload.name(), &payload.point_cloud().at(i))) {
            payload.mutable_point_cloud()->at(i).set_intensity(200);
        }
    }
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/pointCloud", buf, payload.ByteSize());
}
