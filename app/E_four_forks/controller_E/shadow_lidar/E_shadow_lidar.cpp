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
std::vector<std::shared_ptr<WBarcode>> WBarcodeScan::v_barcode_;
bool WBarcodeScan::enable_;

std::string lidar_2_webots_topic = "webots/Lidar/.111/PointCloud";
std::string lidar_4_webots_topic = "webots/Lidar/.113/PointCloud";
std::string lidar_2_webots_base_topic = "webots/LidarToBase/.111/PointCloud";
std::string lidar_4_webots_base_topic = "webots/LidarToBase/.113/PointCloud";

AGVController::AGVController() : BaseLidarControl("webots_shadow_lidar") {
    // Sensor
    lidar_2_ptr_ = std::make_shared<WLidar>("lidar_2", 100);
    lidar_2_ptr_->setSimulationNRLS("mid360.csv");

    lidar_4_ptr_ = std::make_shared<WLidar>("lidar_4", 100);
    lidar_4_ptr_->setSimulationNRLS("mid360.csv");

    // 机器人位姿
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    transfer_ptr_ = std::make_shared<WTransfer>();

    // 升降门
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(true);

    // 删除所有物理属性，碰撞属性
    collision_ptr_ = std::make_shared<WCollision>();

    notifier1_ = std::make_shared<CoderNotifyer>();
    barcode_scaner_ptr_ = std::make_shared<WBarcodeScan>(notifier1_, "Scaner");
    manager1_ = std::make_shared<CoderManager>(notifier1_);

    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // 想高反判断部分注册外参
    reflector_check_ptr_->setSensorMatrix4d("lidar_2",
                                            lidar_2_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d("lidar_4",
                                            lidar_4_ptr_->getMatrixFromLidar());

    v_while_spin_.push_back(bind(&WBase::spin, lidar_2_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, lidar_4_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, liftdoor_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, barcode_scaner_ptr_));

    // creat publish
    ecal_ptr_->addEcal(lidar_2_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_2_webots_base_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_webots_base_topic.c_str());
    ecal_ptr_->addEcal("CodeScanner/read");

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("CodeScanner/write",
                       std::bind(&AGVController::onCoderScannerMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_2_report", std::bind(&AGVController::Slam1ReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_4_report", std::bind(&AGVController::Slam2ReportSpin, this)));
    // TODO: add more scaner thread here, l, r, m
    m_thread_.insert(std::pair<std::string, std::thread>(
        "codeScan1", std::bind(&AGVController::Coder1ReportSpin, this)));
}

AGVController::~AGVController() {
    manager1_->stop();
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

void AGVController::onCoderScannerMsg(const char *topic_name,
                                      const eCAL::SReceiveCallbackData *data) {
    char CMD[100];
    memcpy(CMD, data->buf, data->size);
    if (strcmp(CMD, "Start") == 0) {
        barcode_scaner_ptr_->scanEnableSet(true);
        manager1_->start_scan();
    } else if (strcmp(CMD, "Stop") == 0) {
        barcode_scaner_ptr_->scanEnableSet(false);
        manager1_->stop_scan();
    }
}

void AGVController::Slam1ReportSpin() {
    LOG_INFO("Slam1ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(lidar_2_webots_topic, lidar_2_ptr_, pose_ptr_,
                       lidar_2_webots_base_topic);
    }
}

void AGVController::Slam2ReportSpin() {
    LOG_INFO("Slam2ReportSpin start\n");
    while (!webotsExited_) {
        sendPointCloud(lidar_4_webots_topic, lidar_4_ptr_, pose_ptr_,
                       lidar_4_webots_base_topic);
    }
}

void AGVController::Coder1ReportSpin() {
    LOG_INFO("Code scanner 1 online...");
    int ret = 0;
    char code_str[1024];
    int counter = 0;
    while (!webotsExited_) {
        // wait next timer...
        // ignore incoming start, when scanner is running
        ret = manager1_->run();
        if (ret == 0) {
            const char *code = barcode_scaner_ptr_->getQRCode();
            if (strlen(code) == 0) {
                sprintf(code_str, "%s", "NoRead");
            } else {
                sprintf(code_str, "%s", code);
            }
            LOG_INFO("code = [%s]", code_str);
            ecal_ptr_->send("CodeScanner/read", (const uint8_t *) code_str,
                            strlen(code_str));
        }
    }
    LOG_INFO("Code scanner 1 close offline...");
}
