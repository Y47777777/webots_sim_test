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

    notifier_l_ = std::make_shared<CoderNotifyer>();
    barcode_scaner_l_ptr_ =
        std::make_shared<WBarcodeScan>(notifier_l_, "ScanerL");
    manager_l_ = std::make_shared<CoderManager>(notifier_l_);

    notifier_m_ = std::make_shared<CoderNotifyer>();
    barcode_scaner_m_ptr_ =
        std::make_shared<WBarcodeScan>(notifier_m_, "ScanerM");
    manager_m_ = std::make_shared<CoderManager>(notifier_m_);

    notifier_r_ = std::make_shared<CoderNotifyer>();
    barcode_scaner_r_ptr_ =
        std::make_shared<WBarcodeScan>(notifier_r_, "ScanerR");
    manager_r_ = std::make_shared<CoderManager>(notifier_r_);

    // 高反
    reflector_ptr_ = std::make_shared<WReflector>("HighReflector");
    reflector_check_ptr_ = ReflectorChecker::getInstance();
    reflector_check_ptr_->copyFrom(reflector_ptr_->getReflectors());

    // 想高反判断部分注册外参
    reflector_check_ptr_->setSensorMatrix4d("lidar_2",
                                            lidar_2_ptr_->getMatrixFromLidar());
    reflector_check_ptr_->setSensorMatrix4d("lidar_4",
                                            lidar_4_ptr_->getMatrixFromLidar());

    whileSpinPushBack(bind(&WBase::spin, lidar_2_ptr_));
    whileSpinPushBack(bind(&WBase::spin, lidar_4_ptr_));
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));
    whileSpinPushBack(bind(&WBase::spin, transfer_ptr_));
    whileSpinPushBack(bind(&WBase::spin, liftdoor_ptr_));
    whileSpinPushBack(bind(&WBase::spin, barcode_scaner_l_ptr_));
    whileSpinPushBack(bind(&WBase::spin, barcode_scaner_m_ptr_));
    whileSpinPushBack(bind(&WBase::spin, barcode_scaner_r_ptr_));

    // creat publish
    ecal_ptr_->addEcal(lidar_2_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_webots_topic.c_str());
    ecal_ptr_->addEcal(lidar_2_webots_base_topic.c_str());
    ecal_ptr_->addEcal(lidar_4_webots_base_topic.c_str());
    ecal_ptr_->addEcal("CodeScanner/read_l");
    ecal_ptr_->addEcal("CodeScanner/read_m");
    ecal_ptr_->addEcal("CodeScanner/read_r");

    // creat subscribe
    ecal_ptr_->addEcal("webot/pose",
                       std::bind(&AGVController::poseCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("webot/transfer",
                       std::bind(&AGVController::transferCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("CodeScanner/write_l",
                       std::bind(&AGVController::onCoderLScannerMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    ecal_ptr_->addEcal("CodeScanner/write_m",
                       std::bind(&AGVController::onCoderMScannerMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("CodeScanner/write_r",
                       std::bind(&AGVController::onCoderRScannerMsg, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // 创建线程
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_2_report", std::bind(&AGVController::Slam1ReportSpin, this)));
    m_thread_.insert(std::pair<std::string, std::thread>(
        "lidar_4_report", std::bind(&AGVController::Slam2ReportSpin, this)));
    // TODO: add more scaner thread here, l, r, m
    m_thread_.insert(std::pair<std::string, std::thread>(
        "codeScanL", std::bind(&AGVController::CoderLReportSpin, this)));

    m_thread_.insert(std::pair<std::string, std::thread>(
        "codeScanM", std::bind(&AGVController::CoderMReportSpin, this)));

    m_thread_.insert(std::pair<std::string, std::thread>(
        "codeScanR", std::bind(&AGVController::CoderRReportSpin, this)));
}

AGVController::~AGVController() {
    manager_l_->stop();
    manager_m_->stop();
    manager_r_->stop();
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

void AGVController::onCoderLScannerMsg(const char *topic_name,
                                       const eCAL::SReceiveCallbackData *data) {
    char CMD[100];
    memcpy(CMD, data->buf, data->size);
    if(data->size <= 100){
        CMD[data->size] = 0x00;
    }else{
        LOG_WARN("%s --> bad length = [%d]", __FUNCTION__, data->size);
        CMD[99] = 0x00; 
    }
    LOG_INFO("%s --> origin cmd = %s", __FUNCTION__, CMD);
    if (strcmp(CMD, "Start") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_l_ptr_->scanEnableSet(true);
        manager_l_->start_scan();
    } else if (strcmp(CMD, "Stop") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_l_ptr_->scanEnableSet(false);
        manager_l_->stop_scan();
    }
}

void AGVController::onCoderMScannerMsg(const char *topic_name,
                                       const eCAL::SReceiveCallbackData *data) {
    char CMD[100];
    memcpy(CMD, data->buf, data->size);
    if(data->size <= 100){
        CMD[data->size] = 0x00;
    }else{
        LOG_WARN("%s --> bad length = [%d]", __FUNCTION__, data->size);
        CMD[99] = 0x00; 
    }
    LOG_INFO("%s --> origin cmd = %s", __FUNCTION__, CMD);
    if (strcmp(CMD, "Start") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_m_ptr_->scanEnableSet(true);
        manager_m_->start_scan();
    } else if (strcmp(CMD, "Stop") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_m_ptr_->scanEnableSet(false);
        manager_m_->stop_scan();
    }
}

void AGVController::onCoderRScannerMsg(const char *topic_name,
                                       const eCAL::SReceiveCallbackData *data) {
    char CMD[100];
    memcpy(CMD, data->buf, data->size);
    if(data->size <= 100){
        CMD[data->size] = 0x00;
    }else{
        LOG_WARN("%s --> bad length = [%d]", __FUNCTION__, data->size);
        CMD[99] = 0x00; 
    }
    LOG_INFO("%s --> origin cmd = %s", __FUNCTION__, CMD);
    if (strcmp(CMD, "Start") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_r_ptr_->scanEnableSet(true);
        manager_r_->start_scan();
    } else if (strcmp(CMD, "Stop") == 0) {
        LOG_INFO("%s --> cmd = [%s]", __FUNCTION__, CMD);
        barcode_scaner_r_ptr_->scanEnableSet(false);
        manager_r_->stop_scan();
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

void AGVController::CoderLReportSpin() {
    LOG_INFO("Code scanner 1 online...");
    int ret = 0;
    char code_str[1024];
    int counter = 0;
    while (!webotsExited_) {
        // wait next timer...
        // ignore incoming start, when scanner is running
        ret = manager_l_->standBy();
        if (ret != 0) {
            LOG_DEBUG("%s --> standBy ret = %d\n", __FUNCTION__, ret);
            continue;
        }
        while (!webotsExited_) {
            ret = manager_l_->run();
            if (ret == 0) {
                const char *code = barcode_scaner_l_ptr_->getQRCode();
                if (strlen(code) == 0) {
                    sprintf(code_str, "%s", "NoRead");
                } else {
                    sprintf(code_str, "%s", code);
                }
                LOG_INFO("%s --> code = [%s]", __FUNCTION__, code_str);
                ecal_ptr_->send("CodeScanner/read_l",
                                (const uint8_t *) code_str, strlen(code_str));
            } else {
                break;
            }
        }
    }
    LOG_INFO("Code scanner 1 close offline...");
}

void AGVController::CoderMReportSpin() {
    LOG_INFO("Code scanner 2 online...");
    int ret = 0;
    char code_str[1024];
    int counter = 0;
    while (!webotsExited_) {
        // wait next timer...
        // ignore incoming start, when scanner is running
        ret = manager_m_->standBy();
        if (ret != 0) {
            LOG_DEBUG("%s --> standBy ret = %d\n", __FUNCTION__, ret);
            continue;
        }
        while (!webotsExited_) {
            ret = manager_m_->run();
            if (ret == 0) {
                const char *code = barcode_scaner_m_ptr_->getQRCode();
                if (strlen(code) == 0) {
                    sprintf(code_str, "%s", "NoRead");
                } else {
                    sprintf(code_str, "%s", code);
                }
                LOG_INFO("%s --> code = [%s]", __FUNCTION__, code_str);
                ecal_ptr_->send("CodeScanner/read_m",
                                (const uint8_t *) code_str, strlen(code_str));
            } else {
                break;
            }
        }
    }
    LOG_INFO("Code scanner 2 close offline...");
}

void AGVController::CoderRReportSpin() {
    LOG_INFO("Code scanner 3 online...");
    int ret = 0;
    char code_str[1024];
    int counter = 0;
    while (!webotsExited_) {
        // wait next timer...
        // ignore incoming start, when scanner is running
        ret = manager_r_->standBy();
        if (ret != 0) {
            LOG_DEBUG("%s --> standBy ret = %d\n", __FUNCTION__, ret);
            continue;
        }
        while (!webotsExited_) {
            ret = manager_r_->run();
            if (ret == 0) {
                const char *code = barcode_scaner_r_ptr_->getQRCode();
                if (strlen(code) == 0) {
                    sprintf(code_str, "%s", "NoRead");
                } else {
                    sprintf(code_str, "%s", code);
                }
                LOG_INFO("%s --> code = [%s]", __FUNCTION__, code_str);
                ecal_ptr_->send("CodeScanner/read_r",
                                (const uint8_t *) code_str, strlen(code_str));
            } else {
                break;
            }
        }
    }
    LOG_INFO("Code scanner 3 close offline...");
}