/**
 * @file P_master.cpp
 * @author weijchen (weijchen@visionnav.com)
 * @brief 主控 ctrl
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
#include "sim_data_flow/voyerbelt_msg.pb.h"

#include "geometry/geometry.h"
#include "time/time.h"

#include "logvn/logvn.h"

#include "P_master.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

AGVController::AGVController() : BaseController("webots_master") {
    manager_ptr_ = std::make_shared<VoyerBeltManager>();
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    fork_ptr_ = std::make_shared<WFork>("fork height motor");
    stree_ptr_ = std::make_shared<WWheel>("FL", "SteerWheel", "SteerSolid", "FLWheel");
    l_ptr_ = std::make_shared<WWheel>("", "", "", "RS", "BRPS");
    r_ptr_ = std::make_shared<WWheel>("", "", "", "LS", "BLPS");
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    lidar_pose_ptr_ = std::make_shared<WLidar>("lidar_0", nullptr, 100, false);
    transfer_ptr_ = std::make_shared<WTransfer>(manager_ptr_);
    collision_ptr_ = std::make_shared<WCollision>(false);
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(false);
    hswitchL_ptr_ = std::make_shared<photoelectric>("HL", "HSwitchL");
    hswitchR_ptr_ = std::make_shared<photoelectric>("HR", "HSwitchR");
    vswitchL_ptr_ = std::make_shared<manchanical>("VL", "VSwitchL");
    vswitchR_ptr_ = std::make_shared<manchanical>("VR", "VSwitchR");

    whileSpinPushBack((stree_ptr_));
    whileSpinPushBack((l_ptr_));
    whileSpinPushBack((r_ptr_));
    whileSpinPushBack((fork_ptr_));
    whileSpinPushBack((imu_ptr_));
    whileSpinPushBack((transfer_ptr_));
    whileSpinPushBack((collision_ptr_));

    whileSpinPushBack((hswitchL_ptr_));
    whileSpinPushBack((hswitchR_ptr_));
    whileSpinPushBack((vswitchL_ptr_));
    whileSpinPushBack((vswitchR_ptr_));
    whileSpinPushBack(bind(&WBase::spin, liftdoor_ptr_));
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));

    // 获取当前滚筒线...
    std::list<std::string> convoyer_list;
    manager_ptr_->getServerList(convoyer_list);

    // pub
    ecal_ptr_->addEcal("webot/P_msg");
    ecal_ptr_->addEcal("webot/transfer");
    ecal_ptr_->addEcal("webot/pose");
    ecal_ptr_->addEcal("webot/liftdoor");
    for (auto const &it : convoyer_list) {
        LOG_INFO("E20 cmd topic = %s, state stopic = %s", std::string(it + "/CmdInfo").c_str(),
                 std::string(it + "/Goods/Events").c_str());
        ecal_ptr_->addEcal(std::string(it + "/CmdInfo").c_str());
        ecal_ptr_->addEcal(
            std::string(it + "/Goods/Events").c_str(),
            std::bind(&AGVController::onConveyorStateMsg, this, std::placeholders::_1, std::placeholders::_2));
    }

    // sub
    ecal_ptr_->addEcal("svc/P_msg",
                       std::bind(&AGVController::subPMsgCallBack, this, std::placeholders::_1, std::placeholders::_2));
    ecal_ptr_->addEcal("Goods/Events", std::bind(&AGVController::onConveyorStateMsg, this, std::placeholders::_1,
                                                 std::placeholders::_2));
}

void AGVController::manualSetState(const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double fork_speed = 0;
    if (msg.find("refresh_world") != msg.end()) {
        transfer_ptr_->noticeAll();
    }
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        fork_speed = msg.at("fork_speed");

        stree_ptr_->setSpeed(steer_speed, steer_yaw);
        fork_ptr_->setVelocityAll(fork_speed);
    }
}

void AGVController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["real_speed"] = 0;
    // TODO: fork_speed real_speed
}

void AGVController::whileSpin() {
    /* 主循环 在super_->step()后*/
    // 发送至svc
    pubSerialSpin();

    // 移动感知激光
    movePerLidarSpin();

    // 发送至shadow
    pubRobotPoseSpin();

    // 休眠一下
    Timer::getInstance()->sleep<microseconds>(10);
    pubTransferSpin();

    pubLiftDoorTag();
}

void AGVController::movePerLidarSpin() {
    double fork_z = fork_ptr_->getSenosorValue();  // 米制

    // TODO: 激光随动
    lidar_pose_ptr_->moveLidar(fork_z);
    // LOG_INFO("fork :%.2f", fork_z);
}

void AGVController::pubTransferSpin() {
    sim_data_flow::MTransfer payload;
    transfer_ptr_->getTransfer(payload);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/transfer", buf, payload.ByteSize());
}

void AGVController::pubRobotPoseSpin() {
    double *tran = pose_ptr_->getTransfer();
    double *rotation = pose_ptr_->getRotaion();

    sim_data_flow::Pose pose;
    pose.mutable_position()->set_x(tran[0]);
    pose.mutable_position()->set_y(tran[1]);
    pose.mutable_position()->set_z(tran[2]);

    pose.mutable_orientation()->set_x(rotation[0]);
    pose.mutable_orientation()->set_y(rotation[1]);
    pose.mutable_orientation()->set_z(rotation[2]);
    pose.mutable_orientation()->set_w(rotation[3]);
    pose.set_timestamp(time_stamp_);

    uint8_t buf[pose.ByteSize()];
    pose.SerializePartialToArray(buf, pose.ByteSize());
    ecal_ptr_->send("webot/pose", buf, pose.ByteSize());
}

void AGVController::subPMsgCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::PMsgDown payload;
        payload.ParseFromArray(data->buf, data->size);

        stree_ptr_->setSpeed(payload.steering_speed(), payload.steering_theta());
        fork_ptr_->setVelocity(payload.forkspeedz());
    }
}

void AGVController::pubSerialSpin() {
    sim_data_flow::PMsgUp payload;

    payload.set_timestamp(time_stamp_);
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_steerposition(stree_ptr_->getSenosorValue());

    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());

    payload.set_steering_theta(stree_ptr_->getMotorYaw());

    payload.set_gyroscope(imu_ptr_->getInertialYaw());
    payload.set_hswitchl(hswitchL_ptr_->getValue());
    payload.set_hswitchr(hswitchR_ptr_->getValue());
    payload.set_vswitchl(vswitchL_ptr_->getValue());
    payload.set_vswitchr(vswitchR_ptr_->getValue());

    // double *rotation = pose_ptr_->getRotaion();
    // LOG_INFO("imu: %.2f, robot: %.2f", imu_ptr_->getInertialYaw(),
    // rotation[3]);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/P_msg", buf, payload.ByteSize());
}

void VNSim::AGVController::pubLiftDoorTag() {
    sim_data_flow::MTransfer payload;
    liftdoor_ptr_->getTag(payload);

    if (payload.ByteSize() != 0) {
        uint8_t buf[payload.ByteSize()];

        payload.SerializePartialToArray(buf, payload.ByteSize());
        ecal_ptr_->send("webot/liftdoor", buf, payload.ByteSize());
    }
}
void AGVController::onConveyorKeyboardMsg(const std::map<std::string, std::string> &msg){
    // TODO: add state consideration....
    std::string function = msg.at("function");
    // std::cout << "belt发过来的function: " << function << msg.at("belt") << std::endl;
    if(function == "0")
        manager_ptr_->addRandomPallet(msg.at("belt"),true,false);
    else
        manager_ptr_->addRemovePallet(msg.at("belt"),true);
}

void AGVController::onConveyorStateMsg(const char *topic_name,
                         const eCAL::SReceiveCallbackData *data){
    sim_data_flow::StateInfo payload;
    payload.ParseFromArray(data->buf, data->size);
    int ret = 1;
    if(payload.state() == 0){
        // belt is full
        // TODO: may be add more topic
        ret = manager_ptr_->addRemovePallet(payload.belt_name(), false, payload.who());
    }else if(payload.state() == 1){
        // belt is empty
        ret = manager_ptr_->addRandomPallet(payload.belt_name(), false, true, payload.who());
    }
    if(ret == 1){
        // convoyer belt initialization is over
        std::string res_topic = "";
        res_topic = payload.belt_name() + "/CmdInfo";
        LOG_INFO("%s --> try send stop cmd to %s", __FUNCTION__, res_topic.c_str());
        //printf("%s --> try send stop cmd to %s\n", __FUNCTION__, res_topic.c_str());
        sim_data_flow::CmdInfo res_payload;
        res_payload.set_cmd(0);
        uint8_t buf[res_payload.ByteSize()];
        res_payload.SerializePartialToArray(buf, res_payload.ByteSize());
        ecal_ptr_->send(res_topic.c_str(), buf, res_payload.ByteSize());
    }
}