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

#include "geometry/geometry.h"
#include "time/time.h"

#include "logvn/logvn.h"

#include "R_master.h"

using namespace VNSim;
using namespace webots;

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

AGVController::AGVController() : BaseController("webots_master") {

    stree_ptr_ = std::make_shared<SWWheel>("SteerRun_Motor", "SteerTurn_Motor",
                    "StreeWheelRadius", "SteerRun_Sensor", "SteerTurn_Sensor"); //驱动转向舵轮

    l_ptr_ = std::make_shared<WWheel>("", "", "L_D_SteerSolid", "LDW_Radius",
                    "L_D_Sensor");     //从动左轮

    r_ptr_ = std::make_shared<WWheel>("", "", "R_D_SteerSolid", "RDW_Radius",
                    "R_D_Sensor");     //从动右轮

    forkX_ptr_ = std::make_shared<WFork>("Linear_FB_Motor", "ALL_Linear_FB_Solid");  //X方向前后平移
    forkZ_ptr_ = std::make_shared<WFork>("Lifting_Motor", "Lifting_Component_Solid");  //向上平移
    forkP_ptr_ = std::make_shared<WFork>("Forks_Pitching_Motor", "Picking_Component_Solid");  //货叉俯仰
    forkY_ptr_ = std::make_shared<WFork>("Fork_YMove_Motor", "Forks_YMove_Solid");  //Y方向左右平移


    //世界信息控制指针
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    lidar_pose_ptr_ = std::make_shared<WLidar>("lidar_0", nullptr, 100, false);
    transfer_ptr_ = std::make_shared<WTransfer>();
    collision_ptr_ = std::make_shared<WCollision>(false);
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(false);
    hswitchL_ptr_ = std::make_shared<photoelectric>("HL", "HSwitchL");
    hswitchR_ptr_ = std::make_shared<photoelectric>("HR", "HSwitchR");
    vswitchL_ptr_ = std::make_shared<manchanical>("VL", "VSwitchL");
    vswitchR_ptr_ = std::make_shared<manchanical>("VR", "VSwitchR");

    whileSpinPushBack((stree_ptr_));
    whileSpinPushBack((l_ptr_));
    whileSpinPushBack((r_ptr_));
    whileSpinPushBack((forkX_ptr_));
    whileSpinPushBack((forkZ_ptr_));
    whileSpinPushBack((forkP_ptr_));
    whileSpinPushBack((forkY_ptr_));
    whileSpinPushBack((imu_ptr_));
    //whileSpinPushBack((pose_ptr_));
    whileSpinPushBack((transfer_ptr_));
    whileSpinPushBack((collision_ptr_));
    whileSpinPushBack((liftdoor_ptr_));
    whileSpinPushBack((hswitchL_ptr_));
    whileSpinPushBack((hswitchR_ptr_));
    whileSpinPushBack((vswitchL_ptr_));
    whileSpinPushBack((vswitchR_ptr_));
    //为了解决阻塞的问题，将pose_ptr_放到最后
    whileSpinPushBack(bind(&WBase::spin, pose_ptr_));

    // pub
    ecal_ptr_->addEcal("webot/R_msg");
    ecal_ptr_->addEcal("webot/transfer");
    ecal_ptr_->addEcal("webot/pose");
    ecal_ptr_->addEcal("webot/liftdoor");

    // sub
    ecal_ptr_->addEcal("svc/R_msg",
                       std::bind(&AGVController::subRMsgCallBack, this, std::placeholders::_1, std::placeholders::_2));
}

void AGVController::manualSetState(const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double forkX_speed = 0;
    static double forkY_speed = 0;
    static double forkZ_speed = 0;
    static double forkP_speed = 0;
    if (msg.find("refresh_world") != msg.end()) {
        transfer_ptr_->noticeAll();
    }
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        forkX_speed = msg.at("forkX_speed");
        forkY_speed = msg.at("forkY_speed");
        forkZ_speed = msg.at("fork_speed");
        forkP_speed = msg.at("forkP_speed");

        stree_ptr_->stree_run(steer_speed, steer_yaw);
        forkX_ptr_->setVelocityAll(forkX_speed);
        forkY_ptr_->setVelocityAll(forkY_speed);   
        forkZ_ptr_->setVelocityAll(forkZ_speed);
        forkP_ptr_->setVelocityAll(forkP_speed);      
    }
}

void AGVController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed"] = stree_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["forkX_speed"] = forkX_ptr_->getVelocityValue();
    msg["forkX_height"] = forkX_ptr_->getSenosorValue();
    msg["forkY_speed"] = forkY_ptr_->getVelocityValue();
    msg["forkY_height"] = forkY_ptr_->getSenosorValue();
    msg["fork_speed"] = forkZ_ptr_->getVelocityValue();    
    msg["fork_height"] = forkZ_ptr_->getSenosorValue();
    msg["forkP_speed"] = forkP_ptr_->getVelocityValue();
    msg["forkP_height"] = forkP_ptr_->getSenosorValue();
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
    double fork_z = forkZ_ptr_->getSenosorValue();  // 米制

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

void AGVController::subRMsgCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::RMsgDown payload;
        payload.ParseFromArray(data->buf, data->size);

        stree_ptr_->stree_run(payload.steering_speed(), payload.steering_theta());
        forkX_ptr_->setVelocity(payload.forkspeedx());
        forkZ_ptr_->setVelocity(payload.forkspeedz());
        forkP_ptr_->setVelocity(payload.forkspeedp());
        forkY_ptr_->setVelocity(payload.forkspeedy());
    }
}

void AGVController::pubSerialSpin() {
    sim_data_flow::RMsgUp payload;

    payload.set_timestamp(time_stamp_);
    payload.set_forkposex(forkX_ptr_->getSenosorValue());
    payload.set_forkposez(forkZ_ptr_->getSenosorValue());
    payload.set_forkposep(forkP_ptr_->getSenosorValue());
    payload.set_forkposey(forkY_ptr_->getSenosorValue());

    payload.set_steerposition(stree_ptr_->getSenosorValue());
    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());

    payload.set_steering_theta(stree_ptr_->getMotorYaw());
    payload.set_gyroscope(imu_ptr_->getInertialYaw());
    payload.set_hswitchl(hswitchL_ptr_->getValue());
    payload.set_hswitchr(hswitchR_ptr_->getValue());
    payload.set_vswitchl(vswitchL_ptr_->getValue());
    payload.set_vswitchr(vswitchR_ptr_->getValue());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getGyroValue());
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getAccValue());

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/R_msg", buf, payload.ByteSize());
}

void VNSim::AGVController::pubLiftDoorTag() {
    sim_data_flow::MTransfer payload;
    liftdoor_ptr_->getTag(payload);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/liftdoor", buf, payload.ByteSize());
}
