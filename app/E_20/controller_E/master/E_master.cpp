/**
 * @file E_master.cpp
 * @author xyjie (xyjie@visionnav.com)
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

#include "sim_data_flow/E20_msg.pb.h"
#include "sim_data_flow/point_cloud.pb.h"
#include "sim_data_flow/pose.pb.h"

#include "geometry/geometry.h"
#include "time/time.h"

#include "logvn/logvn.h"

#include "E_master.h"

using namespace VNSim;
using namespace webots;

const double WHEELBASE = 1.7;     // 前后轮间距
const double FRONT_TREAD = 0.78;  // 定向轮间距
const double REAR_TREAD = 1.1;    // 驱动轮间距
const double FOLLOW_START = 0.4;
const double FROK_MIN_SPAC = 1.0;  //外叉内间距最小值 FIXME: E20
const double CLAMP_FACTOR = 15;

// 计算tan角度
inline double CalcTan(double len, double yaw) {
    return len * std::tan(yaw);
}

// 计算角度
inline double CalcAngle(double tangent) {
    return std::atan(tangent);
}

// 函数1：计算转向角和驱动轮速度
void computeSteeringAndDrive(double v, double yaw, double &driver_l,
                             double &driver_r) {
    if (fabs(0 - yaw) < 0.001) {
        driver_l = v;
        driver_r = v;
    } else {
        // 计算转向角度
        double back_radius_c = WHEELBASE / tan(yaw);
        double front_radius_c = WHEELBASE / sin(yaw);

        double back_radius_r = back_radius_c + (REAR_TREAD / 2);
        double back_radius_l = back_radius_c - (REAR_TREAD / 2);

        double w = v / front_radius_c;

        driver_l = w * back_radius_l;
        driver_r = w * back_radius_r;
    }
}

// 函数2：计算内侧驱动轮的转速
double computeInsideWheelSpeed(double steer, double outside_wheel_speed) {
    if (fabs(steer) < 0.001) {
        return outside_wheel_speed;
    }

    double back_radius_c = steer;

    // std::cout << "11 back_radius_c1 = " << back_radius_c1
    //           << "  back_radius_c2 = " << back_radius_c2
    //           << "  back_radius_c = " << back_radius_c << std::endl;

    double back_radius_l = back_radius_c - (REAR_TREAD / 2);
    double back_radius_r = back_radius_c + (REAR_TREAD / 2);

    // std::cout << "22 back_radius_l = " << back_radius_l
    //           << "  back_radius_r = " << back_radius_r << std::endl;

    double inner_speed = 0;
    if (steer > 0) {
        double w = outside_wheel_speed / back_radius_r;
        inner_speed = (w * back_radius_l);

    } else {
        double w = outside_wheel_speed / back_radius_l;
        inner_speed = (w * back_radius_r);
    }

    return inner_speed;
}

// TODO: 构造的位置要想想
std::shared_ptr<Timer> Timer::instance_ptr_ = nullptr;
std::shared_ptr<EcalWrapper> EcalWrapper::instance_ptr_ = nullptr;
std::shared_ptr<ReflectorChecker> ReflectorChecker::instance_ptr_ = nullptr;

AGVController::AGVController() : BaseController("webots_master") {
    imu_ptr_ = std::make_shared<WImu>("inertial unit", "gyro", "accelerometer");

    fork_ptr_ = std::make_shared<WFork>("fork height motor", "ForkZAxis",
                                        "fork height");
    // forkY_ptr_ = std::make_shared<WFork>("YMotor", "ShadowY","","",false,0.0, false, 100, true);
    forkP_ptr_ = std::make_shared<WFork>("PMotor", "ForkPAxis");
    forkCLF1_ptr_ = std::make_shared<WFork>("CLMotor", "LF1", "", "", false,
                                            0.0, true, 200, true, 10);
    forkCRF1_ptr_ = std::make_shared<WFork>("CRMotor", "RF1", "", "", false,
                                            0.0, false, 200);

    stree_ptr_ =
        std::make_shared<WWheel>("", "SteerWheel", "SteerSolid", "FLWheel");

    l_ptr_ = std::make_shared<WWheel>("FL", "", "", "RS", "");
    r_ptr_ = std::make_shared<WWheel>("FR", "", "", "RS", "");
    pose_ptr_ = std::make_shared<WPose>("RobotNode");
    lidar_pose_ptr_ = std::make_shared<WLidar>("lidar_0", 100, false);
    transfer_ptr_ = std::make_shared<WTransfer>();
    collision_ptr_ = std::make_shared<WCollision>(false);
    liftdoor_ptr_ = std::make_shared<WLiftDoor>(false);

    v_while_spin_.push_back(bind(&WBase::spin, stree_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, l_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, r_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, fork_ptr_));
    // v_while_spin_.push_back(bind(&WBase::spin, forkY_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, forkP_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, forkCLF1_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, forkCRF1_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, imu_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, pose_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, transfer_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, collision_ptr_));
    v_while_spin_.push_back(bind(&WBase::spin, liftdoor_ptr_));

    // pub
    ecal_ptr_->addEcal("webot/E_msg");
    ecal_ptr_->addEcal("webot/transfer");
    ecal_ptr_->addEcal("webot/pose");
    ecal_ptr_->addEcal("webot/liftdoor");

    // sub
    ecal_ptr_->addEcal("svc/E_msg",
                       std::bind(&AGVController::subEMsgCallBack, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

static double forkY_speed = 0;

void AGVController::manualSetState(const std::map<std::string, double> &msg) {
    static double steer_speed = 0;
    static double steer_yaw = 0;
    static double fork_speed = 0;
    static double forkP_speed = 0;
    static double forkC_speed = 0;
    if (msg.find("refresh_world") != msg.end()) {
        transfer_ptr_->noticeAll();
    }
    if (isManual_) {
        steer_speed = msg.at("steer_speed");
        steer_yaw = msg.at("steer_yaw");
        fork_speed = msg.at("fork_speed");
        forkY_speed = msg.at("forkY_speed");
        forkP_speed = msg.at("forkP_speed");
        forkC_speed = msg.at("forkC_speed");

        double r_v, l_v;

        computeSteeringAndDrive(steer_speed, steer_yaw, l_v, r_v);

        // double l_speed = 0;
        // double r_speed = 0;
        // std::cout << "v: " << steer_speed << " yaw " << steer_yaw <<
        // std::endl;

        // std::cout << "1 l_yaw: " << l_yaw << " r_yaw " << r_yaw << "l_v" <<
        // l_v
        //           << "r_v" << r_v << std::endl;

        // if ((steer_yaw < 0)) {
        //     r_v = computeInsideWheelSpeed(l_yaw, r_yaw, l_v);
        //     // r_speed = r_v;
        // } else {
        //     l_v = computeInsideWheelSpeed(l_yaw, r_yaw, r_v);
        //     // l_speed = l_v;
        // }

        stree_ptr_->setYaw(steer_yaw);
        l_ptr_->setVelocity(l_v);
        r_ptr_->setVelocity(r_v);

        fork_ptr_->setVelocityAll(fork_speed);
        forkP_ptr_->setVelocityAll(forkP_speed);
        double CL_T = forkC_speed - forkY_speed;
        double CR_T = forkC_speed + forkY_speed;
        if(determineForceCAxisReset(CL_T, CR_T)){
            // CL_T = forkC_speed;
            // CR_T = forkC_speed;
            CL_T = 0;
            CR_T = 0;
        }
        forkCLF1_ptr_->setVelocityAll(CL_T);
        forkCRF1_ptr_->setVelocityAll(CR_T);
    }
}

void AGVController::manualGetState(std::map<std::string, double> &msg) {
    msg["steer_speed_r"] = r_ptr_->getSpeed();
    msg["steer_speed"] = l_ptr_->getSpeed();
    msg["steer_yaw"] = stree_ptr_->getMotorYaw();
    msg["fork_speed"] = fork_ptr_->getVelocityValue();
    msg["forkY_speed"] = forkY_speed;
    msg["forkP_speed"] = forkP_ptr_->getVelocityValue();
    msg["forkC_speed"] = forkCLF1_ptr_->getVelocityValue();
    msg["fork_height"] = fork_ptr_->getSenosorValue();
    msg["forkY_height"] = 0;
    msg["forkP_height"] = forkP_ptr_->getSenosorValue();
    msg["forkC_height"] = forkCLF1_ptr_->getSenosorValue();
    msg["real_speed"] = forkCRF1_ptr_->getSenosorValue();
}

void AGVController::whileSpin() {
    /* 主循环 在super_->step()后*/
    // determine whether we should reset C speed to 0
    determineForceCAxisReset();

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

bool AGVController::determineForceCAxisReset(double CL_T, double CR_T){
    bool flag = false;
    int clf1_bound = forkCLF1_ptr_->isOnBoundary();
    int crf1_bound = forkCRF1_ptr_->isOnBoundary();
    if((CL_T * CR_T) < 0){
        if((CL_T > 0) && ((clf1_bound == -1) || (crf1_bound == -2))){
            // Y left
            LOG_INFO("%s1 --> CL_T = %lf, CR_T = %lf, forkCLF1_ptr_->isOnBoundary() = %d, forkCRF1_ptr_->isOnBoundary() = %d", __FUNCTION__, CL_T, CR_T, 
            clf1_bound, crf1_bound);
            flag = true;
        }else if((CL_T < 0) && ((clf1_bound == -2) || (crf1_bound == -1))){
            // Y right
            LOG_INFO("%s1 --> CL_T = %lf, CR_T = %lf, forkCLF1_ptr_->isOnBoundary() = %d, forkCRF1_ptr_->isOnBoundary() = %d", __FUNCTION__, CL_T, CR_T, 
            clf1_bound, crf1_bound);
            flag = true;
        }
    }
    return flag;
}

void AGVController::determineForceCAxisReset(){
    double CL_T = forkCLF1_ptr_->getVelocityValue();
    double CR_T = forkCRF1_ptr_->getVelocityValue();
    bool flag = false;
    int clf1_bound = forkCLF1_ptr_->isOnBoundary();
    int crf1_bound = forkCRF1_ptr_->isOnBoundary();
    if((CL_T * CR_T) < 0){
        if((CL_T > 0) && ((clf1_bound == -1) || (crf1_bound == -2))){
            // Y left
            flag = true;
            LOG_DEBUG("%s2 --> CL_T = %lf, CR_T = %lf, forkCLF1_ptr_->isOnBoundary() = %d, forkCRF1_ptr_->isOnBoundary() = %d", __FUNCTION__, CL_T, CR_T, 
            clf1_bound, crf1_bound);
        }else if((CL_T < 0) && ((clf1_bound == -2) || (crf1_bound == -1))){
            // Y right
            flag = true;
            LOG_DEBUG("%s2 --> CL_T = %lf, CR_T = %lf, forkCLF1_ptr_->isOnBoundary() = %d, forkCRF1_ptr_->isOnBoundary() = %d", __FUNCTION__, CL_T, CR_T, 
            clf1_bound, crf1_bound);
        }
    }
    forkCLF1_ptr_->forceReset(flag);
    forkCRF1_ptr_->forceReset(flag);
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

void AGVController::subEMsgCallBack(const char *topic_name,
                                    const eCAL::SReceiveCallbackData *data) {
    if (!isManual_) {
        sim_data_flow::E20MsgDown payload;
        payload.ParseFromArray(data->buf, data->size);

        double steeting_theta = payload.steering_theta();
        double speed = payload.steering_speed();
        double l_speed = 0;
        double r_speed = 0;

        if ((steeting_theta > 0)) {
            l_speed = computeInsideWheelSpeed(steeting_theta, speed);
            r_speed = speed;
        } else {
            r_speed = computeInsideWheelSpeed(steeting_theta, speed);
            l_speed = speed;
        }
        // LOG_INFO("L l_yaw: %lf, r_yaw = %lf , l_v = %lf r_v = %lf", l_theta,
        //          r_theta, l_speed, r_speed);

        stree_ptr_->setYaw(steeting_theta);
        l_ptr_->setVelocity(l_speed);
        r_ptr_->setVelocity(r_speed);

        fork_ptr_->setVelocity(payload.forkspeedz());
        forkP_ptr_->setVelocity(payload.forkspeedp());
        double CL_T = payload.forkspeedc() + payload.forkspeedy();
        double CR_T = payload.forkspeedc() - payload.forkspeedy();
        if(determineForceCAxisReset(CL_T, CR_T)){
            // CL_T = payload.forkspeedc();
            // CR_T = payload.forkspeedc();
            CL_T = 0;
            CR_T = 0;
        }
        forkCLF1_ptr_->setVelocity(CL_T);
        forkCRF1_ptr_->setVelocity(CR_T);
    }
}

void AGVController::pubSerialSpin() {
    sim_data_flow::E20MsgUp payload;
    payload.set_timestamp(time_stamp_);
    payload.set_forkposez(fork_ptr_->getSenosorValue());
    payload.set_forkposey(0);
    payload.set_forkposep(forkP_ptr_->getSenosorValue());
    payload.set_forkposecl(forkCLF1_ptr_->getSenosorValue() +
                           FROK_MIN_SPAC / 2);
    payload.set_forkposecr(forkCRF1_ptr_->getSenosorValue() +
                           FROK_MIN_SPAC / 2);
    double origin_force = forkCLF1_ptr_->getForce() * CLAMP_FACTOR;
    payload.set_clamppressure(origin_force);
    //std::cout << "origin_force = " << origin_force << std::endl;
    payload.set_steering_theta(stree_ptr_->getMotorYaw());

    payload.set_l_wheel(l_ptr_->getWheelArcLength());
    payload.set_r_wheel(r_ptr_->getWheelArcLength());

    payload.set_gyroscope(imu_ptr_->getInertialYaw());

    foxglove::Imu *imu = payload.mutable_imu();
    imu->mutable_angular_velocity()->CopyFrom(imu_ptr_->getGyroValue());
    imu->mutable_linear_acceleration()->CopyFrom(imu_ptr_->getAccValue());

    // double *rotation = pose_ptr_->getRotaion();
    // LOG_INFO("imu: %.2f, robot: %.2f", imu_ptr_->getInertialYaw(),
    // rotation[3]);
    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/E_msg", buf, payload.ByteSize());
}

void VNSim::AGVController::pubLiftDoorTag() {
    sim_data_flow::MTransfer payload;
    liftdoor_ptr_->getTag(payload);

    uint8_t buf[payload.ByteSize()];
    payload.SerializePartialToArray(buf, payload.ByteSize());
    ecal_ptr_->send("webot/liftdoor", buf, payload.ByteSize());
}