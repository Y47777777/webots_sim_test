/**
 * @file R_master.h
 * @author weijchen (weijchen@visionnav.com)
 * @brief 主控 ctrl.h
 * @version 2.0
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <thread>

#include "sim_data_flow/R_msg.pb.h"
#include "controller/base_ctrl.h"
#include "webots_device/w_fork.h"
#include "webots_device/w_wheel.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_imu.h"
#include "webots_device/w_pose.h"
#include "webots_device/w_transfer.h"
#include "webots_device/w_liftdoor.h"
#include "webots_device/w_switch.h"
#include "webots_device/w_collision.h"
#include "webots_device/w_stree_wheel.h"

namespace VNSim {

class AGVController : public BaseController {
   public:
    AGVController();
    ~AGVController(){};

    // Manual
    virtual void manualSetState(const std::map<std::string, double> &msg);
    virtual void manualGetState(std::map<std::string, double> &msg);
    virtual void onConveyorKeyboardMsg(const std::map<std::string, std::string> &msg) override;

   protected:
    // task
    void whileSpin();
    void pubSerialSpin();
    void pubTransferSpin();
    void pubRobotPoseSpin();
    void movePerLidarSpin();
    void pubLiftDoorTag();

   private:
    std::shared_ptr<WImu> imu_ptr_;
    std::shared_ptr<SWWheel> stree_ptr_;  //舵轮
    std::shared_ptr<WWheel> l_ptr_;       //从动左轮
    std::shared_ptr<WWheel> r_ptr_;       //从动右轮
    std::shared_ptr<WFork> forkP_ptr_;    //货叉俯仰
    std::shared_ptr<WFork> forkZ_ptr_;    //向上平移
    std::shared_ptr<WFork> forkX_ptr_;    // X方向前后平移
    std::shared_ptr<WFork> forkY_ptr_;    // Y方向左右平移
    std::shared_ptr<WPose> pose_ptr_;
    std::shared_ptr<WLidar> lidar_pose_ptr_;
    std::shared_ptr<WCollision> collision_ptr_;
    std::shared_ptr<WTransfer> transfer_ptr_;
    std::shared_ptr<WLiftDoor> liftdoor_ptr_;
    std::shared_ptr<photoelectric> hswitchL_ptr_;        //水平物料到位传感器左叉
    std::shared_ptr<photoelectric> hswitchR_ptr_;        //水平物料到位传感器右叉
    std::shared_ptr<manchanical> vswitchL_ptr_;          //垂直物料到位传感器左叉
    std::shared_ptr<manchanical> vswitchR_ptr_;          //垂直物料到位传感器右叉
    std::shared_ptr<photoelectric> safetyswitchFL_ptr_;  //左货叉安全防护
    std::shared_ptr<photoelectric> safetyswitchFR_ptr_;  //右货叉安全防护
    std::shared_ptr<VoyerBeltManager> manager_ptr_;      //滚筒线

   public:
    void subRMsgCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onConveyorStateMsg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim