/**
 * @file P_master.h
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

#include "sim_data_flow/P_msg.pb.h"
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
    std::shared_ptr<WWheel> stree_ptr_;
    std::shared_ptr<WWheel> l_ptr_;
    std::shared_ptr<WWheel> r_ptr_;
    std::shared_ptr<WFork> fork_ptr_;
    std::shared_ptr<WPose> pose_ptr_;
    std::shared_ptr<WLidar> lidar_pose_ptr_;
    std::shared_ptr<WCollision> collision_ptr_;
    std::shared_ptr<WTransfer> transfer_ptr_;
    std::shared_ptr<WLiftDoor> liftdoor_ptr_;
    std::shared_ptr<photoelectric> hswitchL_ptr_;
    std::shared_ptr<photoelectric> hswitchR_ptr_;
    //  std::shared_ptr<manchanical> hswitchL_ptr_;
    //  std::shared_ptr<manchanical> hswitchR_ptr_;
    std::shared_ptr<manchanical> vswitchL_ptr_;
    std::shared_ptr<manchanical> vswitchR_ptr_;
    // std::shared_ptr<OrgDistance> org_ptr_;
    std::shared_ptr<VoyerBeltManager> manager_ptr_;

   public:
    void subPMsgCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data);
    void onConveyorStateMsg(const char *topic_name, const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim