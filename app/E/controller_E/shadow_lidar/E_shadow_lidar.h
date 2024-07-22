/**
 * @file E_shadow_lidar.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <thread>

#include "controller/base_lidar.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_pose.h"
#include "webots_device/w_reflector.h"
#include "webots_device/w_transfer.h"
#include "webots_device/w_collision.h"
#include "webots_device/w_liftdoor.h"

namespace VNSim {

class AGVController : public BaseLidarControl {
   public:
    AGVController();
    ~AGVController(){};

    // Manual
    virtual void manualSetState(const std::map<std::string, double> &msg){};
    virtual void manualGetState(std::map<std::string, double> &msg){};

   protected:
    // task
    void whileSpin();

    void Slam1ReportSpin();
    void Slam2ReportSpin();

   private:
    std::shared_ptr<WLidar> slam_1_ptr_;
    std::shared_ptr<WLidar> slam_2_ptr_;

    std::shared_ptr<WPose> pose_ptr_;
    std::shared_ptr<WTransfer> transfer_ptr_;
    std::shared_ptr<WLiftDoor> liftdoor_ptr_;
    std::shared_ptr<WReflector> reflector_ptr_;
    std::shared_ptr<WCollision> collision_ptr_;
    std::shared_ptr<ReflectorChecker> reflector_check_ptr_;

   private:
    void poseCallBack(const char *topic_name,
                      const eCAL::SReceiveCallbackData *data);

    void transferCallBack(const char *topic_name,
                          const eCAL::SReceiveCallbackData *data);

    void liftdoorCallBack(const char *topic_name,
                          const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim