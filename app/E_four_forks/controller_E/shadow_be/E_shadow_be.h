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

#include "controller/base_ctrl.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_pose.h"
#include "webots_device/w_reflector.h"
#include "webots_device/w_transfer.h"
#include "webots_device/w_collision.h"

namespace VNSim {

class AGVController : public BaseController {
   public:
    AGVController();
    ~AGVController(){};

    // Manual
    virtual void manualSetState(const std::map<std::string, double> &msg){};
    virtual void manualGetState(std::map<std::string, double> &msg){};

   protected:
    // task
    void whileSpin();

    void HapReportSpin();

    bool sendPointCloud(std::string topic, std::shared_ptr<WLidar> lidar_ptr);

   private:
    std::shared_ptr<WLidar> HAP_ptr_;

    std::shared_ptr<WPose> pose_ptr_;
    std::shared_ptr<WTransfer> transfer_ptr_;
    std::shared_ptr<WReflector> reflector_ptr_;
    std::shared_ptr<WCollision> collision_ptr_;
    std::shared_ptr<ReflectorChecker> reflector_check_ptr_;
    

   private:
    void poseCallBack(const char *topic_name,
                      const eCAL::SReceiveCallbackData *data);

    void transferCallBack(const char *topic_name,
                          const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim