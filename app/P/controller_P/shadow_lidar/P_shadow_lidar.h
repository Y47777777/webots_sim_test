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
#include "webots_device/w_reflector.h"

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

    void BpReportSpin();
    void Mid360ReportSpin();
    void Mid360TwoReportSpin();

    bool sendPointCloud(std::string topic,std::shared_ptr<WLidar> lidar_ptr);
    

   private:
    std::shared_ptr<WLidar> BP_ptr_;
    std::shared_ptr<WLidar> mid360_ptr_;
    std::shared_ptr<WLidar> mid360Two_ptr_;

    std::shared_ptr<WPose> pose_ptr_;
    std::shared_ptr<WReflector> reflector_ptr_;
    std::shared_ptr<ReflectorChecker> reflector_check_ptr_;

   public:
    void transferCallBack(const char *topic_name, const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim