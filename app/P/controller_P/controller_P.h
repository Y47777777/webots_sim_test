/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 16:22:54
 * @FilePath: /webots_ctrl/app/ST/controller_ST/controller_ST.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
/*
 * @Author: weijchen weijchen@visionnav.com
 * @Date: 2024-06-06 15:18:00
 * @LastEditors: weijchen weijchen@visionnav.com
 * @LastEditTime: 2024-06-07 11:36:09
 * @FilePath: /webots_ctrl/app/ST/controller_ST/controller_ST.h
 * @Description:
 *
 * Copyright (c) 2024 by visionnav, All Rights Reserved.
 */
#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>

#include "sim_data_flow/P_msg.pb.h"
#include "controller/base_ctrl.h"
#include "webots_device/w_fork.h"
#include "webots_device/w_wheel.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_imu.h"

// TODO: 统一该定义
#define SERIAL_MSG_BUF 256

namespace VNSim {

class NormalSTController : public BaseController {
   public:
    NormalSTController();
    ~NormalSTController();

    // Manual
    virtual void manualSetState(const std::map<std::string, double> &msg);
    virtual void manualGetState(std::map<std::string, double> &msg);

   protected:
    // task
    void whileSpin();
    void BpReportSpin();
    void Mid360ReportSpin();
    void sendSerialSpin();

   private:
    std::shared_ptr<WLidar> BP_ptr_;
    std::shared_ptr<WLidar> mid360_ptr_;
    std::shared_ptr<WImu> imu_ptr_;
    std::shared_ptr<WWheel> stree_ptr_;
    std::shared_ptr<WWheel> l_ptr_;
    std::shared_ptr<WWheel> r_ptr_;
    std::shared_ptr<WFork> fork_ptr_;

    sim_data_flow::PMsg payload;
    sim_data_flow::PUp payload_Up;
    foxglove::Imu payload_imu;
    uint8_t buf[SERIAL_MSG_BUF];

   public:
    void onRemoteSerialMsg(const char *topic_name,
                           const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim

#endif
