#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>

#include "sim_data_flow/ST_msg.pb.h"
#include "controller/base_ctrl.h"
#include "webots_device/w_fork.h"
#include "webots_device/w_wheel.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_imu.h"
#include "ecal_wrapper.h"

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
    std::shared_ptr<WFork> fork_ptr_;

    EcalWrapper ecal_wrapper_;
    sim_data_flow::STMsg payload;
    sim_data_flow::STUp payload_Up;
    foxglove::Imu payload_imu;
    uint8_t buf[SERIAL_MSG_BUF];

    // private:
    //  void PointCloud2Init(pb::PointCloud2 &pb, int size);

   public:
    void onRemoteSerialMsg(const char *topic_name,
                           const eCAL::SReceiveCallbackData *data);
};

}  // namespace VNSim

#endif
