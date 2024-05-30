#ifndef __WEBOTST_HPP__
#define __WEBOTST_HPP__
#include <thread>

#include "controller/base_ctrl.h"
#include "webots_device/w_fork.h"
#include "webots_device/w_wheel.h"
#include "webots_device/w_lidar.h"
#include "webots_device/w_imu.h"


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

   private:
    std::shared_ptr<WLidar> BP_ptr_;
    std::shared_ptr<WLidar> mid360_ptr_;
    std::shared_ptr<WImu> imu_ptr_;
    std::shared_ptr<WWheel> stree_ptr_;
    std::shared_ptr<WFork> fork_ptr_;
};

}  // namespace VNSim

#endif
